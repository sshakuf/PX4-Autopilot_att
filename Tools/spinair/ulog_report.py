#!/usr/bin/env python3
"""SpinAir ulog digest - one-shot, token-efficient flight log report.

Usage:
  ulog_report.py                # newest log in the QGC Daily/Logs folder
  ulog_report.py <file.ulg>
  ulog_report.py --window 10 20 # restrict stats to t=[10,20) seconds
  ulog_report.py --table target_hold_status:offset_n,offset_e,vel_cmd_n,vel_cmd_e --step 0.5

Encodes the standard checks for this project: firmware hash, param-wipe
detection against drone_params_default.params, yaw health, motor saturation,
IR camera / target-hold behavior incl. an offset-sign-inversion heuristic.
"""

import argparse
import glob
import os
import subprocess
import sys

import numpy as np
from pyulog import ULog

LOG_DIR = os.path.expanduser("~/Documents/QGroundControl Daily/Logs")
REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
GOLDEN = os.path.join(REPO, "drone_params_default.params")

# params worth flagging when they differ from the golden file
WATCH_PREFIXES = ("DF_", "MC_", "MPC_", "CA_ROTOR", "SER_TEL", "SENS_")


def newest_log():
    files = sorted(glob.glob(os.path.join(LOG_DIR, "*.ulg")), key=os.path.getmtime)
    if not files:
        sys.exit(f"no .ulg files in {LOG_DIR}")
    return files[-1]


def load_golden():
    if not os.path.isfile(GOLDEN):
        return {}
    out = {}
    for line in open(GOLDEN):
        if line.startswith("#"):
            continue
        parts = line.split("\t")
        if len(parts) >= 5:
            out[parts[2]] = float(parts[3])
    return out


def fmt(v, nd=2):
    if v is None or (isinstance(v, float) and not np.isfinite(v)):
        return "nan"
    return f"{v:.{nd}f}"


class Report:
    def __init__(self, path, window):
        self.u = ULog(path)
        self.path = path
        self.window = window
        att = self.get("vehicle_attitude")
        self.t0 = att.data["timestamp"][0] / 1e6 if att else 0.0

    def get(self, name, mi=0):
        return next((d for d in self.u.data_list if d.name == name and d.multi_id == mi), None)

    def t(self, d):
        return d.data["timestamp"] / 1e6 - self.t0

    def mask(self, tt):
        if self.window:
            return (tt >= self.window[0]) & (tt < self.window[1])
        return np.ones(len(tt), dtype=bool)

    # ---------------- sections ----------------
    def meta(self):
        u = self.u
        dur = (u.last_timestamp - u.start_timestamp) / 1e6
        sw = u.msg_info_dict.get("ver_sw", "?")[:10]
        head = ""
        try:
            h = subprocess.run(["git", "-C", REPO, "rev-parse", "HEAD"],
                               capture_output=True, text=True, timeout=5).stdout.strip()
            head = "MATCHES repo HEAD" if h.startswith(sw) else f"repo HEAD is {h[:10]} (DIFFERENT!)"
        except Exception:
            pass
        vs = self.get("vehicle_status")
        navs = sorted(set(vs.data["nav_state"])) if vs else []
        armed_frac = float((vs.data["arming_state"] == 2).mean()) if vs else 0.0
        print(f"== {os.path.basename(self.path)}  dur {dur:.0f}s  fw {sw} {head}")
        print(f"   nav_states {navs}  armed {armed_frac*100:.0f}% of log"
              + (f"  [window {self.window[0]}..{self.window[1]}s]" if self.window else ""))

    def params(self):
        golden = load_golden()
        if not golden:
            print("-- params: no golden file found")
            return
        P = self.u.initial_parameters
        diffs = []
        for k, gv in golden.items():
            if not k.startswith(WATCH_PREFIXES) or k not in P:
                continue
            v = float(P[k])
            if abs(v - gv) > max(1e-4, abs(gv) * 1e-3):
                diffs.append((k, v, gv))
        if not diffs:
            print("-- params: match golden file")
        else:
            print(f"-- params: {len(diffs)} differ from golden (log vs golden):")
            for k, v, gv in diffs[:15]:
                print(f"   {k} = {fmt(v,4)} (golden {fmt(gv,4)})")
            if len(diffs) > 15:
                print(f"   ... +{len(diffs)-15} more")
        ch = self.u.changed_parameters
        if ch:
            print(f"   in-flight changes: " + ", ".join(f"{n}={v}" for _, n, v in ch[:8]))

    def yaw(self):
        att = self.get("vehicle_attitude")
        if att is None:
            return
        q0, q1, q2, q3 = (att.data["q[%d]" % i] for i in range(4))
        yaw = np.arctan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 ** 2 + q3 ** 2))
        ta = self.t(att)
        m = self.mask(ta)
        tgt = np.radians(float(self.u.initial_parameters.get("DF_YAW_HOLD", 0.0)))
        en = self.u.initial_parameters.get("DF_YAW_HOLD_EN", 0)
        err = np.degrees(np.arctan2(np.sin(tgt - yaw[m]), np.cos(tgt - yaw[m])))
        av = self.get("vehicle_angular_velocity")
        yr = ""
        if av is not None:
            mm = self.mask(self.t(av))
            yr = f"  yawrate max {np.degrees(np.abs(av.data['xyz[2]'][mm])).max():.0f}d/s"
        cas = self.get("control_allocator_status")
        un = ""
        if cas is not None:
            mm = self.mask(self.t(cas))
            uz = np.abs(cas.data["unallocated_torque[2]"][mm])
            un = f"  unalloc_z mean {uz.mean():.3f} max {uz.max():.2f}"
        hold = f"hold_en={en} err mean {np.abs(err).mean():.1f} max {np.abs(err).max():.0f}deg" if en else "hold disabled"
        print(f"-- yaw: {hold}{yr}{un}")

    def motors(self):
        mot = self.get("actuator_motors")
        if mot is None:
            return
        tm = self.t(mot)
        m = self.mask(tm)
        arr = np.vstack([mot.data["control[%d]" % i][m] for i in range(4)])
        mx = np.nanmax(arr, axis=0)
        sat = float((mx > 0.98).mean())
        print(f"-- motors: per-motor max [{', '.join(fmt(np.nanmax(arr[i])) for i in range(4))}]"
              f"  any>0.98 {sat*100:.0f}% of time")

    def ir(self):
        ir = self.get("ir_camera_report")
        if ir is None:
            print("-- ir_cam: topic missing")
            return
        tt = self.t(ir)
        m = self.mask(tt)
        v = ir.data["valid"][m].astype(bool)
        n = int(m.sum())
        drops = int(np.sum(np.diff(v.astype(int)) == -1))
        line = f"-- ir_cam: {n} msgs, valid {v.mean()*100:.0f}%, {drops} dropouts"
        if v.sum():
            dx = ir.data["dx_px"][m][v]; dy = ir.data["dy_px"][m][v]
            ax = np.degrees(ir.data["angle_x"][m][v]); ay = np.degrees(ir.data["angle_y"][m][v])
            line += (f" | dx {dx.min():.0f}..{dx.max():.0f}px dy {dy.min():.0f}..{dy.max():.0f}px"
                     f" | ang_x {np.nanmin(ax):.1f}..{np.nanmax(ax):.1f} ang_y {np.nanmin(ay):.1f}..{np.nanmax(ay):.1f}deg")
        print(line)

    def target_hold(self):
        th = self.get("target_hold_status")
        if th is None:
            print("-- target_hold: topic missing")
            return
        tt = self.t(th)
        m = self.mask(tt)
        act = th.data["active"][m].astype(bool)
        print(f"-- target_hold: active {act.mean()*100:.0f}%  "
              f"captures {int(np.sum(np.diff(act.astype(int)) == 1))}  "
              f"losses {int(np.sum(np.diff(act.astype(int)) == -1))}  "
              f"height {fmt(np.nanmedian(th.data['height'][m]))}m")
        if act.sum() < 10:
            return
        on = th.data["offset_n"][m]; oe = th.data["offset_e"][m]
        vn = th.data["vel_cmd_n"][m]; ve = th.data["vel_cmd_e"][m]
        inn = th.data["integral_n"][m]; ine = th.data["integral_e"][m]
        print(f"   offset_n {np.nanmin(on[act]):.2f}..{np.nanmax(on[act]):.2f}m"
              f"  offset_e {np.nanmin(oe[act]):.2f}..{np.nanmax(oe[act]):.2f}m"
              f"  |cmd| max {np.nanmax(np.hypot(vn[act], ve[act])):.2f}m/s"
              f"  integ end ({fmt(inn[act][-1])},{fmt(ine[act][-1])})")
        # sign heuristic: while active, commands should SHRINK the offset:
        # corr( d(offset)/dt, vel_cmd ) < 0 is healthy; > 0 means inverted sign
        ta = tt[m]
        for name, off, cmd in (("N", on, vn), ("E", oe, ve)):
            a = act & np.isfinite(off) & np.isfinite(cmd)
            if a.sum() < 20:
                continue
            doff = np.gradient(off[a], ta[a])
            c = cmd[a]
            if np.std(doff) < 1e-4 or np.std(c) < 1e-4:
                continue
            corr = float(np.corrcoef(doff, c)[0, 1])
            verdict = "OK (cmds shrink offset)" if corr < -0.1 else (
                "SIGN SUSPECT: offset grows along command!" if corr > 0.1 else "inconclusive (little motion)")
            print(f"   sign check {name}: corr(d_offset, cmd) = {corr:+.2f} -> {verdict}")

    def estimator(self):
        lp = self.get("vehicle_local_position")
        if lp is None:
            return
        m = self.mask(self.t(lp))
        fl = self.get("vehicle_optical_flow")
        fq = ""
        if fl is not None:
            mm = self.mask(self.t(fl))
            if mm.sum():
                fq = f"  flow quality mean {fl.data['quality'][mm].mean():.0f}"
        print(f"-- estimator: dist_bottom {lp.data['dist_bottom'][m].min():.2f}..{lp.data['dist_bottom'][m].max():.2f}m"
              f" valid {lp.data['dist_bottom_valid'][m].mean()*100:.0f}%"
              f"  xy_valid {lp.data['xy_valid'][m].mean()*100:.0f}%"
              f"  v {np.hypot(lp.data['vx'][m], lp.data['vy'][m]).max():.2f}m/s max{fq}")

    def table(self, spec, step):
        name, _, fields = spec.partition(":")
        d = self.get(name)
        if d is None:
            print(f"-- table: topic {name} missing")
            return
        fields = fields.split(",") if fields else [k for k in d.data if k != "timestamp"][:6]
        tt = self.t(d)
        m = self.mask(tt)
        tt = tt[m]
        print(f"-- {name} every {step}s:")
        print("   t[s]  " + "  ".join(f"{f[:10]:>10s}" for f in fields))
        for tq in np.arange(tt[0], tt[-1], step):
            row = [f"{np.interp(tq, tt, d.data[f][m]):10.3f}" for f in fields]
            print(f"  {tq:6.1f} " + " ".join(row))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log", nargs="?", help="ulog path (default: newest in QGC Daily/Logs)")
    ap.add_argument("--window", nargs=2, type=float, metavar=("T0", "T1"))
    ap.add_argument("--table", action="append", default=[], metavar="TOPIC:f1,f2")
    ap.add_argument("--step", type=float, default=0.5)
    args = ap.parse_args()

    path = args.log or newest_log()
    r = Report(path, tuple(args.window) if args.window else None)
    r.meta()
    r.params()
    r.yaw()
    r.motors()
    r.estimator()
    r.ir()
    r.target_hold()
    for spec in args.table:
        r.table(spec, args.step)


if __name__ == "__main__":
    main()
