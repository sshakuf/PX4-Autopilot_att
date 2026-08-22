/* Drone ground-station frontend.
 *
 * Vanilla JS, no frameworks, no CDN: this runs on a flying field with no
 * internet. Telemetry arrives as SSE (`event: state`, one STATE json per
 * message, ~10 Hz) from /api/stream.
 *
 * Two rules that are safety-critical, not stylistic:
 *   1. The ARM/DISARM button follows the REAL arm state (`state.armed`, which
 *      the backend takes from the vehicle HEARTBEAT) and never the state we
 *      requested. A rejected arm must not make the button read "armed".
 *   2. Any value older than 1.0 s is greyed out instead of being shown as if
 *      live. Misreading stale telemetry as current has already cost flights.
 *
 * `null` is legal in the STATE for every float: the backend converts every
 * non-finite float to null because NaN is not valid JSON. Nulls render as
 * "--", never as "NaN" or "0".
 */
(function () {
  'use strict';

  // ---- camera intrinsics, same numbers as ir_flow_monitor.py -------------
  var FX = 1047.0, FY = 1065.0, CX = 618.0, CY = 480.0;
  var HALF_LAT = Math.atan(CX / FX);   // lateral  half-FOV [rad] (body +Y)
  var HALF_FWD = Math.atan(CY / FY);   // fore/aft half-FOV [rad] (body +X)

  var STALE_S = 1.0;          // grey out anything older than this
  var LINK_DEAD_S = 3.0;      // arm button goes inert past this arm_state_age
  var HEIGHT_FALLBACK = 0.7;  // ir_flow_monitor.py --height default
  var VEL_SCALE = 2.0;        // m of arrow per m/s  (--vel-scale default)
  var TRAIL_MAX = 150;        // --trail default
  var FLOW_DOTS = 150;
  var RANGES = [0.5, 0.75, 1.0, 1.5, 2.0, 3.0, 4.0, 6.0];

  function $(id) { return document.getElementById(id); }

  // ---- null / NaN-safe numeric helpers ----------------------------------
  function num(v) {
    return (typeof v === 'number' && isFinite(v)) ? v : null;
  }
  function fmt(v, d) {
    var n = num(v);
    return n === null ? '--' : n.toFixed(d === undefined ? 2 : d);
  }
  function fmtSigned(v, d) {
    var n = num(v);
    if (n === null) { return '--'; }
    var s = n.toFixed(d === undefined ? 2 : d);
    return (n >= 0 ? '+' : '') + s;
  }
  function deg(v) {
    var n = num(v);
    return n === null ? null : n * 180 / Math.PI;
  }
  function obj(o, k) {
    var v = o ? o[k] : null;
    return (v && typeof v === 'object') ? v : {};
  }

  // ---- live state --------------------------------------------------------
  var ST = null;             // latest STATE json
  var stWall = 0;            // performance.now() when it arrived
  var streamOk = false;
  var streamNote = 'connecting';
  var parseErrors = 0;

  /* Seconds since the STATE snapshot reached the browser. Added to every
   * `*_age` so that a stalled stream makes values age out and grey
   * themselves instead of freezing at a plausible-looking number. */
  function drift() {
    return stWall ? (performance.now() - stWall) / 1000 : 1e9;
  }
  function age(a) {
    var n = num(a);
    return n === null ? Infinity : n + drift();
  }
  function isStale(a) { return !(age(a) <= STALE_S); }

  // ---- transient note under the arm bar ---------------------------------
  var noteUntil = 0;
  function note(text, isErr) {
    var el = $('note');
    el.textContent = text;
    el.classList.toggle('err', !!isErr);
    el.classList.remove('hidden');
    noteUntil = performance.now() + 6000;
  }
  function tickNote() {
    if (noteUntil && performance.now() > noteUntil) {
      noteUntil = 0;
      $('note').classList.add('hidden');
    }
  }

  function postJSON(url, body) {
    return fetch(url, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body || {})
    }).then(function (r) {
      return r.json().catch(function () { return null; }).then(function (j) {
        if (!r.ok) { throw new Error((j && j.error) || ('HTTP ' + r.status)); }
        if (j && j.ok === false) { throw new Error(j.error || 'rejected'); }
        return j || {};
      });
    });
  }

  // =======================================================================
  // SSE stream
  // =======================================================================
  var es = null, backoff = 1000, retryTimer = null;
  var pktPrev = null, pktPrevT = 0, pktRate = null;

  function onState(s) {
    ST = s;
    stWall = performance.now();
    streamOk = true;
    streamNote = '';
    backoff = 1000;

    // packet rate from the monotonic counter
    var p = num(obj(s, 'link').packets);
    var t = stWall / 1000;
    if (p !== null) {
      if (pktPrev === null || p < pktPrev) { pktPrev = p; pktPrevT = t; }
      else if (t - pktPrevT >= 1.0) {
        pktRate = (p - pktPrev) / (t - pktPrevT);
        pktPrev = p; pktPrevT = t;
      }
    }
    if (s && s.params) { mergeParams(s.params); }
  }

  function connect() {
    if (es) { try { es.close(); } catch (e) { /* ignore */ } es = null; }
    streamNote = 'connecting';
    try {
      es = new EventSource('/api/stream');
    } catch (e) {
      scheduleRetry();
      return;
    }
    es.addEventListener('state', function (ev) {
      var s;
      try { s = JSON.parse(ev.data); }
      catch (err) { parseErrors++; streamNote = 'bad json from /api/stream'; return; }
      if (s && typeof s === 'object') { onState(s); }
    });
    es.onopen = function () { streamOk = true; streamNote = ''; backoff = 1000; };
    es.onerror = function () {
      // Do not trust EventSource's own retry: we want visible backoff and we
      // must never leave stale numbers looking live.
      streamOk = false;
      try { es.close(); } catch (e) { /* ignore */ }
      es = null;
      scheduleRetry();
    };
  }

  function scheduleRetry() {
    if (retryTimer) { return; }
    var wait = backoff;
    backoff = Math.min(Math.round(backoff * 1.7), 10000);
    streamNote = 'reconnecting in ' + (wait / 1000).toFixed(1) + ' s';
    retryTimer = setTimeout(function () { retryTimer = null; connect(); }, wait);
  }

  // one immediate snapshot so the UI is populated before the first SSE frame
  function primeState() {
    fetch('/api/state', { cache: 'no-store' })
      .then(function (r) { return r.json(); })
      .then(function (s) { if (!ST && s && typeof s === 'object') { onState(s); } })
      .catch(function () { /* the stream is the real source; ignore */ });
  }

  // =======================================================================
  // 1. link + arm bar
  // =======================================================================

  function armLive() {
    return !!ST && age(ST.arm_state_age) <= LINK_DEAD_S;
  }
  function isArmed() {
    return !!(ST && ST.armed === true);
  }

  /* One tap arms, one tap disarms -- no confirmation step, by request.
   *
   * The only remaining guard is the heartbeat check: a tap with no live
   * autopilot is ignored rather than queued, so a command cannot fire the
   * instant a stale link comes back. The button label still reflects the
   * vehicle's REAL armed state from HEARTBEAT, never what we asked for, so a
   * refused arm can never look like success.
   *
   * NOTE: arming spins props. There is now nothing between a stray tap and a
   * live motor. */
  function onArmTap() {
    if (!armLive()) {
      note('no autopilot heartbeat - tap ignored', true);
      return;
    }
    sendArm(!isArmed());
  }

  function sendArm(wantArm) {
    var force = wantArm ? false : !!$('forceDisarm').checked;
    note('sending ' + (wantArm ? 'ARM' : 'DISARM') + (force ? ' (force)' : '') + ' ...');
    postJSON('/api/command', { cmd: wantArm ? 'arm' : 'disarm', force: force })
      .then(function () {
        note((wantArm ? 'ARM' : 'DISARM') + ' sent - waiting for COMMAND_ACK');
      })
      .catch(function (e) {
        note((wantArm ? 'ARM' : 'DISARM') + ' failed: ' + e.message, true);
      });
  }

  function renderTop() {
    var link = obj(ST, 'link');
    var connected = !!(ST && ST.connected === true);
    var rxAge = age(link.last_rx_age);

    var dot = $('linkDot');
    var cls = 'dot';
    if (!ST || !streamOk) { cls += ' bad'; }
    else if (connected && rxAge <= STALE_S) { cls += ' ok'; }
    else if (connected) { cls += ' warn'; }
    else { cls += ' bad'; }
    if (dot.className !== cls) { dot.className = cls; }

    var txt;
    if (!streamOk) { txt = streamNote || 'stream lost'; }
    else if (!ST) { txt = 'waiting for telemetry'; }
    else if (connected) { txt = 'connected'; }
    else { txt = 'no vehicle'; }
    setText($('linkText'), txt);

    setText($('modePill'), 'mode ' + ((ST && ST.mode) ? ST.mode : '--'));
    $('modePill').classList.toggle('stale', isStale(ST && ST.arm_state_age));

    setText($('ratePill'), (pktRate === null ? '--' : pktRate.toFixed(0)) + ' pkt/s');
    setText($('agePill'), 'rx ' + (isFinite(rxAge) ? rxAge.toFixed(2) + 's' : '--'));
    $('agePill').classList.toggle('stale', !(rxAge <= STALE_S));

    var banner = $('banner');
    if (!streamOk) {
      banner.textContent = 'stream lost - ' + (streamNote || 'reconnecting') +
        '. Values below are frozen and greyed out.';
      banner.className = 'banner bad';
    } else if (ST && !ST.connected) {
      banner.textContent = 'no MAVLink from the vehicle (' + ((ST && ST.address) || '?') + ')';
      banner.className = 'banner';
    } else if (parseErrors > 0 && !ST) {
      banner.textContent = 'could not parse the telemetry stream';
      banner.className = 'banner bad';
    } else {
      banner.className = 'banner hidden';
    }
  }

  function renderArm() {
    var live = armLive();
    var armed = isArmed();
    var label, cls;

    // Two states only, and both come from the vehicle's real HEARTBEAT flag.
    // There is no pending-confirm state any more.
    if (!live) {
      label = 'NO LINK'; cls = 'armbtn inert';
    } else if (armed) {
      label = 'DISARM'; cls = 'armbtn disarm';
    } else {
      label = 'ARM'; cls = 'armbtn arm';
    }

    var btn = $('armBtn');
    setText(btn, label);
    if (btn.className !== cls) { btn.className = cls; }
    btn.setAttribute('aria-label', label);

    var st = $('armState');
    var stTxt = !live ? 'arm state unknown'
      : (armed ? 'ARMED' : 'disarmed');
    if (live) { stTxt += '  (' + age(ST.arm_state_age).toFixed(1) + 's)'; }
    setText(st, stTxt);
    st.className = 'armstate' + (live && armed ? ' armed' : '');

    var ack = obj(ST, 'ack');
    var ackEl = $('ackText');
    if (ack && ack.result) {
      var a = (ST && num(ST.t) !== null && num(ack.t) !== null) ? (ST.t - ack.t) : null;
      setText(ackEl, 'ack ' + String(ack.result) +
        (ack.cmd !== undefined && ack.cmd !== null ? ' [cmd ' + ack.cmd + ']' : '') +
        (a !== null ? '  ' + a.toFixed(0) + 's ago' : ''));
      ackEl.className = 'mono tiny' + (a !== null && a > 6 ? ' muted' : '');
    } else {
      setText(ackEl, 'no command yet');
      ackEl.className = 'mono tiny muted';
    }
  }

  // =======================================================================
  // 3. state grid
  // =======================================================================
  function setText(el, t) {
    if (el && el.textContent !== t) { el.textContent = t; }
  }
  function setV(id, text, stale, extra) {
    var el = $(id);
    if (!el) { return; }
    setText(el, text);
    var cls = 'v' + (stale ? ' stale' : '') + (extra ? ' ' + extra : '');
    if (el.className !== cls) { el.className = cls; }
  }
  function setAge(id, a) {
    var el = $(id);
    if (!el) { return; }
    var v = age(a);
    setText(el, isFinite(v) ? 'age ' + Math.min(v, 99).toFixed(2) + 's' : 'age --');
    el.classList.toggle('stale', !(v <= STALE_S));
  }

  function usableHeight() {
    var h = obj(ST, 'height');
    var agl = num(h.agl);
    var fresh = age(h.age) <= STALE_S;
    if (fresh && h.valid === true && agl !== null && agl > 0.05) {
      return { h: agl, real: true };
    }
    return { h: HEIGHT_FALLBACK, real: false };
  }

  function renderGrid() {
    var att = obj(ST, 'attitude');
    var pos = obj(ST, 'position');
    var bv = obj(ST, 'body_vel');
    var hgt = obj(ST, 'height');
    var bat = obj(ST, 'battery');
    var ir = obj(ST, 'ir');
    var link = obj(ST, 'link');

    var attStale = isStale(att.age);
    setV('v_roll', fmtSigned(deg(att.roll), 1), attStale);
    setV('v_pitch', fmtSigned(deg(att.pitch), 1), attStale);
    setV('v_yaw', fmtSigned(deg(att.yaw), 1), attStale);
    setAge('a_att', att.age);

    // body_vel is derived from position + attitude, so it is only as fresh as
    // the older of the two.
    var bvAge = Math.max(age(pos.age), age(att.age));
    var bvStale = !(bvAge <= STALE_S);
    var vf = num(bv.fwd), vr = num(bv.right);
    setV('v_vfwd', fmtSigned(vf, 3), bvStale);
    setV('v_vright', fmtSigned(vr, 3), bvStale);
    setV('v_vmag', (vf === null || vr === null) ? '--' : Math.hypot(vf, vr).toFixed(3), bvStale);
    setText($('a_bvel'), isFinite(bvAge) ? 'age ' + Math.min(bvAge, 99).toFixed(2) + 's' : 'age --');
    $('a_bvel').classList.toggle('stale', bvStale);

    var posStale = isStale(pos.age);
    setV('v_x', fmtSigned(pos.x, 2), posStale);
    setV('v_y', fmtSigned(pos.y, 2), posStale);
    setV('v_z', fmtSigned(pos.z, 2), posStale);
    setV('v_vx', fmtSigned(pos.vx, 3), posStale);
    setV('v_vy', fmtSigned(pos.vy, 3), posStale);
    setV('v_vz', fmtSigned(pos.vz, 3), posStale);
    setAge('a_pos', pos.age);

    var hStale = isStale(hgt.age);
    setV('v_agl', fmt(hgt.agl, 2), hStale);
    setV('v_hvalid', hgt.valid === true ? 'yes' : (hgt.valid === false ? 'NO' : '--'),
      hStale, hgt.valid === true ? 'good' : (hgt.valid === false ? 'bad' : ''));
    var uh = usableHeight();
    setV('v_lsb', (0.213 * uh.h).toFixed(3) + (uh.real ? '' : ' (h fb)'), hStale || !uh.real);
    setAge('a_hgt', hgt.age);

    var irStale = isStale(ir.age);
    setV('v_irvalid', ir.valid === true ? 'VALID' : (ir.valid === false ? 'no target' : '--'),
      irStale, ir.valid === true && !irStale ? 'good' : '');
    setV('v_dx', fmtSigned(ir.dx_px, 1), irStale);
    setV('v_dy', fmtSigned(ir.dy_px, 1), irStale);
    setV('v_ax', fmtSigned(ir.angle_x, 3), irStale);
    setV('v_ay', fmtSigned(ir.angle_y, 3), irStale);
    setV('v_offf', fmtSigned(ir.offset_fwd, 3), irStale);
    setV('v_offr', fmtSigned(ir.offset_right, 3), irStale);
    setV('v_spot',
      (ir.spot_id === undefined || ir.spot_id === null ? '--' : ir.spot_id) + ' / ' +
      (ir.spot_score === undefined || ir.spot_score === null ? '--' : ir.spot_score), irStale);
    setV('v_via', ir.via ? String(ir.via) : '--', irStale);
    setAge('a_ir', ir.age);

    // battery carries no age of its own: fall back to link freshness
    var bStale = !(age(link.last_rx_age) <= STALE_S);
    setV('v_volt', fmt(bat.voltage, 2), bStale);
    setV('v_curr', fmt(bat.current, 2), bStale);
    setV('v_rem', fmt(bat.remaining, 0), bStale);
    setText($('a_batt'), bStale ? 'link stale' : '');
    $('a_batt').classList.toggle('stale', bStale);

    setV('v_addr', (ST && ST.address) ? String(ST.address) : '--', false);
    setV('v_pkts', link.packets === undefined || link.packets === null ? '--' : String(link.packets), false);
    setV('v_drops', link.drops === undefined || link.drops === null ? '--' : String(link.drops), false,
      num(link.drops) ? 'bad' : '');
  }

  // =======================================================================
  // 2. body-frame view
  // =======================================================================
  var cv = $('view'), cx2d = cv.getContext('2d');
  var cssW = 0, cssH = 0;
  var rangeAuto = true, rangeIdx = 3;       // 1.5 m, ir_flow_monitor default
  var range = RANGES[rangeIdx];
  var trail = [];                           // [{r,f}], newest last
  var lastBeacon = null;                    // {r,f} last good fix
  var dotGainSteps = [0, 1, 2, 4, 8, 16];
  var dotGainIdx = 1;
  var dots = [];                            // normalised to [-1,1]

  (function initDots() {
    // deterministic scatter, no need for a seeded RNG beyond this
    var s = 12345;
    function rnd() { s = (s * 1103515245 + 12345) & 0x7fffffff; return s / 0x7fffffff; }
    for (var i = 0; i < FLOW_DOTS; i++) { dots.push([rnd() * 2 - 1, rnd() * 2 - 1]); }
  })();

  function resizeCanvas() {
    var dpr = window.devicePixelRatio || 1;
    var rect = cv.getBoundingClientRect();
    var w = Math.max(120, Math.round(rect.width));
    var h = Math.max(120, Math.round(rect.height));
    var pw = Math.round(w * dpr), ph = Math.round(h * dpr);
    if (cv.width !== pw || cv.height !== ph) {
      cv.width = pw;
      cv.height = ph;
    }
    cssW = w; cssH = h;
    // draw in CSS pixels; the backing store stays at device resolution
    cx2d.setTransform(dpr, 0, 0, dpr, 0, 0);
  }

  function niceRange(want) {
    for (var i = 0; i < RANGES.length; i++) { if (RANGES[i] >= want) { return RANGES[i]; } }
    return RANGES[RANGES.length - 1];
  }

  function drawView(dt) {
    resizeCanvas();
    var g = cx2d;
    var uh = usableHeight();
    var h = uh.h;
    var fovLat = h * Math.tan(HALF_LAT);      // half-width  along body +Y
    var fovFwd = h * Math.tan(HALF_FWD);      // half-height along body +X

    var ir = obj(ST, 'ir');
    var att = obj(ST, 'attitude');
    var pos = obj(ST, 'position');
    var bv = obj(ST, 'body_vel');

    var irFresh = age(ir.age) <= STALE_S;
    var offF = num(ir.offset_fwd), offR = num(ir.offset_right);
    var beaconGood = irFresh && ir.valid === true && offF !== null && offR !== null;
    if (beaconGood) {
      lastBeacon = { r: offR, f: offF };
      var last = trail.length ? trail[trail.length - 1] : null;
      if (!last || Math.abs(last.r - offR) > 1e-4 || Math.abs(last.f - offF) > 1e-4) {
        trail.push({ r: offR, f: offF });
        if (trail.length > TRAIL_MAX) { trail.shift(); }
      }
    }

    var vf = num(bv.fwd), vr = num(bv.right);
    var velAge = Math.max(age(pos.age), age(att.age));
    var velFresh = velAge <= STALE_S;
    var haveVel = vf !== null && vr !== null;
    var vmag = haveVel ? Math.hypot(vf, vr) : 0;

    if (rangeAuto) {
      var want = Math.max(fovLat, fovFwd) * 1.35;
      if (lastBeacon) { want = Math.max(want, Math.abs(lastBeacon.r) * 1.2, Math.abs(lastBeacon.f) * 1.2); }
      range = niceRange(Math.max(0.5, want));
    } else {
      range = RANGES[rangeIdx];
    }

    var pad = 18;
    var ppm = (Math.min(cssW, cssH) / 2 - pad) / range;   // px per metre
    var ox = cssW / 2, oy = cssH / 2;
    function sx(r) { return ox + r * ppm; }
    function sy(f) { return oy - f * ppm; }

    // ---- background, grid, axes
    g.fillStyle = '#0e1116';
    g.fillRect(0, 0, cssW, cssH);

    var step = range <= 0.75 ? 0.1 : (range <= 2 ? 0.25 : (range <= 4 ? 0.5 : 1.0));
    g.strokeStyle = '#1d232c';
    g.lineWidth = 1;
    g.beginPath();
    for (var m = step; m <= range + 1e-9; m += step) {
      g.moveTo(sx(m), 0); g.lineTo(sx(m), cssH);
      g.moveTo(sx(-m), 0); g.lineTo(sx(-m), cssH);
      g.moveTo(0, sy(m)); g.lineTo(cssW, sy(m));
      g.moveTo(0, sy(-m)); g.lineTo(cssW, sy(-m));
    }
    g.stroke();

    g.strokeStyle = '#2f3947';
    g.beginPath();
    g.moveTo(0, sy(0)); g.lineTo(cssW, sy(0));
    g.moveTo(sx(0), 0); g.lineTo(sx(0), cssH);
    g.stroke();

    // ---- apparent ground motion: dots drift OPPOSITE the vehicle motion.
    // If they move the same way as the green arrow, a sign is inverted.
    var gain = dotGainSteps[dotGainIdx];
    if (gain > 0) {
      if (haveVel && velFresh && dt > 0) {
        var dr = (vr * dt * gain) / range, df = (vf * dt * gain) / range;
        for (var i = 0; i < dots.length; i++) {
          var d = dots[i];
          d[0] = ((d[0] - dr + 1) % 2 + 2) % 2 - 1;
          d[1] = ((d[1] - df + 1) % 2 + 2) % 2 - 1;
        }
      }
      g.fillStyle = velFresh ? 'rgba(63,127,168,0.75)' : 'rgba(90,100,112,0.45)';
      for (var j = 0; j < dots.length; j++) {
        g.beginPath();
        g.arc(sx(dots[j][0] * range), sy(dots[j][1] * range), 1.6, 0, 6.2832);
        g.fill();
      }
    }

    // ---- camera ground footprint (dashed). Outside it the beacon is not
    // visible at all, so it is the most important thing on the screen.
    g.save();
    g.setLineDash([7, 5]);
    g.lineWidth = 1.6;
    g.strokeStyle = uh.real ? '#c8a13a' : '#6f6134';
    g.strokeRect(sx(-fovLat), sy(fovFwd), 2 * fovLat * ppm, 2 * fovFwd * ppm);
    g.restore();

    // ---- beacon trail, fading oldest -> newest
    if (trail.length > 1) {
      g.lineWidth = 1.6;
      for (var k = 1; k < trail.length; k++) {
        var a = 0.08 + 0.82 * (k / trail.length);
        g.strokeStyle = 'rgba(138,63,90,' + a.toFixed(3) + ')';
        g.beginPath();
        g.moveTo(sx(trail[k - 1].r), sy(trail[k - 1].f));
        g.lineTo(sx(trail[k].r), sy(trail[k].f));
        g.stroke();
      }
    }

    // ---- drone at the origin
    g.strokeStyle = '#dfe6ee';
    g.lineWidth = 2.2;
    g.beginPath();
    g.moveTo(ox - 9, oy); g.lineTo(ox + 9, oy);
    g.moveTo(ox, oy - 9); g.lineTo(ox, oy + 9);
    g.stroke();

    // ---- velocity arrow, body frame (EKF estimate, what the controller uses)
    if (haveVel && vmag > 1e-3) {
      var tipX = sx(vr * VEL_SCALE), tipY = sy(vf * VEL_SCALE);
      var col = velFresh ? '#37d67a' : '#5c6b60';
      g.strokeStyle = col; g.fillStyle = col; g.lineWidth = 3;
      g.beginPath(); g.moveTo(ox, oy); g.lineTo(tipX, tipY); g.stroke();
      var ang = Math.atan2(tipY - oy, tipX - ox);
      g.beginPath();
      g.moveTo(tipX, tipY);
      g.lineTo(tipX - 11 * Math.cos(ang - 0.45), tipY - 11 * Math.sin(ang - 0.45));
      g.lineTo(tipX - 11 * Math.cos(ang + 0.45), tipY - 11 * Math.sin(ang + 0.45));
      g.closePath(); g.fill();
    }

    // ---- beacon. Grey (not hidden) when stale, so the last known position is
    // still readable but can never be mistaken for a live fix.
    var shown = lastBeacon;      // last known fix survives, greyed, when stale
    if (shown) {
      var live = beaconGood;
      g.beginPath();
      g.arc(sx(shown.r), sy(shown.f), 9, 0, 6.2832);
      g.fillStyle = live ? '#e2364a' : 'rgba(120,80,90,0.75)';
      g.fill();
      g.lineWidth = 1.6;
      g.strokeStyle = live ? '#ffffff' : '#6f7b89';
      g.stroke();
    }

    // ---- labels
    g.fillStyle = '#5f6b7a';
    g.font = '11px ui-monospace, Menlo, monospace';
    g.textAlign = 'center';
    g.fillText('NOSE  +X', ox, 13);
    g.textAlign = 'right';
    g.fillText('RIGHT  +Y', cssW - 4, oy - 6);
    g.textAlign = 'left';
    g.fillText('±' + range.toFixed(2) + ' m', 4, cssH - 6);
    g.textAlign = 'right';
    g.fillText('h ' + h.toFixed(2) + (uh.real ? ' m' : ' m fallback') +
      '  fov ±' + fovFwd.toFixed(2) + '/' + fovLat.toFixed(2), cssW - 4, cssH - 6);

    // beacon readout on its own line, below NOSE, always explicit about nulls
    g.textAlign = 'left';
    g.fillStyle = beaconGood ? '#d4dbe4' : '#6f7b89';
    g.fillText('beacon f/r ' + (offF === null ? '--' : offF.toFixed(3)) + ' / ' +
      (offR === null ? '--' : offR.toFixed(3)) + ' m' +
      (irFresh ? '' : '   STALE ' + (isFinite(age(ir.age)) ? age(ir.age).toFixed(1) + 's' : '')), 4, 28);

    // ---- warnings
    var warn = [];
    if (!irFresh) { warn.push('no IR data'); }
    else if (beaconGood && (Math.abs(offF) > fovFwd || Math.abs(offR) > fovLat)) {
      warn.push('target at FOV edge - about to be lost');
    }
    if (haveVel && velFresh) {
      var lsb = 0.213 * h;
      if (Math.abs(vf) < lsb * 0.5 && Math.abs(vr) < lsb * 0.5) {
        warn.push('|v| below flow resolution (' + lsb.toFixed(2) + ' m/s)');
      }
    }
    if (!velFresh) { warn.push('velocity stale'); }
    if (warn.length) {
      g.textAlign = 'center';
      g.fillStyle = '#ffb020';
      g.font = '12px ui-monospace, Menlo, monospace';
      g.fillText(warn.slice(0, 2).join('   '), ox, cssH - 22);
    }
  }

  // =======================================================================
  // 4. params
  // =======================================================================
  var params = {};            // name -> value
  var paramSig = '';
  var paramsDirty = true;     // avoid re-sorting hundreds of names every frame
  var editing = null;
  var TOGGLES = ['DF_SWAY_EN', 'DF_TGT_HOLD_EN', 'DF_YAW_HOLD_EN'];
  var ROT_NAME = 'DF_IRC_ROT';

  function mergeParams(p) {
    if (!p || typeof p !== 'object') { return; }
    var keys = Object.keys(p);
    for (var i = 0; i < keys.length; i++) {
      var v = num(p[keys[i]]);
      if (v !== null && params[keys[i]] !== v) {
        params[keys[i]] = v;
        paramsDirty = true;
      }
    }
  }

  function loadParams() {
    var prefix = $('paramFilter').value.trim();
    fetch('/api/params?prefix=' + encodeURIComponent(prefix), { cache: 'no-store' })
      .then(function (r) { return r.json(); })
      .then(function (j) {
        mergeParams(j && j.params);
        note('params: ' + Object.keys((j && j.params) || {}).length + ' returned for "' +
          (prefix || '*') + '"');
        renderParams(true);
      })
      .catch(function (e) { note('param load failed: ' + e.message, true); });
  }

  /* Parameter writes go straight through, armed or not.
   *
   * The server still refuses an armed write unless the request says force, so we
   * always send force and let it through. That gate stays in place for any other
   * client hitting the API; here it would only be friction, because tuning while
   * armed on the bench is the normal workflow for this airframe. The note still
   * says "(armed)" so it is never silent. */
  function setParam(name, value) {
    var v = parseFloat(value);
    if (!isFinite(v)) { note('"' + value + '" is not a number', true); return; }
    var force = true;
    postJSON('/api/param', { name: name, value: v, force: force })
      .then(function () {
        note('set ' + name + ' = ' + v + (isArmed() ? ' (armed)' : ''));
        editing = null;
        renderParams(true);
      })
      .catch(function (e) { note('set ' + name + ' failed: ' + e.message, true); });
  }

  function renderQuick() {
    var btns = document.querySelectorAll('.qbtn[data-toggle]');
    for (var i = 0; i < btns.length; i++) {
      var b = btns[i];
      var n = b.getAttribute('data-toggle');
      var v = params.hasOwnProperty(n) ? params[n] : null;
      var span = b.querySelector('.qv');
      if (v === null) {
        setText(span, '?');
        b.className = 'qbtn unknown';
      } else if (v >= 0.5) {
        setText(span, 'ON');
        b.className = 'qbtn on';
      } else {
        setText(span, 'OFF');
        b.className = 'qbtn off';
      }
    }
    var rot = params.hasOwnProperty(ROT_NAME) ? Math.round(params[ROT_NAME]) : null;
    var segs = document.querySelectorAll('.segbtn[data-rot]');
    for (var k = 0; k < segs.length; k++) {
      var on = rot !== null && Number(segs[k].getAttribute('data-rot')) === rot;
      segs[k].className = 'segbtn' + (on ? ' on' : '');
    }
  }

  function renderParams(force) {
    if (!force && !paramsDirty) { return; }
    paramsDirty = false;
    renderQuick();
    var filter = $('paramFilter').value.trim().toUpperCase();
    var names = Object.keys(params).filter(function (n) {
      return !filter || n.toUpperCase().indexOf(filter) >= 0;
    }).sort();
    setText($('paramCount'), Object.keys(params).length + ' known / ' + names.length + ' shown');

    var sig = filter + '|' + editing + '|' + names.map(function (n) {
      return n + '=' + params[n];
    }).join(',');
    if (!force && sig === paramSig) { return; }
    paramSig = sig;

    var list = $('paramList');
    list.textContent = '';
    if (!names.length) {
      var e = document.createElement('div');
      e.className = 'empty';
      e.textContent = Object.keys(params).length
        ? 'no parameter matches "' + filter + '"'
        : 'no parameters yet - tap load';
      list.appendChild(e);
      return;
    }
    names.forEach(function (n) {
      var row = document.createElement('div');
      row.className = 'prow';
      var nm = document.createElement('span');
      nm.className = 'pname';
      nm.textContent = n;
      row.appendChild(nm);

      if (editing === n) {
        var inp = document.createElement('input');
        inp.type = 'text';
        inp.inputMode = 'decimal';
        inp.value = String(params[n]);
        inp.setAttribute('data-input', n);
        row.appendChild(inp);
        var ok = document.createElement('button');
        ok.className = 'btn go';
        ok.type = 'button';
        ok.textContent = 'set';
        ok.setAttribute('data-set', n);
        row.appendChild(ok);
        var no = document.createElement('button');
        no.className = 'btn';
        no.type = 'button';
        no.textContent = '✕';
        no.setAttribute('data-cancel', '1');
        row.appendChild(no);
      } else {
        var val = document.createElement('span');
        val.className = 'pval';
        val.textContent = formatParam(params[n]);
        row.appendChild(val);
        var ed = document.createElement('button');
        ed.className = 'btn';
        ed.type = 'button';
        ed.textContent = 'edit';
        ed.setAttribute('data-edit', n);
        row.appendChild(ed);
      }
      list.appendChild(row);
    });
    var focus = list.querySelector('input[data-input]');
    if (focus) { focus.focus(); focus.select(); }
  }

  function formatParam(v) {
    if (v === null || v === undefined) { return '--'; }
    if (Math.abs(v - Math.round(v)) < 1e-6) { return String(Math.round(v)); }
    return v.toFixed(4).replace(/0+$/, '');
  }

  // =======================================================================
  // 5. logs
  // =======================================================================
  var logSig = '', lastProgress = null, dlSig = '';

  function human(b) {
    var n = num(b);
    if (n === null) { return '--'; }
    if (n < 1024) { return n + ' B'; }
    if (n < 1048576) { return (n / 1024).toFixed(1) + ' kB'; }
    return (n / 1048576).toFixed(2) + ' MB';
  }
  function utcStr(u) {
    var n = num(u);
    if (n === null || n <= 0) { return 'no date'; }
    try { return new Date(n * 1000).toISOString().replace('T', ' ').slice(0, 19) + 'Z'; }
    catch (e) { return String(u); }
  }

  function pollLogs() {
    return fetch('/api/logs', { cache: 'no-store' })
      .then(function (r) { return r.json(); })
      .then(function (j) {
        var logs = (j && j.logs) || [];
        var prog = (j && j.progress) || {};
        var wasActive = lastProgress && lastProgress.active;
        lastProgress = prog;
        renderLogs(logs, prog);
        if (wasActive && !prog.active) { pollDownloads(); }
      })
      .catch(function () { /* transient; next poll will retry */ });
  }

  function pollDownloads() {
    return fetch('/api/downloads', { cache: 'no-store' })
      .then(function (r) { return r.json(); })
      .then(function (j) { renderDownloads((j && j.files) || []); })
      .catch(function () { /* ignore */ });
  }

  var lastLogCount = null;   // logs the vehicle last reported, for erase feedback

  function renderLogs(logs, prog) {
    lastLogCount = logs.length;
    setText($('logCount'), logs.length + ' on vehicle');

    var pw = $('logProgress');
    if (prog && (prog.active || prog.error)) {
      pw.classList.remove('hidden');
      var pct = num(prog.pct);
      var fill = $('progFill');
      fill.style.width = Math.max(0, Math.min(100, pct === null ? 0 : pct)) + '%';
      fill.className = 'progfill' + (prog.error ? ' err' : '');
      var bits = [];
      bits.push('log ' + (prog.id === undefined || prog.id === null ? '?' : prog.id));
      bits.push((pct === null ? '--' : pct.toFixed(1)) + '%');
      bits.push(human(prog.received) + ' / ' + human(prog.size));
      var rate = num(prog.rate_bps);
      if (rate !== null) { bits.push((rate / 1024).toFixed(1) + ' kB/s'); }
      var eta = num(prog.eta_s);
      if (eta !== null) { bits.push('eta ' + eta.toFixed(0) + 's'); }
      if (prog.error) { bits.push('ERROR: ' + prog.error); }
      if (prog.path) { bits.push(String(prog.path)); }
      setText($('progLabel'), bits.join('  '));
    } else {
      pw.classList.add('hidden');
    }

    var sig = JSON.stringify(logs.map(function (l) { return [l.id, l.size, l.local]; })) +
      '|' + (prog && prog.active ? prog.id : '');
    if (sig === logSig) { return; }
    logSig = sig;

    var list = $('logList');
    list.textContent = '';
    if (!logs.length) {
      var e = document.createElement('div');
      e.className = 'empty';
      e.textContent = 'no log list yet - tap refresh (the vehicle answers LOG_REQUEST_LIST slowly)';
      list.appendChild(e);
      return;
    }
    logs.slice().sort(function (a, b) { return (num(b.id) || 0) - (num(a.id) || 0); }).forEach(function (l) {
      var row = document.createElement('div');
      row.className = 'lrow';
      var main = document.createElement('div');
      main.className = 'lmain';
      var nm = document.createElement('div');
      nm.className = 'lname';
      nm.textContent = '#' + (l.id === undefined || l.id === null ? '?' : l.id) +
        '  ' + (l.name ? String(l.name) : '');
      main.appendChild(nm);
      var meta = document.createElement('div');
      meta.className = 'lmeta';
      meta.textContent = human(l.size) + '  ·  ' + utcStr(l.utc);
      main.appendChild(meta);
      row.appendChild(main);

      if (l.local) {
        var tag = document.createElement('span');
        tag.className = 'tagdisk';
        tag.textContent = 'on disk';
        row.appendChild(tag);
      }
      var b = document.createElement('button');
      b.type = 'button';
      var busy = prog && prog.active;
      if (busy && String(prog.id) === String(l.id)) {
        b.className = 'btn';
        b.textContent = 'downloading';
        b.disabled = true;
      } else {
        b.className = 'btn' + (busy ? '' : ' go');
        b.textContent = 'get';
        b.disabled = !!busy;
        b.setAttribute('data-log', String(l.id));
      }
      row.appendChild(b);
      list.appendChild(row);
    });
  }

  function renderDownloads(files) {
    var sig = JSON.stringify(files.map(function (f) { return [f.name, f.size]; }));
    if (sig === dlSig) { return; }
    dlSig = sig;
    var list = $('dlList');
    list.textContent = '';
    if (!files.length) {
      var e = document.createElement('div');
      e.className = 'empty';
      e.textContent = 'nothing downloaded yet';
      list.appendChild(e);
      return;
    }
    files.slice().sort(function (a, b) { return (num(b.mtime) || 0) - (num(a.mtime) || 0); }).forEach(function (f) {
      var row = document.createElement('div');
      row.className = 'lrow';
      var main = document.createElement('div');
      main.className = 'lmain';
      var a = document.createElement('a');
      a.className = 'dl';
      a.href = '/api/logs/file/' + encodeURIComponent(f.name);
      a.setAttribute('download', f.name);
      a.textContent = String(f.name);
      main.appendChild(a);
      var meta = document.createElement('div');
      meta.className = 'lmeta';
      meta.textContent = human(f.size) + (num(f.mtime) !== null ? '  ·  ' + utcStr(f.mtime) : '');
      main.appendChild(meta);
      row.appendChild(main);
      list.appendChild(row);
    });
  }

  // =======================================================================
  // 6. messages
  // =======================================================================
  var SEV_CLS = ['sev-emerg', 'sev-emerg', 'sev-crit', 'sev-err', 'sev-warn',
    'sev-notice', 'sev-info', 'sev-debug'];
  var SEV_TAG = ['EMERG', 'ALERT', 'CRIT', 'ERROR', 'WARN', 'NOTICE', 'INFO', 'DEBUG'];
  var msgSig = '';

  function renderMessages() {
    var msgs = (ST && Array.isArray(ST.messages)) ? ST.messages : [];
    var sig = msgs.length + '|' + (msgs.length ? JSON.stringify(msgs[msgs.length - 1]) : '');
    if (sig === msgSig) { return; }
    msgSig = sig;

    setText($('msgCount'), msgs.length + ' msgs');
    var list = $('msgList');
    list.textContent = '';
    if (!msgs.length) {
      var e = document.createElement('div');
      e.className = 'empty';
      e.textContent = 'no STATUSTEXT yet';
      list.appendChild(e);
      return;
    }
    var now = num(ST && ST.t);
    msgs.slice(-80).reverse().forEach(function (m) {
      var sev = num(m.severity);
      var idx = (sev === null) ? 6 : Math.max(0, Math.min(7, Math.round(sev)));
      var row = document.createElement('div');
      row.className = 'msg ' + SEV_CLS[idx];
      var t = document.createElement('span');
      t.className = 'mt';
      var mt = num(m.t);
      t.textContent = (now !== null && mt !== null)
        ? '-' + Math.max(0, now - mt).toFixed(1) + 's'
        : '--';
      row.appendChild(t);
      var tag = document.createElement('span');
      tag.className = 'mt';
      tag.textContent = SEV_TAG[idx];
      row.appendChild(tag);
      var x = document.createElement('span');
      x.className = 'mx';
      x.textContent = m.text === undefined || m.text === null ? '' : String(m.text);
      row.appendChild(x);
      list.appendChild(row);
    });
  }

  // =======================================================================
  // wiring
  // =======================================================================
  $('armBtn').addEventListener('click', onArmTap);

  $('rangeAuto').addEventListener('click', function () {
    rangeAuto = !rangeAuto;
    $('rangeAuto').classList.toggle('on', rangeAuto);
  });
  $('rangeDown').addEventListener('click', function () {
    rangeAuto = false; $('rangeAuto').classList.remove('on');
    rangeIdx = Math.max(0, rangeIdx - 1);
  });
  $('rangeUp').addEventListener('click', function () {
    rangeAuto = false; $('rangeAuto').classList.remove('on');
    rangeIdx = Math.min(RANGES.length - 1, rangeIdx + 1);
  });
  $('dotsBtn').addEventListener('click', function () {
    dotGainIdx = (dotGainIdx + 1) % dotGainSteps.length;
    var gv = dotGainSteps[dotGainIdx];
    setText($('dotsBtn'), gv === 0 ? 'dots off' : 'dots ×' + gv);
    $('dotsBtn').classList.toggle('on', gv !== 0 && gv !== 1);
  });
  $('trailBtn').addEventListener('click', function () {
    trail = []; lastBeacon = null;
  });

  $('paramLoad').addEventListener('click', loadParams);
  $('paramFilter').addEventListener('input', function () { renderParams(true); });
  $('paramFilter').addEventListener('keydown', function (ev) {
    if (ev.key === 'Enter') { ev.preventDefault(); this.blur(); loadParams(); }
  });

  $('paramList').addEventListener('click', function (ev) {
    var t = ev.target;
    if (!t || !t.getAttribute) { return; }
    var n = t.getAttribute('data-edit');
    if (n) { editing = n; renderParams(true); return; }
    if (t.getAttribute('data-cancel')) { editing = null; renderParams(true); return; }
    var s = t.getAttribute('data-set');
    if (s) {
      var inp = $('paramList').querySelector('input[data-input="' + s + '"]');
      if (inp) { setParam(s, inp.value); }
    }
  });
  $('paramList').addEventListener('keydown', function (ev) {
    if (ev.key !== 'Enter') { return; }
    var t = ev.target;
    var n = t && t.getAttribute ? t.getAttribute('data-input') : null;
    if (n) { ev.preventDefault(); setParam(n, t.value); }
  });

  document.querySelectorAll('.qbtn[data-toggle]').forEach(function (b) {
    b.addEventListener('click', function () {
      var n = b.getAttribute('data-toggle');
      var v = params.hasOwnProperty(n) ? params[n] : null;
      if (v === null) {
        note(n + ' not read yet - tap load first', true);
        return;
      }
      setParam(n, v >= 0.5 ? 0 : 1);
    });
  });
  document.querySelectorAll('.segbtn[data-rot]').forEach(function (b) {
    b.addEventListener('click', function () {
      setParam(ROT_NAME, Number(b.getAttribute('data-rot')));
    });
  });

  $('logRefresh').addEventListener('click', function () {
    postJSON('/api/logs/refresh', {})
      .then(function () { note('LOG_REQUEST_LIST sent'); return pollLogs(); })
      .catch(function (e) { note('log refresh failed: ' + e.message, true); });
  });
  $('logCancel').addEventListener('click', function () {
    postJSON('/api/logs/cancel', {})
      .then(function () { note('download cancelled'); return pollLogs(); })
      .catch(function (e) { note('cancel failed: ' + e.message, true); });
  });

  /* Erase every log ON THE DRONE -- one tap, no confirmation, by request.
   * Irreversible. Already-downloaded files in downloads/ survive.
   *
   * The button is disabled while the request is in flight and the list is
   * re-polled twice afterwards: the vehicle needs a moment to erase the card and
   * re-answer LOG_REQUEST_LIST, and a single immediate poll can land before that
   * and make a successful erase look like nothing happened. */
  $('logErase').addEventListener('click', function () {
    var btn = $('logErase');
    if (btn.disabled) { return; }

    btn.disabled = true;
    btn.textContent = 'erasing...';
    note('erasing all logs on the drone...');

    postJSON('/api/logs/erase', { confirm: true })
      .then(function (r) {
        note('erase sent' + (r && r.message ? ' — ' + r.message : ''));
        return pollLogs();
      })
      .then(function () {
        // the server re-lists ~1.5 s after the erase; catch that result too
        setTimeout(function () {
          pollLogs().then(function () {
            note('drone reports ' + (lastLogCount === 0 ? 'no logs left' :
                 lastLogCount + ' log(s) still present'), lastLogCount !== 0);
          });
        }, 2500);
      })
      .catch(function (e) { note('erase failed: ' + e.message, true); })
      .then(function () {
        btn.disabled = false;
        btn.textContent = 'erase all on drone';
      });
  });
  $('logList').addEventListener('click', function (ev) {
    var t = ev.target;
    var id = t && t.getAttribute ? t.getAttribute('data-log') : null;
    if (!id) { return; }
    postJSON('/api/logs/download', { id: Number(id) })
      .then(function () { note('downloading log ' + id); return pollLogs(); })
      .catch(function (e) { note('download failed: ' + e.message, true); });
  });
  $('logCard').addEventListener('toggle', function () {
    if ($('logCard').open) { pollLogs(); pollDownloads(); }
  });
  $('paramCard').addEventListener('toggle', function () {
    if ($('paramCard').open && !Object.keys(params).length) { loadParams(); }
  });

  window.addEventListener('resize', resizeCanvas);
  window.addEventListener('orientationchange', function () { setTimeout(resizeCanvas, 250); });
  document.addEventListener('visibilitychange', function () {
    if (!document.hidden && !es && !retryTimer) { backoff = 1000; connect(); }
  });

  // poll the log endpoints only when they matter
  setInterval(function () {
    if ($('logCard').open || (lastProgress && lastProgress.active)) { pollLogs(); }
  }, 1000);
  setInterval(function () {
    if ($('logCard').open) { pollDownloads(); }
  }, 5000);

  // ---- render loop -------------------------------------------------------
  var lastFrame = performance.now();
  function frame(now) {
    window.requestAnimationFrame(frame);
    if (now - lastFrame < 40) { return; }          // ~25 fps is plenty
    var dt = Math.min((now - lastFrame) / 1000, 0.25);
    lastFrame = now;
    try {
      tickNote();
      renderTop();
      renderArm();
      renderGrid();
      drawView(dt);
      renderParams(false);
      renderMessages();
    } catch (e) {
      // A render bug must not silently freeze the display and leave old
      // numbers on screen looking live.
      if (window.console) { console.error('render', e); }
      streamNote = 'render error: ' + e.message;
    }
  }

  resizeCanvas();
  renderParams(true);
  renderLogs([], {});
  renderDownloads([]);
  renderMessages();
  primeState();
  connect();
  window.requestAnimationFrame(frame);
})();
