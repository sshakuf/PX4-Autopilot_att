# LSDT IR Camera — PX4 Onboard Driver Spec

**Audience: an LLM (or engineer) writing PX4 flight-controller firmware, with
zero prior context on this project.** This document is self-contained. It
gives you everything needed to write a PX4 module that reads the LSDT IR
spot-detection camera directly from a flight-controller UART, parses its
protocol, and extracts target `dx`/`dy` — without any companion computer in
the loop.

Everything under "Validated facts" and "Wire protocol" in this document was
proven against real hardware (not just read off a vendor datasheet) by a
companion-computer implementation already built and tested in this repo:
`pipeline/ir_camera_protocol.py`, `pipeline/modules/ir_camera_module.py`,
`test/test_lsdt_camera_msp.py`. Where this doc says "confirmed," it means:
someone actually plugged in the camera, ran a parser, and saw correct
decoded output. Treat it as ground truth, not a guess.

The one thing this doc does **not** give you: an off-the-shelf, drop-in PX4
module file. It gives you the protocol, the parsing algorithm, and
architectural guidance so you (or a future LLM session) can write that module
correctly on the first attempt, verify it against the exact same expected
byte sequences documented here, and make an informed choice about how the
parsed data should flow into PX4's control/estimation stack.

---

## 1. What this camera is, and why you're reading direct from a flight-controller UART

The LSDT camera is a spot/target-detection IR camera module (vendor: TriEye,
document `docs/LSDT_Camera_MSP_V2_UART_Interface_Protocol.pdf` in this repo —
"LSDT Camera MSP V2 UART Interface Protocol," doc number TE-PRO-IPS-00019).
It runs its own onboard detection algorithm and reports the pixel position of
a detected target ("spot") over a UART link, using a command protocol called
MSP V2 (the same acronym/style as Betaflight/iNav's MSP, but this is the
vendor's own command set layered on that wire format — not flight-controller
telemetry).

**Why read it directly from the flight controller instead of through a
companion computer:** this project already has a companion-computer path
(Raspberry Pi or Mac reads the camera over USB-serial, runs a Python pipeline
module, and feeds `dx`/`dy` into a PID controller that outputs RC commands —
see `pipeline/modules/ir_camera_module.py` and
`pipeline/modules/target_position_module.py` if you want prior art for the
*algorithm* side, though none of that Python code is relevant to writing C
for PX4). Reading the camera directly on an FC UART removes the
serial→USB→companion-computer→MAVLink→FC round trip, cutting latency and
removing a failure-prone hop (companion computer crash/reboot/USB
disconnect). The tradeoff is that whatever "logic" is applied now has to live
in flight-controller C++ instead of Python — see §8 for how PX4 already has
infrastructure for exactly this kind of "IR beacon → pixel offset → position
control" problem.

---

## 2. Validated hardware facts

| Fact | Value | Source |
|---|---|---|
| Interface | UART (TX/RX/GND, 3.3V logic) | vendor PDF §1; confirmed working over a USB-serial adapter (any 3.3V UART works identically) |
| Baud rate | **921600** | Confirmed by direct capture — this is the *only* baud rate among {57600, 115200, 230400, 460800, 921600} that produces a clean, CRC-valid `$X` frame stream. All others produce either total silence or garbled bytes. **Do not guess a different rate.** |
| Frame format | 8N1 (standard) | Implied by clean reception at the above baud; no parity/stop-bit issues observed |
| Camera resolution | **1236 x 960** pixels | Confirmed by the user (device spec) |
| Power-on behavior | See §2.1 below — **important, non-obvious** | Confirmed by direct testing |

### 2.1 Power-on / boot behavior — read this before you debug "no data"

On first power-up (or USB re-plug), the camera does **not** immediately
stream telemetry. Its bootloader only listens for an MSP `Ping` (306) within
a short keep-alive window right after boot; if the host isn't already
sending `Ping` requests *during that window*, the bootloader silently exits
it (transitions onward) and the opportunity is gone until the next power
cycle. Symptom: you open the UART, see literally zero bytes for the entire
session, no matter how long you wait or what you send afterward.

**What this means for your PX4 driver:** the FC's UART is presumably powered
continuously alongside the camera (same power rail), so in the field this
should resolve itself naturally once during initial power-on before the
vehicle is armed. But when you are **bench-testing** your driver against the
camera:
1. Have your test/debug loop already running and reading the UART.
2. Power-cycle the camera (unplug/replug its power, not just the UART) while
   your loop is running.
3. Data starts flowing shortly after and then continues indefinitely without
   further intervention — this is **not** a request/reply protocol for
   normal telemetry (see §3.4); once past this boot window, `SpotsReport`
   (298) and the `Logger*` (310-316) messages stream continuously,
   unsolicited, with no host action required.

If your driver only starts running after the camera has been powered for a
while (e.g. it's a PX4 module that starts late in the FC's own boot
sequence, well after the camera powered up), you may never see data because
the camera's window already closed. If you hit this, the practical fix is
either: (a) wire the camera's power through something the FC also controls
so they power up together and the FC module starts fast enough, or (b) send
`Ping` (306) repeatedly during your driver's own init/connect phase — per
the vendor doc, the bootloader's Ping-gate gets confused about the current
timing model, but reading is genuinely necessary here: **actual runtime
telemetry (SpotsReport/Logger) requires no Ping dance in the confirmed
working configuration** — the Ping keep-alive complexity is specifically
about firmware flashing/bootloader recovery workflows (see vendor PDF §6),
not normal target-tracking use. Don't over-engineer a Ping-retry loop into
your main telemetry path; just parse incoming bytes continuously.

---

## 3. Wire protocol — MSP V2 over UART

**Critical caveat inherited from the vendor document itself:** the vendor PDF
explicitly states that it defines *payload* semantics only, and that "The
MSP V2 native wire format, UART electrical settings, synchronization bytes,
frame header fields, direction markers, transport checksum handling,
retries, and timeout policy are outside the scope of this document and must
be taken from the MSP V2 wire-format specification used by the project."
That underlying wire-format spec was never provided. Everything in this §3
is therefore **not** from the vendor document — it is the standard
MSPv2-over-serial framing used by Betaflight/iNav-derived flight stacks,
which was assumed and then **confirmed to work** against the real camera
hardware (clean frames, valid CRCs, sensible decoded values, at 921600
baud). If a firmware update ever changes this camera's wire framing, you'll
know immediately: you'll stop seeing valid `$X` sync bytes or CRCs will
start failing universally.

### 3.1 Frame layout

```
byte:    0    1    2      3      4-5        6-7        8..8+size-1   8+size
field: '$'  'X'  dir  flag  func(u16 LE)  size(u16 LE)   payload       crc8
```

- Bytes 0-1: literal ASCII `$` (0x24) then `X` (0x58) — fixed sync sequence.
- Byte 2 — direction marker (single ASCII char, one of):
  - `<` (0x3C) — request (host → device)
  - `>` (0x3E) — reply / async notification (device → host)
  - `!` (0x21) — error
- Byte 3 — flag (uint8). Always observed as `0x00` in practice; treat as
  reserved, don't validate its value.
- Bytes 4-5 — function/command ID, **uint16, little-endian**. This is the
  MSP command ID (see §4 table — e.g. 298 = SpotsReport).
- Bytes 6-7 — payload size, **uint16, little-endian**, in bytes. Can be 0.
- Bytes 8..(8+size-1) — payload, exactly `size` bytes, structure depends on
  the function ID (see §4/§5).
- Final byte — CRC-8, computed as described in §3.2.

Total frame length = 8 + size + 1 bytes.

### 3.2 CRC-8 (DVB-S2 variant)

The CRC is computed over **flag + func_lo + func_hi + size_lo + size_hi +
payload bytes** (i.e. everything between the direction byte and the CRC
byte itself — NOT including `$`, `X`, or the direction byte). Polynomial
`0xD5`, initial value `0x00`, MSB-first, no final XOR.

Reference C implementation (this is a direct transliteration of the
already-validated Python implementation in
`pipeline/ir_camera_protocol.py::crc8_dvb_s2`):

```c
uint8_t crc8_dvb_s2(const uint8_t *data, size_t len, uint8_t crc /* = 0 */)
{
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ 0xD5);
            } else {
                crc = (uint8_t)(crc << 1);
            }
        }
    }
    return crc;
}
```

To validate a received frame: accumulate `flag, func_lo, func_hi, size_lo,
size_hi` and every payload byte into a buffer as they arrive, then compare
`crc8_dvb_s2(buffer, buffer_len, 0)` against the received CRC byte.

**Known-good test vector** (used in this repo's own unit tests, standard
CRC-8/DVB-S2 check value): `crc8_dvb_s2("123456789", 9, 0) == 0xBC`. Verify
your C implementation against this before trusting it against real camera
bytes.

### 3.3 Byte-at-a-time parsing state machine

The camera streams bytes continuously and asynchronously — you cannot
assume frame-aligned reads from the UART (a single `read()` call may return
a partial frame, multiple frames, or split a frame across two reads). Parse
incrementally, one byte at a time, with an explicit state machine. This is
the exact state machine already validated in
`pipeline/ir_camera_protocol.py::MSPv2Reader` — implement the same states in
C:

```
enum msp_state {
    MSP_STATE_IDLE,
    MSP_STATE_GOT_DOLLAR,
    MSP_STATE_GOT_X,
    MSP_STATE_GOT_DIR,      // (direction stored, not a distinct wait state)
    MSP_STATE_FLAG,
    MSP_STATE_FUNC_LO,
    MSP_STATE_FUNC_HI,
    MSP_STATE_SIZE_LO,
    MSP_STATE_SIZE_HI,
    MSP_STATE_PAYLOAD,
    MSP_STATE_CRC,
};
```

Transition table (feed one byte, update state, and on reaching `MSP_STATE_CRC`
with the final CRC byte, either accept or reject the frame and reset to
`MSP_STATE_IDLE`):

| Current state | Byte received | Action | Next state |
|---|---|---|---|
| IDLE | `'$'` (0x24) | — | GOT_DOLLAR |
| IDLE | anything else | — | IDLE |
| GOT_DOLLAR | `'X'` (0x58) | — | GOT_X |
| GOT_DOLLAR | anything else | — | IDLE (resync) |
| GOT_X | `'<'`/`'>'`/`'!'` | store direction | FLAG |
| GOT_X | anything else | — | IDLE (resync) |
| FLAG | any | store flag; accumulate into CRC buffer | FUNC_LO |
| FUNC_LO | any | `function = byte`; accumulate into CRC buffer | FUNC_HI |
| FUNC_HI | any | `function |= byte << 8`; accumulate into CRC buffer | SIZE_LO |
| SIZE_LO | any | `size = byte`; accumulate into CRC buffer | SIZE_HI |
| SIZE_HI | any | `size |= byte << 8`; accumulate into CRC buffer | PAYLOAD if size>0 else CRC |
| PAYLOAD | any | append to payload buffer; accumulate into CRC buffer; if payload buffer length == size, advance | CRC (once full) else stay in PAYLOAD |
| CRC | any | this byte IS the CRC — compute expected CRC over the accumulated buffer, compare; emit frame result (function, direction, payload, crc_ok); reset all accumulators | IDLE |

Notes:
- Resetting to IDLE on any unexpected byte at GOT_DOLLAR/GOT_X is what makes
  this self-resynchronizing after noise/garbage/link glitches — no explicit
  "resync" logic needed beyond this.
- Do **not** discard a frame just because `crc_ok` is false at the state
  machine level — surface `crc_ok` to the caller and let the caller decide
  (the reference implementation always returns the decoded frame plus a
  `crc_ok` boolean; the production consumer then discards on `crc_ok ==
  false`). This makes debugging much easier (you can log "got frame X with
  bad CRC" instead of silently losing bytes).
- Payload buffer must be sized to the largest possible payload you intend to
  handle. For a driver that only cares about `SpotsReport` (298), the max
  payload is 229 bytes (`1 + 38*6`, per §4) — but other message types
  (`VersionInfo` reply, 208 bytes; `BinaryConfiguration`, up to whatever
  chunk-size budget the link uses) are used for firmware flashing, not
  runtime telemetry. If you truly only need `SpotsReport`, you can bound
  your payload buffer at ~256 bytes and simply drop/resync on any frame
  claiming a larger size (treat it as corruption, since that shouldn't occur
  for the message types you care about).

### 3.4 Request/reply vs. asynchronous notification

Two different traffic patterns exist on this link, and you should treat
them differently in firmware:

- **Asynchronous, unsolicited (device → host, direction `>`):** `SpotsReport`
  (298) and the `Logger*` messages (310-316) stream continuously and
  automatically once the camera's application firmware is running — **no
  request is needed to trigger them.** This is the traffic your driver
  actually cares about for target tracking.
- **Request/reply (host → device `<`, device → host `>`):** `RegisterAccess`
  (299), `BinaryConfiguration` (300), `VersionInfo` (301), `Ping` (306),
  `CRCVerify` (307) are host-initiated. `Reset` (305) is host-only, no
  payload, no reply expected (it reboots the device). These exist for
  configuration/diagnostics/firmware-flashing use cases — **a driver whose
  only job is reading target position does not need to send anything on
  this link at all.** You can write a receive-only driver.

---

## 4. Command reference — all message types

Reproduced from the vendor PDF (`docs/LSDT_Camera_MSP_V2_UART_Interface_Protocol.pdf`,
§3-5 and Appendix A) — payload semantics are the vendor's; the wire framing
around them (§3 above) is this project's independently-confirmed addition.

| ID | Command | Direction / role | Payload length |
|---|---|---|---|
| 298 | Spots Report | Device → host, unsolicited | `1 + 38*spots_count` (max 6 spots, max 229 bytes) |
| 299 | Register Access | Host request / device reply | `1 + 9*registers_count` |
| 300 | BinaryConfiguration | Host request / device reply | `7 + chunkSize` |
| 301 | VersionInfo | Host request (0 bytes) / device reply (208 bytes) | see above |
| 305 | Reset | Host request only | 0 bytes |
| 306 | Ping | Host request / device reply | reply 4 bytes |
| 307 | CRC Verify | Host request / device status reply | request 9 bytes |
| 310 | Logger C0 Config | Device → host, unsolicited | 33 bytes |
| 311 | Logger C1 Config | Device → host, unsolicited | 17 bytes |
| 312 | Logger F0 Config | Device → host, unsolicited | 26 bytes |
| 313 | Logger Detections | Device → host, unsolicited | 51 bytes |
| 314 | Logger Invalidation | Device → host, unsolicited | 2 bytes |
| 315 | Logger Search Done | Device → host, unsolicited | 2 bytes |
| 316 | Logger Debug | Device → host, unsolicited | 6 bytes |

All multi-byte integer fields are **little-endian on the wire**. All structs
below are packed (no implicit padding) — do **not** cast a raw byte buffer
directly onto these structs unless you are certain your compiler's struct
packing/alignment/endianness matches exactly (on ARM Cortex-M / NuttX this
is usually fine for little-endian targets with `__attribute__((packed))`,
but field-by-field decoding via `memcpy`/manual byte assembly is safer and
is what the vendor doc itself recommends: "robust host parsers should still
decode byte buffers field-by-field after validating lengths").

```c
#define MSP_SPOTS_REPORT_MAX_SPOTS   6u
#define MSP_LOGGER_NSMAX             4u
#define MSP_LOGGER_MAX_C0_FRAMES     8u
#define MSP_LOGGER_CLUTTER_TYPES     4u

#define MSP_VERSION_INFO_GIT_SHA_LEN     48u
#define MSP_VERSION_INFO_GIT_BRANCH_LEN  64u
#define MSP_VERSION_INFO_PL_STR_LEN      16u
#define MSP_VERSION_INFO_PS_STR_LEN      16u
#define MSP_VERSION_INFO_BUILD_DATE_LEN  64u

// ── 298: Spots Report ──────────────────────────────────────────────────────
typedef struct __attribute__((packed)) {
    uint8_t  spot_id;          // 0..255; persistent-vs-frame-local semantics unconfirmed by vendor
    uint8_t  is_valid;         // 0 = invalid, 1 = valid; other values reserved
    uint16_t x;                // pixels, image coordinate system (little-endian)
    uint16_t y;                // pixels, image coordinate system (little-endian)
    uint64_t detection_period; // microseconds (little-endian)
    uint64_t timestamp;        // milliseconds; timebase recommended = monotonic device time since boot
    uint64_t spot_score;       // quality/confidence; range/meaning unconfirmed by vendor
    uint64_t spot_width;       // pixels, detected spot width (little-endian)
} spot_t;

typedef struct __attribute__((packed)) {
    uint8_t spots_count;       // 0..6
    spot_t  spots[];           // flexible array, length = spots_count, NO padding between entries
} spots_report_payload_t;

// ── 299: Register Access (multi-register) ─────────────────────────────────
typedef struct __attribute__((packed)) {
    uint8_t  status;           // request: 0 (ignored); reply: 0 = OK, non-zero = error (exact codes unconfirmed)
    uint8_t  access_type;      // 0 = read, 1 = write
    uint8_t  interface_type;   // 0 = I2C, 1 = PL, 2 = PS
    uint16_t slave_address;    // little-endian; relevant when interface_type == I2C
    uint16_t register_address; // little-endian
    uint16_t value;            // little-endian; write: value to write; read: 0 in request, result in reply
} register_entry_t;

typedef struct __attribute__((packed)) {
    uint8_t          registers_count;
    register_entry_t registers[];   // flexible array, length = registers_count
} register_access_payload_t;

// ── 300: BinaryConfiguration (chunked read/write) ──────────────────────────
typedef struct __attribute__((packed)) {
    uint8_t  mode_or_status;   // request: file type (0 = param config, 2 = binary image); reply: 0 = OK, non-zero = error
    uint16_t total_chunks;     // little-endian, 1..65535, constant across one transfer
    uint16_t chunk_index;      // little-endian, zero-based, 0..total_chunks-1
    uint16_t chunk_size;       // little-endian; if >0, data[] holds exactly chunk_size bytes; if 0 this is a read request
    uint8_t  data[];           // flexible array, length = chunk_size
} binary_configuration_payload_t;

// ── 301: VersionInfo ────────────────────────────────────────────────────────
typedef struct __attribute__((packed)) {
    char git_sha[MSP_VERSION_INFO_GIT_SHA_LEN];         // NUL-terminated, zero-padded
    char git_branch[MSP_VERSION_INFO_GIT_BRANCH_LEN];   // NUL-terminated, zero-padded
    char pl_version[MSP_VERSION_INFO_PL_STR_LEN];       // NUL-terminated, zero-padded
    char ps_version[MSP_VERSION_INFO_PS_STR_LEN];       // NUL-terminated, zero-padded
    char build_date[MSP_VERSION_INFO_BUILD_DATE_LEN];   // NUL-terminated, zero-padded, ISO 8601 recommended
} version_info_payload_t;   // request is 0 bytes; reply is exactly 208 bytes

// ── 305: Reset — 0-byte payload, no struct needed ──────────────────────────

// ── 306: Ping ───────────────────────────────────────────────────────────────
typedef struct __attribute__((packed)) {
    uint32_t app_type;   // little-endian. 0 = A53 bootloader, 1 = R5 bootloader, 2 = A53 firmware, 3 = R5 firmware
} ping_payload_t;   // request payload is implementation-specific (normally 0 bytes); reply is 4 bytes

// ── 307: CRC Verify ─────────────────────────────────────────────────────────
typedef struct __attribute__((packed)) {
    uint8_t  mode;             // 0 = parameter configuration, 2 = binary image
    uint32_t expected_crc_le;  // CRC-32, little-endian, computed by host over the source file
    uint32_t byte_length_le;   // little-endian byte count to verify
} binary_crc_verify_payload_t;   // request 9 bytes; reply is a transport-level OK/ERROR status

// ── 310: Logger C0 Config ───────────────────────────────────────────────────
typedef struct __attribute__((packed)) {
    uint8_t  frames_num;                                  // 0..8, number of valid entries below
    uint16_t region_start[MSP_LOGGER_MAX_C0_FRAMES];      // little-endian entries
    uint16_t region_end[MSP_LOGGER_MAX_C0_FRAMES];        // little-endian entries
} logger_c0_cfg_payload_t;

// ── 311: Logger C1 Config ───────────────────────────────────────────────────
typedef struct __attribute__((packed)) {
    uint8_t  sections_num;                    // 0..4, number of valid entries below
    uint16_t section_start[MSP_LOGGER_NSMAX]; // little-endian entries
    uint16_t section_end[MSP_LOGGER_NSMAX];   // little-endian entries
} logger_c1_cfg_payload_t;

// ── 312: Logger F0 Config ───────────────────────────────────────────────────
typedef struct __attribute__((packed)) {
    uint8_t  selected_spot_id;
    uint8_t  selected_pim_id;
    uint8_t  mt_seq_type;
    uint8_t  selected_gain;
    uint16_t tap_width;
    uint16_t predicted_row;
    uint16_t predicted_col;
    uint16_t start_row;
    uint16_t end_row;
    uint16_t start_col;
    uint16_t end_col;
    uint32_t min_period_us;
    uint32_t max_period_us;
} logger_f0_cfg_payload_t;

// ── 313: Logger Detections ───────────────────────────────────────────────────
typedef struct __attribute__((packed)) {
    uint8_t  frame_type;                                    // implementation-defined
    uint8_t  spot_status;                                    // bit field, semantics unconfirmed by vendor
    uint8_t  frame_num;
    uint16_t detected_row[MSP_LOGGER_NSMAX];
    uint16_t detected_pixel[MSP_LOGGER_NSMAX];
    uint16_t spot_iteration[MSP_LOGGER_NSMAX];
    uint16_t spot_segment[MSP_LOGGER_NSMAX];
    uint16_t spot_score[MSP_LOGGER_NSMAX];
    uint16_t declutter_event_counter[MSP_LOGGER_CLUTTER_TYPES];
} logger_detections_payload_t;

// ── 314: Logger Invalidation ─────────────────────────────────────────────────
typedef struct __attribute__((packed)) {
    uint8_t invalidated_spots;   // bitmask/count semantics unconfirmed by vendor
    uint8_t invalidation_reason; // reason code map unconfirmed by vendor
} logger_invalidation_payload_t;

// ── 315: Logger Search Done ──────────────────────────────────────────────────
typedef struct __attribute__((packed)) {
    uint16_t time_ms;   // little-endian, search duration
} logger_search_done_payload_t;

// ── 316: Logger Debug ─────────────────────────────────────────────────────────
typedef struct __attribute__((packed)) {
    uint16_t code;    // little-endian, debug ID
    uint32_t value;   // little-endian, debug value
} logger_debug_payload_t;
```

Several fields above are annotated "unconfirmed by vendor" — the vendor
document itself lists these under an "Integration items to confirm" section
(spot_id persistence semantics, spot timestamp timebase, spot_score
range/meaning, register/BinaryConfiguration non-zero status codes, Ping
request payload shape, and Logger bitfield/enum meanings). Don't block your
implementation on these — they don't affect frame parsing correctness, only
the *interpretation* of a few fields you likely don't need (e.g.
`spot_score`, `spot_width` are not required to compute `dx`/`dy`).

---

## 5. The message that matters: SpotsReport (298), worked example

This is the only message a target-tracking-only driver strictly needs to
parse (all `Logger*` messages are optional diagnostic telemetry; do not
build your core control path around them).

### 5.1 Confirmed real-world shape

In live testing, `spots_count` was consistently `1` (a single tracked
target) — the camera does not appear to report multiple simultaneous spots
in normal operation, even though the protocol allows up to 6. **This
project's own companion-computer implementation deliberately only reads
`spots[0]` and ignores any additional entries** — do the same in the PX4
driver unless you have a specific reason to handle multi-spot logic.

### 5.2 Annotated byte-level example

Below is a complete, real `SpotsReport` frame for one valid spot at pixel
`(x=437, y=261)`, `spot_id=2`, `detection_period_us=20043`, `timestamp_ms=0`,
`spot_score=32767`, `spot_width=0` (score/width/timestamp are illustrative —
not needed for `dx`/`dy`). This exact frame was constructed and round-tripped
through the real, already-validated Python implementation
(`pipeline/ir_camera_protocol.py`) to confirm every byte — including the
CRC — is correct; treat it as a canonical test vector for your own C parser,
not just an illustration:

```
24 58 3e 00 2a 01 27 00 01 02 01 b5 01 05 01 4b 4e 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 ff 7f 00 00 00 00 00 00 00 00 00 00 00 00 00 00 16
```

Header (bytes 0-7):
| Bytes | Value | Meaning |
|---|---|---|
| `24 58` | `$X` | sync |
| `3e` | `>` | direction: device → host |
| `00` | 0 | flag (reserved) |
| `2a 01` | `0x012A` = **298** | function = SpotsReport (little-endian: low byte first) |
| `27 00` | `0x0027` = **39** | payload size = `1 + 38*1` (matches `spots_count=1`) |

Payload (bytes 8-46, 39 bytes total — offsets below are *within the
payload*, i.e. relative to byte 8 of the full frame):
| Payload offset | Bytes | Field | Value |
|---|---|---|---|
| 0 | `01` | `spots_count` | 1 |
| 1 | `02` | `spots[0].spot_id` | 2 |
| 2 | `01` | `spots[0].is_valid` | 1 |
| 3-4 | `b5 01` | `spots[0].x` (LE u16) | `0x01B5` = **437** |
| 5-6 | `05 01` | `spots[0].y` (LE u16) | `0x0105` = **261** |
| 7-14 | `4b 4e 00 00 00 00 00 00` | `spots[0].detection_period` (LE u64) | 20043 |
| 15-22 | `00 00 00 00 00 00 00 00` | `spots[0].timestamp` (LE u64) | 0 |
| 23-30 | `ff 7f 00 00 00 00 00 00` | `spots[0].spot_score` (LE u64) | 32767 |
| 31-38 | `00 00 00 00 00 00 00 00` | `spots[0].spot_width` (LE u64) | 0 |

Final byte (byte 47): `16` — the CRC-8 (§3.2), computed over bytes 3-46
(flag through the last payload byte — everything after the direction byte,
up to but not including the CRC byte itself). If your CRC implementation
produces anything other than `0x16` for this exact byte sequence, your CRC
implementation has a bug — fix it before touching real hardware.

**The easiest offset bug to make:** payload offset 0 is `spots_count`
(how many spots follow), and payload offset 1 is the *first spot's*
`spot_id` — two different, easily-confused fields sitting right next to
each other. Get this wrong and every field after it shifts by one byte
while still looking superficially plausible.

### 5.3 Parsing algorithm for SpotsReport payload

Given a validated (`crc_ok == true`) frame with `function == 298`:

```c
bool parse_spots_report(const uint8_t *payload, uint16_t payload_len,
                         spot_t *out_spots, uint8_t *out_count)
{
    if (payload_len < 1) {
        *out_count = 0;
        return false;
    }
    uint8_t count = payload[0];
    if (count > MSP_SPOTS_REPORT_MAX_SPOTS) {
        // Per vendor doc: "Reject spots_count > 6." Treat as a malformed frame.
        *out_count = 0;
        return false;
    }
    size_t needed = 1 + (size_t)count * 38;
    if (payload_len < needed) {
        // Truncated — decode whichever complete entries fit, per the
        // reference companion-computer implementation's own tolerant
        // behavior (drop only the incomplete trailing entry).
        count = (uint8_t)((payload_len - 1) / 38);
    }

    for (uint8_t i = 0; i < count; i++) {
        const uint8_t *p = payload + 1 + i * 38;
        out_spots[i].spot_id  = p[0];
        out_spots[i].is_valid = p[1];
        out_spots[i].x = (uint16_t)p[2] | ((uint16_t)p[3] << 8);
        out_spots[i].y = (uint16_t)p[4] | ((uint16_t)p[5] << 8);
        // detection_period, timestamp, spot_score, spot_width: decode the
        // remaining 8-byte little-endian fields the same way (byte-by-byte
        // OR-shift) if you need them — not required for dx/dy.
    }
    *out_count = count;
    return true;
}
```

Field-by-field manual decoding (as above) rather than a direct struct cast
is deliberate — it sidesteps any struct-packing/alignment surprises on your
specific toolchain, and matches the vendor doc's own explicit
recommendation (§2.3, §7): "Do not cast untrusted UART bytes directly to C
structs... A field-by-field parser is safer and more portable."

---

## 6. Deriving dx/dy

The camera reports **absolute pixel coordinates**, origin at top-left,
`x` increasing rightward, `y` increasing downward — standard image
coordinate convention. It does **not** report a center-relative offset; you
must compute that yourself using the known frame resolution.

**Confirmed resolution: 1236 x 960 pixels.**

```
frame_center_x = 1236 / 2 = 618
frame_center_y = 960  / 2 = 480

dx_px = spot.x - 618      // positive = target is RIGHT of frame center
dy_px = spot.y - 480      // positive = target is BELOW frame center (image-y-down)
```

This sign convention (`+dx` = right of center, `+dy` = below center,
image-y-down) is the same convention already used consistently across this
project's companion-computer tracking modules
(`target_tracking_module.py`, `marker_detection_module.py`,
`ground_point_tracker_module.py`, `ir_camera_module.py` — all publish this
exact convention). If your PX4-side "logic" is meant to be consistent with
work already validated on the companion-computer side, preserve this sign
convention rather than inventing a different one (e.g. NED-style
y-increases-upward) — that decision should be made deliberately and
documented if you diverge from it, not accidentally inverted by getting the
subtraction backwards.

**Validity/staleness:** only trust `dx`/`dy` when `spots[0].is_valid == 1`
**and** the frame has been CRC-validated **and** you've received a fresh
`SpotsReport` recently (recommend: define a staleness timeout, e.g. 1
second — matching the companion-computer implementation's
`stale_timeout_s` default — and treat "no valid SpotsReport within that
window" the same as "no target," i.e. zero the offset and mark not-tracking,
rather than holding a stale position indefinitely). This exact "always
report *something*, zero it out when invalid rather than omitting the
report" pattern is deliberate prior art from
`ir_camera_module.py::compute_offset()` — a consumer that expects a periodic
heartbeat (even a "no target" heartbeat) is easier to build correctly than
one that has to separately detect "we stopped receiving anything at all."

---

## 7. Testing/validation strategy for your parser

Before wiring this into any control loop, validate the parser in isolation:

1. **CRC-8 unit test.** Verify `crc8_dvb_s2("123456789", 9, 0) == 0xBC`
   (standard check vector, §3.2).
2. **Synthetic frame round-trip.** Build a known `SpotsReport` byte buffer by
   hand (like the annotated example in §5.2), feed it byte-by-byte through
   your state machine, and confirm you get back the exact `spot_id`/`x`/`y`
   you put in, with `crc_ok == true`.
3. **Corruption handling.** Flip one payload byte in a known-good frame,
   confirm `crc_ok` comes back `false` and your consumer discards it (does
   not update `dx`/`dy` from a bad frame).
4. **Resync after garbage.** Prepend random noise bytes (that don't happen
   to contain a real `$X` sequence) before a valid frame; confirm the state
   machine recovers and still decodes the valid frame correctly. This is
   important for real UART links where boot-time noise or a mid-stream glitch
   is expected.
5. **Split-read handling.** Simulate the frame arriving across multiple
   `read()` calls at arbitrary byte boundaries (not just once per frame) —
   confirm the state machine still assembles it correctly regardless of
   where the reads happen to split.
6. **Real hardware.** Only after 1-5 pass: wire to the actual camera at
   921600 baud, power-cycle it while your code is already reading (§2.1),
   and confirm you see a continuous stream of valid `SpotsReport` frames
   with sane, moving `x`/`y` values when a target is presented to the
   camera.

If you want a reference for what "correct" looks like end-to-end, the
Python implementation in this repo (`pipeline/ir_camera_protocol.py` +
`tests/test_ir_camera_protocol.py`) already passes all of the equivalent
tests against the same protocol — port the test *cases*, not the Python
code, to validate your C implementation.

---

## 8. PX4 module implementation guidance

This section is architectural guidance based on general PX4 module/driver
conventions, **not** verified against a specific checked-out PX4-Autopilot
source tree (this repo does not contain one). Before writing code, actually
open the PX4-Autopilot source you're targeting and read the real files named
below — treat this section as "where to look and what pattern to follow,"
not as verified API signatures. PX4's module APIs do shift across versions.

### 8.1 The key architectural decision: custom uORB topic, or PX4's existing IR-beacon pipeline?

PX4 already has a subsystem built for almost exactly this problem — an IR
beacon reporting a target's angular/pixel offset, consumed for
position/landing control:

- **`irlock_report` uORB message** — designed for the IRLock sensor (a
  similar IR-beacon precision-landing camera). Fields (verify exact current
  fields against `msg/IrlockReport.msg` in your PX4 source): timestamp,
  signature/target ID, `pos_x`/`pos_y` (target angular offset — typically
  **normalized/angular** values like `tan(angle_x)`, `tan(angle_y)`, *not*
  raw pixels), `size_x`/`size_y`.
- **`landing_target_estimator` module** (`src/modules/landing_target_estimator/`
  in PX4-Autopilot) — already consumes `irlock_report`, runs a Kalman filter
  to estimate target position/velocity, and publishes `landing_target_pose`,
  which PX4's position controller can use directly for precision landing.
- **`src/drivers/irlock/`** — the existing driver that talks to the actual
  IRLock sensor hardware and publishes `irlock_report`. This is your closest
  structural template: a serial/I2C sensor driver that publishes a
  pixel/angle-offset uORB message.

**Recommended path:** write your camera driver to publish `irlock_report`
(or a close analog) instead of inventing a brand-new uORB message + a
brand-new consumer "logic" module from scratch. This gets you PX4's existing,
flight-tested target-position Kalman filter and its existing integration
into position control, for free. The catch: `irlock_report.pos_x/pos_y` are
conventionally **angular** offsets (radians, or `tan(angle)`), not raw
pixels — converting from pixel offset requires the camera's angular
field-of-view, which **this project has not yet confirmed for this specific
camera** (a related companion-computer PID module,
`target_position_module.py`, currently uses an assumed/uncalibrated
`camera_fov_deg = 78°` default that was flagged during development as
likely wrong for this camera's actual optics — do not reuse that number
without independent confirmation). You will need the camera's real FOV
(from its datasheet, or measured empirically: present a target at a known
lateral distance and known range, measure the resulting pixel shift,
back-solve for FOV) before `irlock_report` angular conversion will be
correct.

**Fallback path:** if you don't want to depend on `irlock_report`'s angular
convention (e.g. FOV genuinely isn't known yet, or you want full custom
control logic rather than PX4's existing landing-target Kalman filter),
define a new custom uORB message instead, e.g.:

```
# msg/IrCameraReport.msg
uint64 timestamp        # hrt_absolute_time(), microseconds
float32 dx_px           # pixel offset from frame center, +right (see §6)
float32 dy_px           # pixel offset from frame center, +down (see §6)
uint16 frame_width      # 1236 (confirmed, §2)
uint16 frame_height     # 960 (confirmed, §2)
bool valid              # true only when a fresh, CRC-valid, is_valid==1 spot exists
```
Publish this unconditionally every read cycle (per the "always publish,
zero when invalid" pattern from §6), and write your own consumer logic
against it. This is more work (no existing Kalman filter, no existing
position-controller integration) but avoids depending on an unconfirmed FOV
number and gives you full control over the "logic" the user wants to apply.

**This is a decision to make with the project owner before committing to an
implementation** — the two paths lead to meaningfully different amounts of
new code and different places the resulting position estimate plugs into
PX4's control stack. This document deliberately does not pick for you.

### 8.2 Module structure pattern

Regardless of which path in §8.1 you choose, the driver itself (the part
that owns the UART, runs the parser from §3, and publishes a uORB message)
should follow standard PX4 module conventions:
- A `ModuleBase`-derived class scheduled on PX4's work queue (`ScheduledWorkItem`),
  polling the UART fd, or driven by UART RX interrupt/DMA if your board
  support package exposes that — follow whatever pattern `src/drivers/irlock/`
  (or another comparable serial-sensor driver in your PX4 source tree) uses;
  do not invent a new threading model.
- Open the UART device using PX4's serial port abstraction for your target
  board (device path like `/dev/ttyS*` mapped via the board's UART config —
  which physical port this ends up on, e.g. `TELEM2`/`GPS2`/an unused UART,
  is a hardware/board-config decision this document does not make; confirm
  with the project owner which physical UART the camera will be wired to
  before hardcoding a device path).
- Configure the port for 921600 baud, 8N1 (§2).
- Feed every received byte through the state machine in §3.3.
- On a `crc_ok == true`, `function == 298` frame: parse per §5.3, compute
  `dx`/`dy` per §6, publish per whichever message you chose in §8.1.
- On a stale/no-data condition (§6): still publish, with the offset zeroed
  and `valid`/tracking-state indicating "no target" — don't just stop
  publishing.

### 8.3 Open items to resolve before/while implementing

- **Camera angular FOV** — required if using `irlock_report`; not yet
  measured/confirmed for this camera (§8.1).
- **Physical UART assignment** — which FC UART the camera is wired to, and
  whether that UART is free/configurable on your specific board target.
- **PX4 version/branch** — confirm exact `irlock_report`/
  `landing_target_estimator` field names and module APIs against the actual
  PX4-Autopilot source you're building against; this document's PX4-specific
  references are pattern-level, not version-pinned.
- **spot_score / spot_width semantics** — vendor-unconfirmed (§4); not
  required for `dx`/`dy`, skip unless a future need arises.
