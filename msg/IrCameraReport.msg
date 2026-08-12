# LSDT IR beacon camera target report (downward-facing spot-detection camera)
# Published continuously: on every camera SpotsReport frame and as a low-rate
# heartbeat when no target is tracked (valid = false, offsets zeroed).

uint64 timestamp        # time since system start (microseconds)

uint16 frame_width      # camera frame width [px]
uint16 frame_height     # camera frame height [px]

float32 dx_px           # raw image-frame offset from center, positive = target right of image center [px]
float32 dy_px           # raw image-frame offset from center, positive = target below image center (image y-down) [px]

float32 angle_x         # target bearing in body frame, positive = target toward body +X / forward [rad]; undistorted (DF_IRC_FX/FY/CX/CY/K1); NAN if intrinsics not configured
float32 angle_y         # target bearing in body frame, positive = target toward body +Y / right [rad]; undistorted; NAN if intrinsics not configured

uint8 spot_id           # camera-reported spot id
uint32 spot_score       # camera-reported quality score (vendor semantics unconfirmed, for diagnostics)

bool valid              # true only while fresh (< timeout), CRC-valid reports with is_valid == 1 are arriving
