# Status of the IR-beacon stay-above-target controller (DF_TGT_*)

uint64 timestamp        # time since system start (microseconds)

bool active             # controller currently commanding velocity setpoints
bool target_valid       # fresh valid ir_camera_report with finite angles
float32 height          # height above ground used for projection [m]

float32 offset_n        # tilt-compensated beacon offset from vehicle, NED north [m]
float32 offset_e        # tilt-compensated beacon offset from vehicle, NED east [m]

float32 vel_cmd_n       # commanded velocity setpoint, NED north [m/s]
float32 vel_cmd_e       # commanded velocity setpoint, NED east [m/s]

float32 integral_n      # integrator state contribution [m/s]
float32 integral_e      # integrator state contribution [m/s]
