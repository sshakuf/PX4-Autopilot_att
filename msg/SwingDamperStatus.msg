# Status of the pendulum swing damper (DF_SWAY_*)

uint64 timestamp        # time since system start (microseconds)

bool active             # DF_SWAY_EN set and a finite acceleration is arriving

float32 accel_n         # low-passed horizontal acceleration used, NED north [m/s^2]
float32 accel_e         # low-passed horizontal acceleration used, NED east [m/s^2]

float32 vel_n           # band-passed swing velocity estimate, NED north [m/s]
float32 vel_e           # band-passed swing velocity estimate, NED east [m/s]

float32 thrust_n        # damping thrust added to the setpoint, NED north [normalised]
float32 thrust_e        # damping thrust added to the setpoint, NED east [normalised]
