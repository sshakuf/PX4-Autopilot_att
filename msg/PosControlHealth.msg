# Health of the horizontal control stack and which contributions are live.
#
# One place to see how the independently-enabled features are composing:
# position/velocity control, IR beacon hold (DF_TGT_*), swing damping
# (DF_SWAY_*) and heading hold (DF_YAW_*).

uint64 timestamp            # time since system start (microseconds)

bool input_valid            # PositionControl::_inputValid(): horizontal loop is running
bool xy_vel_valid           # estimator velocity usable; the usual reason input_valid drops
bool height_valid           # dist_bottom_valid, required by target hold

float32 decay_scale         # DF_INVALID_DECAY fade applied to the held thrust: 1 = just lost, 0 = faded out
float32 thrust_ctrl_n       # horizontal thrust from position control after fading, NED north [normalised]
float32 thrust_ctrl_e       # horizontal thrust from position control after fading, NED east [normalised]
float32 thrust_sway_n       # swing damper contribution, NED north [normalised]
float32 thrust_sway_e       # swing damper contribution, NED east [normalised]
float32 thrust_total_n      # sum actually commanded after arbitration, NED north [normalised]
float32 thrust_total_e      # sum actually commanded after arbitration, NED east [normalised]

float32 thrust_budget       # MPC_THR_MAX - DF_YAW_RESERVE: horizontal magnitude allowed [normalised]
float32 thrust_demand       # |thrust_ctrl + thrust_sway| before arbitration [normalised]
float32 thrust_scale        # factor applied to fit the budget; 1.0 = untouched, <1 = translation yielded

float32 yaw_rate_sp         # yaw rate setpoint published to the rate loop [rad/s]
bool yaw_rate_sp_valid      # false when a non-finite setpoint had to be replaced with zero

bool target_hold_active     # DF_TGT_HOLD_EN engaged and tracking the beacon
bool sway_active            # DF_SWAY_EN engaged with finite acceleration
bool heading_hold_active    # heading hold producing the yaw rate setpoint
