std_msgs/Header header

float64 v_trailer
float64 wheel_spd
int8 turning_light_switch
float64 trailer_wheelbase

float64 artic_angle # rad
float64 artic_angle_confid
float64 artic_angle_vd
float64 artic_angle_confid_vd
float64 artic_angle_cv
float64 artic_angle_confid_cv

float64 v_ego # unit: kph
float64 hrw_speed_fl
float64 tsmo_rpm
float64 tsmi_rpm
float64 tach_speed
float64 tsmo_speed
float64 tsmi_speed
float64 gear_ratio

float64 a_ecoroll # unit: m/s2
float64 m # unit: kg
float64 weight_amt
float64 weight_ebs
float64 engine2wheel_ratio
float64 max_acc
# The follwing signals are new added
float64 tractor_yawrate # rad/s

float64 forward_drive_distance
float64 trailer_wheelbase_wb_save
float64 trailer_len
float64 trailer_len_confid
float64 acceleration # m/s2
int8 turning_light_right
float64 accel_pedal_position
float64 brake_pedal_position
float64 steering_angle # rad
float64 xbr_deceleration_limit
int32 current_gear
int8 shift_process

# The following signals are directly from CAN message
int8 pri_xbr_aeb
int32 reference_torque
int32 friction_torque

# The following signals are used for enable or disable articulation_cv calculation
bool is_wheel_turn
bool is_forward_drive

# The following signals are used for pnc node state
int8 amber_lamp_eng
int8 amber_lamp_ebs
int8 amber_lamp_tcm
int8 red_lamp_eng
int8 red_lamp_ebs
int8 red_lamp_tcm
