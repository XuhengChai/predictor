std_msgs/Header header

# at present, think the hmi is only need ego and agent to plot bev,
# without line and group information.

# If caution/pay_attention/critical
string time_stamp
float64 articulation_angle
float64 yaw_rate
float64 speed
# TURN_LEFT = 0, TURN_RIGHT = 1,  STRAIGHT = 2
uint8 turn_light_lr
string ica_action_guide
string ica_trigger_ag_id
float64 ica_target_time
float64 ica_target_speed
HmiAgent[] agents
string[] time_str
string bsd_downgrade_reason
string ica_downgrade_reason
int32 driver_triggers_ica
int32 ica_control_flg

# # ------------ego data used in plot bev------------
# string[] ego_cols_float
# int8[] ego_shape_float
# string[] ego_data_float

# # ------------agent data used in plot bev, including BSD warn result-------
# string[] ag_cols_str
# string[] ag_cols_float
# string[] ag_cols_int
# string[] ag_cols_lst_tup

# int8[] ag_shape_str
# int8[] ag_shape_float
# int8[] ag_shape_int
# int8[] ag_shape_lst_tup

# string[] ag_data_str
# string[] ag_data_float
# string[] ag_data_int
# string[] ag_data_lst_tup

# # --------------ica hmi information--------------------
# # If downgrade
# string[] ica_downgrade_reason

# # If caution/pay_attention/critical
# string ica_action_guide
# string ica_trigger_ag_id
# float64 ica_target_time
# float64 ica_target_speed


# #----------------bsd hmi information, mainly for downgrade--------
# # If downgrade
# string[] bsd_downgrade_reason





