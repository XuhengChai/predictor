# 一帧预测输入数据

std_msgs/Header header

# 场景及帧信息
string scenario_id
string save_path
uint64 frame_id
float64 time_stamp

# Ego 公共信息
float64 ego_v
float64 ego_yawrate
float64 steer_wheel_angle
# Ego 地图坐标信息
# float64 ego_easting
# float64 ego_northing
# float64 ego_orientation

# 当前帧所有目标
PredictAgent[] agents