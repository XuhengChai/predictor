from enum import Enum, IntEnum

class EDatasource(Enum):
    ONLINE = 0 # IPM
    ONLINE_NO_GPS = 0 # online lidar with integrated ego pose
    OFFLINE_GPS = 1 # AV2
    OFFLINE_RELATIVE = 2 # AV2
    ONLINE_GPS = 3 # online lidar with ego gps pose

class EMacro(Enum):
    EGO_DATA = 'ego'
    OBJ_DATA = 'object'
    SCENE_ID = 'scene_id'
    ORIGIN_META = 'origin_meta'

class EMacroProp(Enum):
    SCENE_ID = "scenario_id"
    TRACK_ID = "track_id"
    FRAME_ID = "frame_id"
    REL_X = "rel_x"
    REL_Y = "rel_y"
    REL_VX = "rel_vx"
    REL_VY = "rel_vy"
    AGENT_TYPE = "agent_type"
    EGO_V = "ego_v"
    EGO_YAWRATE = "ego_yawrate"
    TIME_STAMP = "time_stamp"
    STEER_WHEEL_ANGLE = "SteerWheelAngle"
    X = "x"
    Y = "y"
    YAW_RAD = "yaw_rad"
    EGO_X = "ego_x"
    EGO_Y = "ego_y"
    EGO_HEADING_RAD = "ego_heading_rad"


class EPredictorName(Enum):
    BASE = 'base'
    CV = 'cv'
    CA = 'ca'
    IMMMDN = 'imm_mdn'
    TNT = 'tnt'

class EAgentType(IntEnum):
    PEDESTRIAN = 0
    BICYCLE = 1
    CYCLIST = 1
    MOTORCYCLIST = 2
    VEHICLE = 3
    BUS = 4
    TRUCK = 4
    DUMMY_CLS_CA = 10
    DUMMY_CLS_CV = 11
    DUMMY_CLS_TNT = 12
    DUMMY_CLS_IMM_MDN = 13

class EAgentProp(IntEnum):
    X = 0                          # 0
    Y = X + 1                      # 1
    VX = Y + 1                     # 2
    VY = VX + 1                    # 3
    YR = VY + 1                    # 4   yawrate
    YAW = YR + 1
    CLASS = YAW + 1                #    class (0: ped, 1: 2w)
    TRACK_ID = CLASS + 1    #
    FRAME_ID = TRACK_ID + 1        #
    LEVEL = FRAME_ID + 1           #
    TIME_STAMP = LEVEL + 1         #
    V = TIME_STAMP + 1
    TRAFFIC = V + 1       # 0 , 1 , 2 红绿黄灯
    LANE_LEFT = TRAFFIC + 1        # LANE车道是否左转
    LANE_RIGHT = LANE_LEFT + 1     # LANE车道是否右转
    INTERSECTION = LANE_RIGHT + 1  #
    OX = INTERSECTION + 1  #
    OY = OX + 1  #
    OYAW = OY + 1  #
    OBJECT_CATEGORY = OYAW + 1  #  #
    IS_RESET = OBJECT_CATEGORY + 1
    # NUM_AGENT_COLS = LEVEL + 1

    @classmethod
    def names(cls):
        return [e.name for e in cls]

# static params
class CommonParams:
    ego_frequency = 10 # fps 10HZ
    ego_range = 30 # perception range: meter
    obj_frequency = 10 # fps 10HZ
    obj_history = 3 # history trajectory is 3s
    obj_predict = 3 # history trajectory is 3s
    DEBUG = False
    dimensions_set = {
        'pedestrian': (0.8, 0.5, 1.8),  # person
        'bicycle': (1.6, 0.3, 1.865),  # bicycle
        'cyclist': (1.6, 0.3, 1.865),  # bicycle
        'car': (4.075, 1.668, 1.474),  # car
        'vehicle': (4.075, 1.668, 1.474),  # car
        'motorcyclist': (1.75, 0.5, 1.0),  # motorcycle
        'bus': (9.6, 2.5, 3.45),  # bus
        'truck': (7.2, 2.3, 2.7),  # truck
        'unknown': (0.5, 0.5, 0.5),  #
        4: (0.5, 0.5, 0.5),  #
        8: (0.5, 0.5, 0.5),  #
        9: (0.5, 0.5, 0.5),  #
        10: (0.5, 0.5, 0.5)  #
    }