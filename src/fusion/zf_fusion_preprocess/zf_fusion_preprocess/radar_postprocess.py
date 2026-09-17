import copy
import math

from collections import deque
from enum import Enum, unique, IntEnum
import numpy as np
import cv2
import collections
# from boxmot.utils.association import linear_assignment

# dict_post_status_5G4T = {
#     0: "INIT",
#     1: "COLLISION_TRAILER",
#     2: "ON_TRAILER",
#     4: "REAR_END",
#     8: "DECT_STOP",
#     16: "DECT_OVERLAP",
# }
@unique
class EPostStatus5G4T(Enum):
    INIT = 0
    COLLISION_TRAILER = 1
    ON_TRAILER = 2 # Objects suddenly appear on the trailer
    REAR_END = 4 # Objects with rear end collisions
    DECT_STOP = 8 # Stationary objects suddenly appear
    DECT_OVERLAP = 16 # With overlap area


@unique
class EProp5G4T(IntEnum):
    id = 0
    rel_spd_lat = id + 1
    rel_spd_lon = rel_spd_lat + 1
    pos_y = rel_spd_lon + 1
    pos_x = pos_y + 1
    status = pos_x + 1
    motion = status + 1
    length = motion + 1
    width = length + 1
    lifetime = width + 1
    class_confidence = lifetime + 1
    CLASS = class_confidence + 1
    prob = CLASS + 1
    heading = prob + 1
    rel_spd = heading + 1
    post_status = rel_spd + 1

@unique
class EPropIPM(IntEnum):
    id = 0
    pos_x = id + 1
    abs_spd = pos_x + 1
    heading = abs_spd + 1
    prob = heading + 1
    CLASS = prob + 1
    status = CLASS + 1
    motion = status + 1
    pos_y = motion + 1
    width = pos_y + 1
    Lane = width + 1
    BrakeLight = Lane + 1
    rel_spd = BrakeLight + 1
    CutInCutOut = rel_spd + 1
    length = CutInCutOut + 1
    height = length + 1
    rel_spd_lat = height + 1
    rel_spd_lon = rel_spd_lat + 1

class GlobalDataTrailer:

    @staticmethod
    def trailer_length() -> float:
        return 13.0 # unit:m

    @staticmethod
    def trailer_width() -> float:
        return 2.676 # unit:m

    @staticmethod

    def trailer_corners():
        """
    (p3)|-------------|(p4) left          y
        |             |                 |
        |          o  |                 |------------ x
        |             |
    (p2)|-------------|(p1) right
        """
        return np.array([
            [1.27,   -11.73, -11.73, 1.27],
            [-1.339, -1.339, 1.337,  1.337],
            [1, 1, 1, 1],
        ])

class MathUtil:

    @staticmethod
    def is_round_zero(value):
        return abs(value) < 0.3
        # return abs(value) < 1e-5

    @staticmethod
    def is_near_0(value):
        return abs(value) < 1e-5

    @staticmethod
    def rad2deg(rad):
        return rad * 180 / math.pi

    @staticmethod
    def angle_2v(vector_a, vector_b):
        cross_product = vector_a[0] * vector_b[1] - vector_a[1] * vector_b[0]
        dot_product = vector_a[0] * vector_b[0] + vector_a[1] * vector_b[1]
        angle = math.atan2(cross_product, dot_product)
        # Convert the angle from radians to degrees
        # angle_degrees = math.degrees(angle)
        return angle

    @staticmethod
    def powers_finder(num):
        powers = []
        i = 1
        while i <= num:
            if i & num:
                powers.append(i)
            i <<= 1
        return powers

    @staticmethod
    def rotate_2d(angle, is_degree=True):
        """
        Returns a 3D transformation matrix for a rotation about the z-axis
        by a given angle in either degrees or radians.

        Parameters:
        angle (float): The rotation angle.
        is_degree (bool): True if the angle is in degrees, False if it is in radians.
                          Default is False.

        Returns:
        A 3x3 NumPy array representing the transformation matrix.
        """
        if is_degree:
            angle = np.deg2rad(angle)
        cos = math.cos(angle)
        sin = math.sin(angle)
        return np.array([[cos, -sin, 0],
                         [sin, cos, 0],
                         [0, 0, 1]])

    @staticmethod
    def is_left(p, pA, pB):
        v1 = [p[0] - pA[0], p[1] - pA[1]]
        v2 = [p[0] - pB[0], p[1] - pB[1]]
        cross = v1[0] * v2[1] - v1[1] * v2[0]
        if cross >= 0:
            return True
        else:
            return False

    @staticmethod
    def rbbox_to_corners(rbbox):
        # generate clockwise corners
        # Positive angles are counter-clockwise and negative are clockwise rotations.
        cx, cy, x_d, y_d, angle = rbbox # angle is heading -- rad
        a_cos = math.cos(angle)
        a_sin = math.sin(angle)
        # corners_x = [-x_d / 2, -x_d / 2, x_d / 2, x_d / 2]
        # corners_y = [-y_d / 2, y_d / 2, y_d / 2, -y_d / 2]
        # corners = [0] * 8
        # for i in range(4):
        #     corners[2 * i] = a_cos * corners_x[i] - \
        #                  a_sin * corners_y[i] + cx
        #     corners[2 * i + 1] = a_sin * corners_x[i] + \
        #                  a_cos * corners_y[i] + cy
        # return corners
        """
        (p3)|-------------|(p4) left          y
            |             |                 |
            |      o      |                 |------------ x
            |             |
        (p2)|-------------|(p1) right
        """
        corners = np.array([
            [x_d / 2, -x_d / 2, -x_d / 2, x_d / 2],
            [-y_d / 2, -y_d / 2, y_d / 2, y_d / 2],
            [1, 1, 1, 1],
        ])
        rotation_matrix = np.array([
            [a_cos, -a_sin, cx],
            [a_sin, a_cos, cy],
            [0, 0, 1],
        ])
        rotated_corners = rotation_matrix @ corners
        # print(rotated_corners.shape[1]) # 3, 4
        # print(rotated_corners[0:2, 0])
        return rotated_corners

    @staticmethod
    def iou_batch_rotate(rbboxes1, rbboxes2):
        """
        From SORT: Computes IOU between two bboxes in the form [cx, cy, length, width, angle:deg]
        """
        intersection_matrix = np.zeros((rbboxes1.shape[0], rbboxes2.shape[0]), dtype=int)
        iou_matrix = np.zeros((rbboxes1.shape[0], rbboxes2.shape[0]), dtype=float)
        for i in range(rbboxes1.shape[0]):
            rbbox1 = rbboxes1[i]
            rect1 = ((rbbox1[0], rbbox1[1]),
                     (rbbox1[2], rbbox1[3]),
                     rbbox1[4])
            area1 = rbbox1[2] * rbbox1[3]
            for j in range(rbboxes2.shape[0]):
                rbbox2 = rbboxes2[j]
                rect2 = ((rbbox2[0], rbbox2[1]),
                         (rbbox2[2], rbbox2[3]),
                         rbbox2[4])
                area2 = rbbox2[2] * rbbox2[3]
                intersection_matrix[i][j], int_pts = cv2.rotatedRectangleIntersection(rect1, rect2)
                if int_pts is not None:
                    order_pts = cv2.convexHull(int_pts, returnPoints=True)
                    intersec = cv2.contourArea(order_pts)
                    union = area1 + area2 - intersec
                    iou_matrix[i][j] = intersec * 1.0 / union
                # print(int_pts)
        # print(intersection_matrix)
        # print(iou_matrix)
        return intersection_matrix, iou_matrix

    @staticmethod
    def iou_batch_rotate_self(rbboxes):
        """
        Calculate IOU between rotated bounding boxes in the form [cx, cy, length, width, angle: deg]
        for the same set of rbboxes, resulting in an upper triangular matrix.
        """
        num_rbboxes = len(rbboxes)
        intersection_matrix = np.zeros((num_rbboxes, num_rbboxes), dtype=int)
        # iou_matrix = np.zeros((num_rbboxes, num_rbboxes), dtype=float)
        for i in range(num_rbboxes):
            rbbox1 = rbboxes[i]
            rect1 = ((rbbox1[0], rbbox1[1]), (rbbox1[2], rbbox1[3]), MathUtil.rad2deg(rbbox1[4]))
            area1 = rbbox1[2] * rbbox1[3]
            for j in range(i + 1, num_rbboxes):
                rbbox2 = rbboxes[j]
                rect2 = ((rbbox2[0], rbbox2[1]), (rbbox2[2], rbbox2[3]), MathUtil.rad2deg(rbbox2[4]))
                area2 = rbbox2[2] * rbbox2[3]
                intersection_matrix[i][j], _ = cv2.rotatedRectangleIntersection(rect1, rect2)
        #         intersection_matrix[i][j], int_pts = cv2.rotatedRectangleIntersection(rect1, rect2)
        #         if int_pts is not None:
        #             order_pts = cv2.convexHull(int_pts, returnPoints=True)
        #             intersec = cv2.contourArea(order_pts)
        #             union = area1 + area2 - intersec
        #             iou_matrix[i][j] = intersec * 1.0 / union
        # return intersection_matrix, iou_matrix
        return intersection_matrix

    @staticmethod
    # 中心点坐标、矩形的宽和高、旋转角（单位是角度，不是弧度）
    def iou_rotate(box1, box2):
        area1 = box1[2] * box1[3]
        area2 = box2[2] * box2[3]
        r1 = ((box1[0], box1[1]), (box1[2], box1[3]), box1[4])
        r2 = ((box2[0], box2[1]), (box2[2], box2[3]), box2[4])
        int_pts = cv2.rotatedRectangleIntersection(r1, r2)[1]
        if int_pts is not None:
            order_pts = cv2.convexHull(int_pts, returnPoints=True)
            intersec = cv2.contourArea(order_pts)
            union = area1 + area2 - intersec
            iou = intersec * 1.0 / union
        else:
            iou = 0.0
        return iou

    # @staticmethod
    # def min_cost_matching_indices(iou_matrix: np.array([]), max_distance: float=0.01):
    #     """Solve linear assignment problem.
    #         Parameters
    #         ----------
    #         distance_metric : Callable[List[Track], List[Detection], List[int], List[int]) -> ndarray
    #             The distance metric is given a list of detections and trackers as well as
    #             a list of N track indices and M detection indices. The metric should
    #             return the NxM dimensional cost matrix, where element (i, j) is the
    #             association cost between the i-th track in the given track indices and
    #             the j-th detection in the given detection_indices.

    #         Returns
    #         List[(int, int)]
    #         """
    #     if min(iou_matrix.shape) > 0:
    #         a = (iou_matrix > max_distance).astype(np.int32)
    #         # print(a)
    #         if a.sum(1).max() == 1 and a.sum(0).max() == 1:
    #             matched_indices = np.stack(np.where(a), axis=1)
    #         else:
    #             matched_indices = linear_assignment(-iou_matrix)  # 匈牙利匹配计算出检测框与跟踪框能够匹配起来的索引对
    #             # print(matched_indices)
    #     else:
    #         matched_indices = np.empty(shape=(0, 2))
    #     return matched_indices

class ObjectData:
    def __init__(self, data_dic: dict, time_stamp):
        self._time_stamp = time_stamp
        # self._data_dic = {k: v[:] for k, v in data_dic.items()}
        self._data_dic = data_dic
        # self._data_raw = {}
        # for key, value in self._data_dic.items():
        #     key = int(value[EProp5G4T.id])
            # self._data_raw[key] = value[:]
            # self._id_list.append(value[EProp5G4T.id])
        self._id_list = list(self._data_dic.keys())


    def get_id_list(self):
        return self._id_list

    def set_data(self, dat: dict):
        self._data_dic = dat

    # get obj data frame
    def get_data(self):
        return self._data_dic

    # def get_data_raw(self):
    #     return self._data_raw

    def get_time_stamp(self):
        return self._time_stamp

class Singleton(object):
    def __init__(self, cls):
        self._cls = cls
        self._instance = {}

    def __call__(self):
        if self._cls not in self._instance:
            self._instance[self._cls] = self._cls()
        return self._instance[self._cls]

@Singleton
class RadarSigsDescription5G4T:
    def __init__(self):
        self.__motion_st: dict = {}
        self.__motion_st = self.__motion_st.fromkeys(range(8), "not_available")
        self.__motion_st[0] = "not defined"
        self.__motion_st[1] = "stationary"
        self.__motion_st[2] = "moving"
        self.__motion_st[3] = "stopped"

    def get_motion_st(self) -> dict:
        return self.__motion_st


class PostProcess5G4T:

    def __init__(self, his_max_length=5):
        # self.__key_list = ["ID_A",
        #                  "rel_Lat_Speed",
        #                  "rel_Long_Speed",
        #                  "rel_Lat_Pos",
        #                  "Lifetime",
        #                  "ClassConfidence",
        #                  "Class",
        #                  "Quality"]
        self.__key_dic = {
            "ID_A": "id",
            "rel_Lat_Speed": "rel_spd_lat",
            "rel_Long_Speed": "rel_spd_lon",
            "rel_Lat_Pos": "pos_y",
            "rel_Long_Pos": "pos_x",
            "TrackingStatus": "status",
            "Motion_Class": "motion",
            "Length": "length",
            "Width": "width",
            "Lifetime": "lifetime",
            "ClassConfidence": "class_confidence",
            "Class": "class",
            "Quality": "prob",
            "heading": "heading",
            "rel_spd": "rel_spd",
        }
        # self.__key_list = self.__key_dic.keys()
        # self.__prop_len = len(self.__key_list)
        # self.__index_dic = dict(zip(self.__key_list, range(self.__prop_len)))
        # self.__data_dic = {}
        # self.__data_status = {}
        # self.__time_stamp = None
        self.__max_angle_range = [np.deg2rad(15), np.deg2rad(150)]

        sig_decp = RadarSigsDescription5G4T()
        self.__motion_st = sig_decp.get_motion_st()
        self.__his_max_length: int = his_max_length
        self.__data_queue = deque(maxlen=his_max_length)
        self.__id_cnt = None
        self.__empty = True
        self.__heading_queue = deque(maxlen=his_max_length)
        self.__cur_ego_angle = 0.0
        self.__rotate_corners = np.array([])
        self.__trailer_rect: list = [0.0, 0.0, 1.0, 1.0, 0.0]
        self.__cols = ['pos_x', 'pos_y', 'length', 'width', 'heading']

    def set_ego_state_angle(self, angle: float):
        self.__cur_ego_angle = angle

    def add_object_data(self, obj: ObjectData) -> ObjectData:
        revised_obj = self.__update_obj(obj)
        if len(self.__data_queue) == self.__his_max_length:
            t_obj: ObjectData = self.__data_queue[-1]
            if t_obj.get_data() and self.__id_cnt:  # len(t_data)
                self.__id_cnt.subtract(t_obj.get_id_list())
        self.__data_queue.appendleft(revised_obj)
        return self.__data_queue[0]

    def __update_obj(self, obj: ObjectData):
        data_cur = obj.get_data()
        if self.__empty:
            if data_cur:
                self.__id_cnt = collections.Counter(
                    obj.get_id_list())
            self.__empty = not self.__empty
            return obj
        if data_cur:
            if self.__id_cnt is None:
                self.__id_cnt = collections.Counter(
                    obj.get_id_list())
            else:
                self.__id_cnt.update(obj.get_id_list())
        else:
            return obj

        trailer_corners = GlobalDataTrailer.trailer_corners()
        self.__rotate_corners = MathUtil.rotate_2d(self.__cur_ego_angle, False) \
                         @ trailer_corners
        cx = np.mean(self.__rotate_corners[0])
        cy = np.mean(self.__rotate_corners[1])
        cv_width: float = GlobalDataTrailer.trailer_length()  #col 13.0
        cv_height: float = GlobalDataTrailer.trailer_width()  #row 2.676
        self.__trailer_rect = [
            cx, cy, cv_width, cv_height, self.__cur_ego_angle
        ]
        for key, value in data_cur.items():
            self.__update_value(value)
            if value[EProp5G4T.post_status] == EPostStatus5G4T.DECT_STOP:
                value[EProp5G4T.class_confidence] = 0.2
        # data_cur = self.__check_overlap(data_cur)
        # obj.set_data(data_cur)
        return obj

    def __check_overlap(self, dat: dict):
        # Extract the columns from DataFrame
        # print(df[self.__cols].values)
        filter_list = [
            EPostStatus5G4T.INIT, EPostStatus5G4T.COLLISION_TRAILER,
            EPostStatus5G4T.DECT_STOP
        ]
        t_list = []
        id_list = []
        for key, value in dat.items():
            if value[EProp5G4T.post_status] in filter_list:
                t_list.append([value[EProp5G4T.pos_x],
                               value[EProp5G4T.pos_y],
                               value[EProp5G4T.length],
                               value[EProp5G4T.width],
                               value[EProp5G4T.heading]])
                id_list.append(int((value[EProp5G4T.id])))
        inters_m = MathUtil.iou_batch_rotate_self(t_list)
        # inters_m, iou_matrix = MathUtil.iou_batch_rotate_self(df[self.__cols].values)
        """
        S5 With overlap area: 
            take the average position and larger size, 
            prioritize selecting the ID that has existed for a long time in the first 5 frames, 
            and then take the ID with larger size.
        """
        rows, cols = np.nonzero(inters_m)
        for i in range(rows.shape[0]):
            id1 = id_list[rows[i]]
            id2 = id_list[cols[i]]
            dat1 = dat[id1]
            dat2 = dat[id2]
            if dat1[EProp5G4T.post_status] == EPostStatus5G4T.DECT_OVERLAP \
                or dat2[EProp5G4T.post_status] == EPostStatus5G4T.DECT_OVERLAP:
                continue
            # tdf.iat[index1, tdf.columns.get_loc(post_status_str)] = EPostStatus5G4T.DECT_OVERLAP
            # tdf.iat[index2, tdf.columns.get_loc(post_status_str)] = EPostStatus5G4T.DECT_OVERLAP
            x1, y1, l1, w1, heading1 = [dat1[EProp5G4T.pos_x],
                                        dat1[EProp5G4T.pos_y],
                                        dat1[EProp5G4T.length],
                                        dat1[EProp5G4T.width],
                                        dat1[EProp5G4T.heading]]
            x2, y2, l2, w2, heading2 = [dat2[EProp5G4T.pos_x],
                                        dat2[EProp5G4T.pos_y],
                                        dat2[EProp5G4T.length],
                                        dat2[EProp5G4T.width],
                                        dat2[EProp5G4T.heading]]
            x = (x1 + x2) / 2
            y = (y1 + y2) / 2
            l = max(l1, l2)
            w = max(w1, w2)
            id = id1
            heading = heading1
            if self.__id_cnt[id1] < self.__id_cnt[id2]:
                id = id2
                heading = heading2
            elif self.__id_cnt[id1] == self.__id_cnt[id2]:
                if (l1 * w1) < (l2 * w2):
                    id = id2
                    heading = heading2
            # copy
            dat1[EProp5G4T.post_status] = EPostStatus5G4T.DECT_OVERLAP
            dat2[EProp5G4T.post_status] = EPostStatus5G4T.DECT_OVERLAP

            vals_new = dat[id][:]
            vals_new[EProp5G4T.pos_x] = x
            vals_new[EProp5G4T.pos_y] = y
            vals_new[EProp5G4T.length] = l
            vals_new[EProp5G4T.width] = w
            vals_new[EProp5G4T.heading] = heading
            vals_new[EProp5G4T.post_status] = EPostStatus5G4T.INIT
            dat[-id] = vals_new
        return dat

    def __update_value(self, vals):
        # 1. delete objects that suddenly appear on the trailer
        obj_id = vals[EProp5G4T.id]
        """
        S3.1 If vx or vy is 0 and there is a sudden change in heading 
            (30 degrees<angle<150 degrees), discard the change
        S3.2 Calculate mean object heading angle of last five frames
        S4: If an object with motion stopped is suddenly detected, discard it 
            (combined with the latest 5 frames)
        """
        if not self.__check_heading_stop_skip(vals):
            # heading_average = df.iat[index, df.columns.get_loc("heading")]
            vx_average = vals[EProp5G4T.rel_spd_lon]
            vy_average = vals[EProp5G4T.rel_spd_lat]
            length_valid = 1
            for obj in self.__data_queue:
                data_dict = obj.get_data()
                if not data_dict:
                    continue
                # elem_df = elem.obj_data_frame()
                if obj_id in data_dict:
                    vals_t = data_dict[obj_id]
                    # heading_average += t_row["heading"].iloc[0]
                    t_vx = vals_t[EProp5G4T.rel_spd_lon]
                    t_vy = vals_t[EProp5G4T.rel_spd_lon]
                    if abs(MathUtil.angle_2v([vals[EProp5G4T.rel_spd_lon], vals[EProp5G4T.rel_spd_lat]],
                                             [t_vx, t_vy])) > (math.pi / 2):
                        continue
                    vx_average += t_vx
                    vy_average += t_vy
                    length_valid += 1
                    if vals_t[EProp5G4T.post_status] != EPostStatus5G4T.DECT_STOP:
                        # if df.iat[index, df.columns.get_loc(post_status_str)] == EPostStatus5G4T.DECT_STOP:
                        vals[EProp5G4T.post_status] = EPostStatus5G4T.INIT
            vx_average /= length_valid
            vy_average /= length_valid
            # print("heading_average =--", heading_average)
            vals[EProp5G4T.heading] = math.atan2(
                vy_average, vx_average)
            vals[EProp5G4T.rel_spd_lon] = vx_average
            vals[EProp5G4T.rel_spd_lat] = vy_average
            # row["heading"] = heading_average # since python has changed the value in row
        # if df.iat[index, df.columns.get_loc(post_status_str)] == EPostStatus5G4T.DECT_STOP:
        #     return
        """
        check collisions with trailer
        """
        [x, y, l, w, a] = [vals[EProp5G4T.pos_x],
                           vals[EProp5G4T.pos_y],
                           vals[EProp5G4T.length],
                           vals[EProp5G4T.width],
                           vals[EProp5G4T.heading]]
        r2 = ((x, y), (l, w), MathUtil.rad2deg(a))
        r1 = ((self.__trailer_rect[0], self.__trailer_rect[1]),
              (self.__trailer_rect[2], self.__trailer_rect[3]),
              MathUtil.rad2deg(self.__trailer_rect[4]))
        intsec_status, intsec_pts = cv2.rotatedRectangleIntersection(r1, r2)
        if not intsec_status:
            return
        # print(obj_id, r1, r2, intsec_pts)
        """
        S1 suddenly appear on the trailer
        S2 objects with rear end collisions
        """
        t_status = vals[EProp5G4T.post_status]
        vals[EProp5G4T.post_status] = EPostStatus5G4T.COLLISION_TRAILER  # suppose the status is COLLISION_TRAILER
        for obj in self.__data_queue:
            data_dict = obj.get_data()
            if not data_dict:
                break
            # not suddenly appear on the trailer
            if obj_id in data_dict:
                vals_previous = data_dict[obj_id]
                # 2. Delete objects with rear end collisions
                # if previous status is REAR_END or current status is REAR_END
                if vals_previous[EProp5G4T.post_status] == EPostStatus5G4T.REAR_END or \
                        self.__is_rear_end_collisions(vals, vals_previous):
                    vals[EProp5G4T.post_status] = EPostStatus5G4T.REAR_END
                    # print("id ", obj_id, " in rear end collisions")
                    break
                break
        if t_status == EPostStatus5G4T.DECT_STOP and vals[EProp5G4T.post_status] == EPostStatus5G4T.COLLISION_TRAILER:
            vals[EProp5G4T.post_status] = EPostStatus5G4T.DECT_STOP

    def __check_heading_stop_skip(self, vals: list):
        # check if need to skip cal the average heading
        # S3.1 If vx or vy is 0 and there is a sudden change in heading (30 degrees<angle<150 degrees),
        # discard the change by the last frame
        obj_last: ObjectData = self.__data_queue[0]
        data_last = obj_last.get_data()

        need_discard = False
        obj_id = vals[EProp5G4T.id]
        # if obj_id == 153:
        #     print("obj.153")
        find_in_last = obj_id in data_last
        if find_in_last:
            vals_last = data_last[obj_id]
            if MathUtil.is_round_zero(vals_last[EProp5G4T.rel_spd_lat]) or \
                    MathUtil.is_round_zero(vals_last[EProp5G4T.rel_spd_lon]):
                # print("heading range1 : ",df.iat[index, df.columns.get_loc("heading")],
                #       row_raw_pre["heading"].iloc[0],
                #       df.iat[index, df.columns.get_loc("heading")] - row_raw_pre["heading"].iloc[0])
                err_deg = abs(vals[EProp5G4T.heading] - vals_last[EProp5G4T.heading])
                if err_deg > math.pi:
                    err_deg -= math.pi
                if self.__max_angle_range[
                        0] < err_deg < self.__max_angle_range[1]:
                    need_discard = True
                    vals[EProp5G4T.rel_spd_lat] = vals_last[EProp5G4T.rel_spd_lat]
                    vals[EProp5G4T.rel_spd_lon] = vals_last[EProp5G4T.rel_spd_lon]
                    vals[EProp5G4T.heading] = vals_last[EProp5G4T.heading]
                    # print("find id ", obj_id, " with heading change, time stamp", data_last.get_time_stamp())
        # S4: If an object with motion stopped is suddenly detected, discard it (combined with the latest 5 frames)
        if vals[EProp5G4T.motion] == self.__motion_st[3]:
            if find_in_last:
                vals_last = data_last[obj_id]
                if vals_last[EProp5G4T.post_status] == EPostStatus5G4T.DECT_STOP:
                    vals[EProp5G4T.post_status] = EPostStatus5G4T.DECT_STOP
                    return True
            else:
                vals[EProp5G4T.post_status] = EPostStatus5G4T.DECT_STOP
                return False
        return need_discard

    def __is_rear_end_collisions(self, vals, vals_p):
        # 2. delete objects with rear end collisions
        rear_end_collisions = False
        p2 = [self.__rotate_corners[0][1], self.__rotate_corners[1][1]]
        p3 = [self.__rotate_corners[0][2], self.__rotate_corners[1][2]]
        [x, y, l, w, a] = [vals[EProp5G4T.pos_x],
                           vals[EProp5G4T.pos_y],
                           vals[EProp5G4T.length],
                           vals[EProp5G4T.width],
                           vals[EProp5G4T.heading]]
        p_cur = MathUtil.rbbox_to_corners([x, y, l, w, a])
        x_p, y_p, l_p, w_p, a_p = [vals_p[EProp5G4T.pos_x],
                                   vals_p[EProp5G4T.pos_y],
                                   vals_p[EProp5G4T.length],
                                   vals_p[EProp5G4T.width],
                                   vals_p[EProp5G4T.heading]]
        p_pre = MathUtil.rbbox_to_corners([x_p, y_p, l_p, w_p, a_p])
        # p4
        if MathUtil.is_left([p_cur[0][3],  p_cur[1][3]], p2, p3) and \
            not MathUtil.is_left([p_pre[0][3],  p_pre[1][3]], p2, p3):
            return True
        # p1
        if MathUtil.is_left([p_cur[0][0],  p_cur[1][0]], p2, p3) and \
            not MathUtil.is_left([p_pre[0][0],  p_pre[1][0]], p2, p3):
            rear_end_collisions = True
            return True
        return rear_end_collisions


class KMFilter:
    def __init__(self, x_init, v_init = 0.0):
        self.X_posterior_prev = np.matrix([x_init, v_init, 0.], dtype=float).T
        self.P_posterior_prev = np.matrix([[0.1, 0., 0.],
                                           [0., 0.1, 0.],
                                           [0., 0., 0.0001]],
                                          dtype=float)

    def km_filter(self, dt, sig):
        """
        Prediction func:
        x(k)   = x(k-1)   + x'(k-1)*dt + x''(k-1)*(dt)^2 * (1/2!)  + Q1
        x'(k)  = 0*x(k-1) + x'(k-1)    + x''(k-1)*dt               + Q2
        x''(k) = 0*x(k-1) + 0*x'(k-1)  + x''(k-1)                  + Q3
        State variable:
        X = [x(k), x'(k), x''(k)].T

        F = [[1, dt, 0.5*dt^2],
             [0,  1,    dt   ],
             [0,  0,     1   ]]

        Q = [[Q1, 0,  0],
             [ 0, Q2, 0],
             [ 0, 0,  Q3]]

        Q1 ~ N(0, 1); Q2 ~ N(0, 0.01); Q3 ~ N(0, 0.001)
        X(k) = F * X(k-1) + Q

        Measurement func:
        Z(k) = H * X(k) + R
        R ~ N(0, 1)
        H = [1, 0, 0]
        """

        F2 = np.matrix([[1., dt, 0.5 * dt ** 2],
                        [0., 1., dt],
                        [0., 0., 1.]], dtype=float)
        H2 = np.matrix([1., 0., 0.], dtype=float)
        Q2 = np.matrix([[1., 0., 0.],
                        [0., 1, 0.],
                        [0., 0., 0.001]], dtype=float)
        R2 = np.matrix([10.], dtype=float)

        X_prior = F2 * self.X_posterior_prev  # 3&3 * 3&1 = (3&1)
        P_prior = F2 * self.P_posterior_prev * F2.T + Q2

        K = (P_prior * H2.T) * np.linalg.inv(H2 * P_prior * H2.T + R2)  # 3&1
        # 3&1 + 3&1 * (1 - 1&3 * 3&1)
        X_posterior1 = X_prior + K * (sig - H2 * X_prior)
        P_posterior1 = (np.identity(n=3, dtype=float) - K * H2) * P_prior

        self.X_posterior_prev = X_posterior1
        self.P_posterior_prev = P_posterior1

        return self.X_posterior_prev[0, 0], self.X_posterior_prev[1, 0]

    def km_extrapolate(self, dt):
        """
        This extrapolation method updates the global state, instead of
        only outputing the extrapolated x, and the dt is identified with
        the one in km_km_filter. This dt is the diff between the adjacent two
        images.
        However, the P update differs a little from the km_filter(), because
        the Q is a constant element. Since the Q is added once in km_filter()
        when the dt is 3s, but will be added 2 more times at 1s and 2s in
        km_extra(), which cause the difference of P at 3s after switching back
        from km_extra() to km_filter().

        Parameters
        ----------
        dt : TYPE
            This dt is the delta time from two adjacent objects.
        Returns
        -------
        X_extra : np.ndarray
            DESCRIPTION.

        """
        F2 = np.matrix([[1., dt, 0.5 * dt ** 2],
                        [0., 1., dt],
                        [0., 0., 1.]], dtype=float)
        self.X_posterior_prev = F2 * self.X_posterior_prev
        self.P_posterior_prev = F2 * self.P_posterior_prev * F2.T
        return self.X_posterior_prev[0, 0], self.X_posterior_prev[1, 0]

class PostProcessIPM:
    def __init__(self, his_max_length=5):
        # self.__key_dic = {"Identifier": "id",
        #                  "LongitudinalDistance": "pos_x",
        #                  "AbsoluteSpeed": "abs_spd",
        #                  "OrientationAngle": "heading",
        #                  "ExistenceProbability": "prob",
        #                  "Class": "class", #5
        #                  "DetectionStatus": "status", #6
        #                  "MotionStatus": "motion", #7
        #                  "LateralDistance": "pos_y",
        #                  "Width": "width",
        #                  "Lane": "Lane",
        #                  "BrakeLight": "BrakeLight",
        #                  "RelativeVelocity": "rel_spd",
        #                  "CutInCutOut": "CutInCutOut",
        #                  "Length": "length",
        #                  "Height": "height"
        #                  }
        self.__his_max_length: int = his_max_length
        self.__data_queue = deque(maxlen=his_max_length)
        self.__empty = True
        self.__id_cnt = collections.Counter()
        self.__x:dict = {}  #key: id, value: KMFilter()
        self.__y:dict = {}

    def add_object(self, obj: ObjectData) -> ObjectData:
        data_cur = self.__update_obj(obj)
        if len(self.__data_queue) == self.__his_max_length:
            t_obj = self.__data_queue[0]
            if t_obj.get_data():
                self.__id_cnt.subtract(t_obj.get_id_list())
        self.__data_queue.append(data_cur)
        return self.__data_queue[-1]
    
    def __update_obj(self, obj_cur: ObjectData):
        data_cur = obj_cur.get_data()
        # print(self.__empty, "---------", data_cur,  data_cur.keys(), self.__id_cnt)
        for key, value in data_cur.items():
            id = int(value[EPropIPM.id])
            if self.__empty or not self.__id_cnt.get(id):
                self.__x[id] = KMFilter(value[EPropIPM.pos_x], value[EPropIPM.rel_spd_lon])
                self.__y[id] = KMFilter(value[EPropIPM.pos_y], value[EPropIPM.rel_spd_lat])
                continue
            for obj_pre in self.__data_queue:
                data_pre = obj_pre.get_data()
                if not data_pre:
                    continue
                if id not in data_pre:
                    continue
                deta_t = float(obj_cur.get_time_stamp()) - float(
                    obj_pre.get_time_stamp())
                if MathUtil.is_near_0(deta_t):
                    continue
                x_vx = self.__x[id].km_filter(deta_t, value[EPropIPM.pos_x])
                y_vy = self.__y[id].km_filter(deta_t, value[EPropIPM.pos_y])
                value[EPropIPM.pos_x] = x_vx[0]
                value[EPropIPM.rel_spd_lon] = x_vx[1]
                value[EPropIPM.pos_y] = y_vy[0]
                value[EPropIPM.rel_spd_lat] = y_vy[1]

                # data_cur.iat[row_index, data_cur.columns.get_loc("rel_spd_lon")] \
                #     = (row.pos_x -row_last.pos_x.iloc[0])/deta_t
                # data_cur.iat[row_index, data_cur.columns.get_loc("rel_spd_lat")] \
                #     = (row.pos_y -row_last.pos_y.iloc[0])/deta_t
                break

        if self.__empty:
            if data_cur:
                self.__id_cnt = collections.Counter(
                    data_cur.keys())
            self.__empty = not self.__empty
            return obj_cur
        if data_cur:
            if self.__id_cnt is None:
                self.__id_cnt = collections.Counter(
                    data_cur.keys())
            else:
                self.__id_cnt.update(data_cur.keys())
        else:
            return obj_cur
        return obj_cur