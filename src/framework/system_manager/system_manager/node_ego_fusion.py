import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from interface.msg import DetectedEgoInfo, NodeState, VehicleInfo
# from python folder under package import filename
from can_msgs.msg import CanMsgData, CanDatas, CanSigData

class EgoFusionNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("System manager node initialization start...")
        qos_profile_best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        # qos_profile_reliable = QoSProfile(
        #     reliability=QoSReliabilityPolicy.RELIABLE,
        #     history=QoSHistoryPolicy.KEEP_LAST,
        #     depth=1)
        # self.declare_parameter('use_image_raw', True)
        # self.declare_parameter('show_log_in_terminal', True)
        # self.declare_parameter('save_log_in_file', True)
        # self.use_image_raw = self.get_parameter('use_image_raw').value
        # self.show_log_in_terminal = self.get_parameter(
        #                                        'show_log_in_terminal').value
        # self.save_log_in_file = self.get_parameter('save_log_in_file').value

        # Timers 50 ms for fused angle
        self.timer = self.create_timer(
            0.05, self.timer_callback)
        # Subscriptions
        self.sub_ego_side_cam = self.create_subscription(
            DetectedEgoInfo, "/ego_info_side_cam", self.cbk_ego_side_cam,
            qos_profile_best_effort)

        self.sub_ego_front_cam = self.create_subscription(
            DetectedEgoInfo, "/ego_info_front_cam", self.cbk_ego_front_cam,
            qos_profile_best_effort)
        
        self.sub_ego_kinematics = self.create_subscription(
            DetectedEgoInfo, "/ego_info_kinematics", self.cbk_ego_kinematics,
            qos_profile_best_effort)
        # Define a publisher object for publishing egostate messages.
        self.egoinfo_msg = VehicleInfo()
        self.egoinfo_pub = self.create_publisher(VehicleInfo, "/vehicle_info",
                                                  qos_profile_best_effort)
        # Publishers node_state
        self.node_state_msg = NodeState()
        self.node_state_msg.node_state = 10
        self.node_state_msg.watchdog_signal = 0
        self.pub_node_state = self.create_publisher(
            NodeState, "/ego_state", qos_profile_best_effort)
        # Publishers sync info
        self.record_dic_ego_sync = {
            'v_trailer': self.egoinfo_msg.v_trailer,
            'wheel_spd': self.egoinfo_msg.wheel_spd,
            'artic_angle': self.egoinfo_msg.artic_angle,
            'artic_angle_confid': self.egoinfo_msg.artic_angle_confid,
            'turning_light_switch': self.egoinfo_msg.turning_light_switch,
        }
        self.ego_sync_datas = CanDatas()
        self.egoinfo_pub_sync = self.create_publisher(
            CanDatas, "can0/hal/from_packed_radar_data",
            qos_profile_best_effort)

    def timer_callback(self):
        # update ego info to planning
        self.egoinfo_msg.artic_angle, self.egoinfo_msg.artic_angle_confid = self.fused_angle()
        #TODO
        self.egoinfo_msg.v_trailer = 10.0
        self.egoinfo_msg.wheel_spd = 10.0
        self.egoinfo_msg.turning_light_switch = 1
        self.egoinfo_msg.trailer_wheelbase = 2.5
        self.egoinfo_msg.header.stamp = self.get_clock().now().to_msg()
        self.egoinfo_pub.publish(self.egoinfo_msg)

        # update ego_sync msg: CanDatas
        self.ego_sync_datas.header.stamp = self.egoinfo_msg.header.stamp
        t_msg_data = CanMsgData()
        for name, val in self.record_dic_ego_sync.items():
            t_sig = CanSigData()
            t_sig.sig_name = name
            t_sig.sig_data = float(getattr(self.egoinfo_msg, name))
            t_msg_data.sig_datas.append(t_sig)
        self.ego_sync_datas.msg_datas = [t_msg_data]
        self.ego_sync_datas.data_size = len(self.ego_sync_datas.msg_datas)
        self.egoinfo_pub_sync.publish(self.ego_sync_datas)

        # update ego node state
        self.node_state_msg.node_state = 30
        self.node_state_msg.watchdog_signal = not self.node_state_msg.watchdog_signal
        self.node_state_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_node_state.publish(self.node_state_msg)
    
    def cbk_ego_side_cam(self, data:DetectedEgoInfo):
        self.egoinfo_msg.aa_cv_side = data.articulation_angle
        self.egoinfo_msg.aa_confid_cv_side = data.confidence

    def cbk_ego_front_cam(self, data:DetectedEgoInfo):
        self.egoinfo_msg.aa_cv_front = data.articulation_angle
        self.egoinfo_msg.aa_confid_cv_front = data.confidence

    def cbk_ego_kinematics(self, data:DetectedEgoInfo):
        self.egoinfo_msg.aa_vd = data.articulation_angle
        self.egoinfo_msg.aa_confid_vd = data.confidence

    def fused_angle(self):
        confid_sum = self.egoinfo_msg.aa_confid_cv_side + \
                     self.egoinfo_msg.aa_confid_cv_front + \
                     self.egoinfo_msg.aa_confid_vd + 0.01
        confid_side = self.egoinfo_msg.aa_confid_cv_side/confid_sum
        confid_front = self.egoinfo_msg.aa_confid_cv_front/confid_sum
        confid_vd = self.egoinfo_msg.aa_confid_vd/confid_sum
        angle = confid_side*self.egoinfo_msg.aa_cv_side + \
                confid_front*self.egoinfo_msg.aa_cv_front + \
                confid_vd*self.egoinfo_msg.aa_vd
        confid = max(self.egoinfo_msg.aa_confid_cv_side, 
                     self.egoinfo_msg.aa_confid_cv_side, 
                     self.egoinfo_msg.aa_confid_cv_side)
        return angle, confid