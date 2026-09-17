import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from interface.msg import DetectedEgoInfo, NodeState, VehicleInfo
# from python folder under package import filename
from can_msgs.msg import CanMsgData, CanDatas, CanSigData

class EgoKinematicsNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("Ego Kinematics node initialization start...")
        qos_profile_best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        # qos_profile_reliable = QoSProfile(
        #     reliability=QoSReliabilityPolicy.RELIABLE,
        #     history=QoSHistoryPolicy.KEEP_LAST,
        #     depth=1)

        # Timers 50 ms for fused angle
        self.timer = self.create_timer(
            0.05, self.timer_callback)
        self.egostate_sig_list = ['reference_torque',
                            'max_available_torque',
                            'friction_torque',
                            'tsmo_rpm',
                            'tsmi_rpm',
                            'shift_process',
                            'gear_ratio',
                            'current_gear',
                            'hrw_speed_fl',
                            'hrw_speed_fr',
                            'weight_amt',
                            'weight_ebs',
                            'tractor_yawrate',
                            'wheel_angle',
                            'tach_speed'
                            ]
        self.egoinfo_sub_can = self.create_subscription(
            CanMsgData, "can0/hal/from_can_msg_data",self.cbk_parse_canmsg,
            qos_profile_best_effort)
        
        
        self.egoinfo = VehicleInfo()
        
        self.ego_kinematics_msg = DetectedEgoInfo()
        self.pub_ego_kinematics = self.create_publisher(DetectedEgoInfo, "/ego_info_kinematics",
                                                  qos_profile_best_effort)
        # Publishers node_state
        self.node_state_msg = NodeState()
        self.node_state_msg.node_state = 10
        self.node_state_msg.watchdog_signal = 0
        self.pub_node_state = self.create_publisher(
            NodeState, "/ego_kinematics_state", qos_profile_best_effort)


    def timer_callback(self):
        #TODO
        self.ego_kinematics_msg.articulation_angle = 10.0
        self.ego_kinematics_msg.confidence = 0.8
        self.pub_ego_kinematics.publish(self.ego_kinematics_msg)

        # update ego node state
        self.node_state_msg.node_state = 30
        self.node_state_msg.watchdog_signal = not self.node_state_msg.watchdog_signal
        self.node_state_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_node_state.publish(self.node_state_msg)
    
    def cbk_parse_canmsg(self, data:CanMsgData):
        # startpoint_begin_ = self.get_clock().now()
        # curr_time_stamp = self.get_clock().now()
        # elapsed_time = (curr_time_stamp - Time.from_msg(data.header.stamp)).nanoseconds * 1e-9
        # self.get_logger().info('[CAN]: elapsed_time: "%.4f"' %(elapsed_time))
        # 判断输入是否在列表中，并执行相应的操作
        for sig in data.sig_datas:
            if sig.sig_name in self.egostate_sig_list:
                setattr(self.egoinfo, sig.sig_name, type(getattr(self.egoinfo, sig.sig_name))(sig.sig_data))
            # elif sig.sig_name == "tsmi_rpm":
            #     if self.egostate.shift_process:
            #         self.egostate.tsmi_rpm = self.egostate.tsmi_rpm_prev
            #     else:
            #         self.egostate.tsmi_rpm = type(self.egostate.tsmi_rpm)(sig.sig_data)
            
    
def main(args=None):
    rclpy.init(args=args)
    node = EgoKinematicsNode("node_ego_kinematics")
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    # rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()