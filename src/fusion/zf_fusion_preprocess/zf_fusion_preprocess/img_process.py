import copy
import rclpy
import math
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.time import Time
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
import cv2

class ImgPreprocess(Node):
    def __init__(self):
        super().__init__('img_preprocess_sub')
        qos_profile = QoSProfile(
            # reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=10
        )
        self._bridge = CvBridge()
        self._subscription = self.create_subscription(
            Image,
            # "/cam1/camera/image_raw",
            "/cam1/image_converter/img",
            self.img_data_callback,
            qos_profile)
        self._subscription  # prevent unused variable warning
        # self._sub_tracking = self.create_subscription(
        #     Image,
        #     # "/cam1/camera/image_raw",
        #     "/cam1/image_converter/img",
        #     self.img_tracking_callback,
        #     qos_profile)
        # self._sub_tracking  # prevent unused variable warning
        # self._sub_hmi = self.create_subscription(
        #     Image,
        #     # "/cam1/camera/image_raw",
        #     "/cam1/image_converter/img",
        #     self.img_hmi_callback,
        #     qos_profile)
        # self._sub_hmi  # prevent unused variable warning

        # self._sub_compressed = self.create_subscription(
        #     CompressedImage,
        #     "/cam1/camera/image_raw/compressed",
        #     self.img_compressed_callback,
        #     qos_profile)
        # self._sub_compressed  # prevent unused variable warning


        self._last_time_stamp = 0
        self._last_header = 0

    def img_data_callback(self, msg):
        # self.show_elapsed_time(msg)
        try:
            imgMsg = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.show_elapsed_time(msg)
        print()
        self.showImage(imgMsg)


    def img_tracking_callback(self, msg):
        try:
            imgMsg = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        curr_time_stamp = self.get_clock().now()
        elapsed_time = (curr_time_stamp - Time.from_msg(msg.header.stamp)).nanoseconds * 1e-9
        self.get_logger().info('[tracking]: elapsed_time: "%.4f"' % (elapsed_time))

    def img_hmi_callback(self, msg):
        try:
            imgMsg = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        curr_time_stamp = self.get_clock().now()
        elapsed_time = (curr_time_stamp - Time.from_msg(msg.header.stamp)).nanoseconds * 1e-9
        self.get_logger().info('----- hmi elapsed_time: "%.4f"' % (elapsed_time))
        self.showImage(imgMsg)


    def img_compressed_callback(self, msg):
        # print("img_compressed_callback")
        try:
            imgMsg = self._bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.showImage(imgMsg)


    def show_elapsed_time(self, msg):
        if not self._last_time_stamp:
            self._last_time_stamp = self.get_clock().now()
            return
        if not self._last_header:
            self._last_header = msg.header.stamp
            return
        curr_time_stamp = self.get_clock().now()
        # print(curr_time_stamp)
        elapsed_time = (curr_time_stamp - Time.from_msg(msg.header.stamp)).nanoseconds * 1e-9
        self.get_logger().info('elapsed_time: "%.4f"' % (elapsed_time))

        elapsed_time = (Time.from_msg(msg.header.stamp) - Time.from_msg(self._last_header)).nanoseconds * 1e-9
        # elapsed_time = (curr_time_stamp - self._last_time_stamp).nanoseconds * 1e-9
        # self.get_logger().info('I heard: "%.4f, %.2f"' % (elapsed_time, 1.0 / elapsed_time))
        self._last_time_stamp = curr_time_stamp
        self._last_header = msg.header.stamp


        # # self.get_logger().info('I heard: "%s"' % msg.data)
        # print("I heard: radar_datas: ", len(msg.radar_datas))
        # print("I heard: vehicle_datas: ", len(msg.vehicle_datas))
        # print("I heard: ipm_datas: ", len(msg.ipm_datas))

    def showImage(self, img):
        cv2.imshow('image', img)
        cv2.waitKey(1)

def main(args=None):
    print('Hi from zf_fusion__img_preprocess.')
    rclpy.init(args=args)
    minimal_subscriber = ImgPreprocess()
    rclpy.spin(minimal_subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


# if __name__ == '__main__':
#     main()
