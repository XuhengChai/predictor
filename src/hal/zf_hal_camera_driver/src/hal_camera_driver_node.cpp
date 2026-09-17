#include "zf_hal_camera_driver/hal_camera_driver.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  /*创建对应节点的共享指针对象*/
  // auto node = std::make_shared<MinimalDepthSubscriber>();
  const rclcpp::NodeOptions options;
  auto node = std::make_shared<NS_ZF::ImageConverter>(options, "image_convert");
  /* 运行节点，并检测退出信号*/
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
// #include <chrono>
// #include <opencv2/opencv.hpp>
// #define  millisecond 1000000
// #define DEBUG_PRINT(...)  printf( __VA_ARGS__); printf("\n")
// #define DEBUG_TIME(time_) auto time_ =std::chrono::high_resolution_clock::now()
// #define RUN_TIME(time_)  (double)(time_).count()/millisecond
// using namespace std;

// cv::Mat image_resize(cv::Mat image, int width, int height, int interpolation, int num) {
//     cv::Mat dest;
//     for (int i = 0; i < num; ++i) {
//         cv::resize(image, dest, cv::Size(width, height), 0, 0, interpolation);//最近邻插值
//     }
//     return dest;
// }


// int main() {
//     string path = "./1.jpg";
//     cv::Mat image = cv::imread(path);
//     //cv::resize(image, image, cv::Size(1000, 1000));
//     //DEBUG_PRINT("image input size:%dx%d", image.rows, image.cols);

//     int re_width = 1280;
//     int re_height = 640;
//     int  num = 10;
//     cv::Mat image2X_INTER_NEAREST;
//     cv::Mat image2X_INTER_LINEAR;
//     cv::Mat image2X_INTER_AREA;
//     cv::Mat image2X_INTER_CUBIC;
//     cv::Mat initMat;
//     DEBUG_PRINT("image input size:%dx%d", image.rows, image.cols);
//     DEBUG_TIME(T0);
//     image2X_INTER_NEAREST = image_resize(image, re_width, re_height, cv::INTER_NEAREST, num);
//     DEBUG_TIME(T1);
//     image2X_INTER_LINEAR = image_resize(image, re_width, re_height, cv::INTER_LINEAR, num);
//     DEBUG_TIME(T2);
//     image2X_INTER_AREA = image_resize(image, re_width, re_height, cv::INTER_AREA, num);
//     DEBUG_TIME(T3);
//     image2X_INTER_CUBIC = image_resize(image, re_width, re_height, cv::INTER_CUBIC, num);
//     DEBUG_TIME(T4);
//     DEBUG_PRINT("resize_image:%dx%d,INTER_NEAREST:%3.3fms",
//         image2X_INTER_NEAREST.rows,
//         image2X_INTER_NEAREST.cols,
//         RUN_TIME(T1 - T0) / num);
//     DEBUG_PRINT("resize_image:%dx%d,INTER_LINEAR :%3.3fms",
//         image2X_INTER_LINEAR.rows,
//         image2X_INTER_LINEAR.cols,
//         RUN_TIME(T2 - T1) / num);
//     DEBUG_PRINT("resize_image:%dx%d,INTER_AREA   :%3.3fms",
//         image2X_INTER_AREA.rows,
//         image2X_INTER_AREA.cols,
//         RUN_TIME(T3 - T2) / num);
//     DEBUG_PRINT("resize_image:%dx%d,INTER_CUBIC  :%3.3fms",
//         image2X_INTER_CUBIC.rows,
//         image2X_INTER_CUBIC.cols,
//         RUN_TIME(T4 - T3) / num);
//     return 0;
// }