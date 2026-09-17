#ifndef ZF_CAM_MODEL_BASE_H
#define ZF_CAM_MODEL_BASE_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>

#include "zf_global/in/zf_detect_global.h"

BEGIN_NS_ZF_DETECTION

class BaseCameraModel {
 public:
  BaseCameraModel() = default;
  virtual ~BaseCameraModel(){};

  virtual Eigen::Vector2d Project(const Eigen::Vector3d& point3d) = 0;
  virtual Eigen::Vector3d UnProject(const Eigen::Vector2d& point2d) = 0;
  virtual std::string name() const = 0;
  virtual bool set_params(const std::string& yamlFile) = 0;
  virtual void undistort_polyconic(cv::Mat &mapx, cv::Mat &mapy) {};
  virtual void undistort_perspective(cv::Mat &mapx, cv::Mat &mapy, float f = 1.0) {};
  virtual void undistort_bev(cv::Mat &mapx, cv::Mat &mapy, double mat33[9], float f = 10.0) {};
  // virtual bool set_params(int width, int height,
  //                         const Eigen::VectorXf& params) = 0;
  inline void set_width(int width) { image_width_ = width; }
  inline void set_height(int height) { image_height_ = height; }

  inline int get_width() const { return image_width_; }
  inline int get_height() const { return image_height_; }

 protected:
  int image_width_ = 0;
  int image_height_ = 0;
};

END_NS_ZF_DETECTION
#endif  // ZF_CAM_MODEL_BASE_H