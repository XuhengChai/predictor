#ifndef ZF_CAM_MODEL_SCARA_H
#define ZF_CAM_MODEL_SCARA_H

#include "zf_cam_model/polynomial.h"
#include "zf_cam_model/zf_cam_model_base.h"
#include "zf_global/in/zf_detect_global.h"

BEGIN_NS_ZF_DETECTION

class CameraModelScara : public BaseCameraModel {
 public:
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  CameraModelScara() = default;
  ~CameraModelScara() = default;

  Eigen::Vector2d Project(const Eigen::Vector3d& point3d) override;
  Eigen::Vector3d UnProject(const Eigen::Vector2d& point2d) override;
  std::string name() const override { return "CameraModelScara"; }
  bool set_params(const std::string& yamlFile) override;
  void undistort_polyconic(cv::Mat &mapx, cv::Mat &mapy) override;
  void undistort_perspective(cv::Mat &mapx, cv::Mat &mapy, float f = 1.0) override;
  void undistort_bev(cv::Mat &mapx, cv::Mat &mapy, double mat33[9], float f = 10.0) override;
  // bool set_params(size_t width, size_t height,
  //                 const Eigen::VectorXf& params) override;
  //   std::shared_ptr<BaseCameraModel> get_camera_model() override;

 protected:
  Eigen::Matrix3d intrinsic_params_;
  Polynomial cam2world_;
  Polynomial world2cam_;
  float center_[2];  // x, y
  float affine_[3];  // c, d, e
};

END_NS_ZF_DETECTION
#endif  // ZF_CAM_MODEL_SCARA_H
