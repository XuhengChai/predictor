#ifndef ZF_CAM_MODEL_WIDE_H
#define ZF_CAM_MODEL_WIDE_H

#include "zf_cam_model/polynomial.h"
#include "zf_cam_model/zf_cam_model_base.h"
#include "zf_global/in/zf_detect_global.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

BEGIN_NS_ZF_DETECTION

class CameraModelWide : public BaseCameraModel {
 public:
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  CameraModelWide() = default;
  ~CameraModelWide() = default;

  Eigen::Vector2d Project(const Eigen::Vector3d& point3d) override;
  Eigen::Vector3d UnProject(const Eigen::Vector2d& point2d) override;
  std::string name() const override { return "CameraModelScara"; }
  bool set_params(const std::string& yamlFile) override;

  // bool set_params(size_t width, size_t height,
  //                 const Eigen::VectorXf& params) override;
  //   std::shared_ptr<BaseCameraModel> get_camera_model() override;

 protected:
  Eigen::Matrix3d intrinsic_params_;
  Eigen::Matrix2d dist_coeffs_;
  Polynomial cam2world_;
  Polynomial world2cam_;
  float fx_;
  float fy_;
  float cx_;
  float cy_;
  float k_[4];  // k1, k2, k3, k4
};

END_NS_ZF_DETECTION
#endif  // ZF_CAM_MODEL_WIDE_H
