#include "zf_cam_model/zf_cam_model_wide.h"

#include <math.h>
#include <opencv2/opencv.hpp>
#include <fstream>

BEGIN_NS_ZF_DETECTION

// world2cam
Eigen::Vector2d CameraModelWide::Project(const Eigen::Vector3d& point3d) {
  // if (std::isgreater(point3d[2], 0.f)) {
  //   LOG_ERROR() << "The input point (" << point3d
  //               << ") should be in front of the camera";
  // }

  // rotate:
  // [0 1 0;
  //  1 0 0;
  //  0 0 -1];
  double x[3] = {point3d(1), point3d(0), -point3d(2)};
  const double norm = sqrt(x[0] * x[0] + x[1] * x[1]);

  Eigen::Vector2d projection;
  // if (norm < std::numeric_limits<double>::epsilon()) {
  //   projection(0) = center_[1];  // yc
  //   projection(1) = center_[0];  // xc
  //   return projection;
  // }

  // const double theta = atan(x[2] / norm);
  // const double rho = world2cam_(theta);

  // const float u = static_cast<float>(x[0] / norm * rho);
  // const float v = static_cast<float>(x[1] / norm * rho);
  // projection(1) = affine_[0] * u + affine_[1] * v + center_[0];
  // projection(0) = affine_[2] * u + v + center_[1];
  return projection;
}

// cam2world
Eigen::Vector3d CameraModelWide::UnProject(const Eigen::Vector2d& point2d) {
  // rotate:
  // [0 1 0;
  //  1 0 0;
  //  0 0 -1];

  // 1/det(A), where A = [c,d;e,1] as in the Matlab file
  // double invdet = 1 / (affine_[0] - affine_[1] * affine_[2]);
  // double xp = invdet * ((point2d(1) - center_[0]) -
  //                       affine_[1] * (point2d(0) - center_[1]));
  // double yp = invdet * (-affine_[2] * (point2d(1) - center_[0]) +
  //                       affine_[0] * (point2d(0) - center_[1]));
  // // distance [pixels] of  the point from the image center
  // double r = sqrt(xp * xp + yp * yp);
  // const double zp = cam2world_(r);

  // // printf(" yp (%f, %f,%f, %f)\n", xp, yp, zp, xc);

  // // normalize to unit norm
  // double invnorm = 1 / sqrt(xp * xp + yp * yp + zp * zp);
  Eigen::Vector3d projection;
  // projection(1) = invnorm * xp;
  // projection(0) = invnorm * yp;
  // projection(2) = -1 * invnorm * zp;
  return projection;
}

// std::shared_ptr<BaseCameraModel> CameraModelWide::get_camera_model() {
//   std::shared_ptr<PinholeCameraModel> camera_model(new PinholeCameraModel());
//   camera_model->set_width(width_);
//   camera_model->set_height(height_);
//   camera_model->set_intrinsic_params(intrinsic_params_);

//   return std::dynamic_pointer_cast<BaseCameraModel>(camera_model);
// }

bool CameraModelWide::set_params(const std::string& yamlFile) {
  std::ifstream file(yamlFile, std::iostream::binary | std::ios::ate);
  if (!file.good()) {
    return false;
  }

  file.exceptions(std::ifstream::badbit | std::ifstream::failbit |
                  std::ifstream::eofbit);
  auto length(file.tellg());
  std::string buffer(length, '\0');
  file.seekg(0);
  file.read(&buffer[0], length);
  if (buffer.empty()) {
    return false;
  }
  cv::String dataString = "%YAML:1.0\n" + buffer;
  cv::FileStorage n(dataString,
                     cv::FileStorage::READ | cv::FileStorage::MEMORY);

  n["width"] >> image_width_;
  n["height"] >> image_height_;
  n["fx"] >> fx_;
  n["fy"] >> fy_;
  n["cx"] >> cx_;
  n["cy"] >> cy_;
  n["k1"] >> k_[0];
  n["k2"] >> k_[1];
  n["k3"] >> k_[2];
  n["k4"] >> k_[3];

  intrinsic_params_ << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;
  dist_coeffs_ << k_[0], k_[2], k_[3], k_[4];
  return true;
}

// bool CameraModelWide::set_params(size_t width, size_t height,
//                                   const Eigen::VectorXf& params) {
//   if (params.size() < 9) {
//     AINFO << "Missing cam2world and world2cam model.";
//     return false;
//   }

//   uint32_t cam2world_order = uint32_t(params(8));
//   AINFO << "cam2world order: " << cam2world_order << ", size: " <<
//   params.size()
//         << std::endl;

//   if (params.size() < 9 + cam2world_order + 1) {
//     AINFO << "Incomplete cam2world model or missing world2cam model.";
//     return false;
//   }

//   uint32_t world2cam_order = uint32_t(params(9 + cam2world_order));
//   AINFO << "world2cam order: " << world2cam_order << ", size: " <<
//   params.size()
//         << std::endl;

//   if (params.size() < 9 + cam2world_order + 1 + world2cam_order) {
//     AINFO << "Incomplete world2cam model.";
//     return false;
//   }

//   width_ = width;
//   height_ = height;

//   center_[0] = params(0);
//   center_[1] = params(1);

//   affine_[0] = params(2);
//   affine_[1] = params(3);
//   affine_[2] = params(4);

//   intrinsic_params_ = Eigen::Matrix3f::Identity();
//   intrinsic_params_(0, 0) = params(5);
//   intrinsic_params_(1, 1) = params(5);
//   intrinsic_params_(0, 2) = params(6);
//   intrinsic_params_(1, 2) = params(7);

//   for (size_t i = 0; i < cam2world_order; ++i) {
//     cam2world_[static_cast<uint32_t>(i)] = static_cast<double>(params(9 +
//     i));
//   }

//   for (size_t i = 0; i < world2cam_order; ++i) {
//     world2cam_[static_cast<uint32_t>(i)] =
//         static_cast<double>(params(10 + cam2world_order + i));
//   }

//   return true;
// }

END_NS_ZF_DETECTION
