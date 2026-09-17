#include "zf_cam_model/zf_cam_model_scara.h"

#include <math.h>

#include <fstream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

BEGIN_NS_ZF_DETECTION

// world2cam
Eigen::Vector2d CameraModelScara::Project(const Eigen::Vector3d &point3d) {
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
  if (norm < std::numeric_limits<double>::epsilon()) {
    projection(0) = center_[1];  // yc
    projection(1) = center_[0];  // xc
    return projection;
  }

  const double theta = atan(x[2] / norm);
  const double rho = world2cam_(theta);

  const float u = static_cast<float>(x[0] / norm * rho);
  const float v = static_cast<float>(x[1] / norm * rho);
  projection(1) = affine_[0] * u + affine_[1] * v + center_[0];
  projection(0) = affine_[2] * u + v + center_[1];
  return projection;
}

// cam2world
Eigen::Vector3d CameraModelScara::UnProject(const Eigen::Vector2d &point2d) {
  // rotate:
  // [0 1 0;
  //  1 0 0;
  //  0 0 -1];

  // 1/det(A), where A = [c,d;e,1] as in the Matlab file
  double invdet = 1 / (affine_[0] - affine_[1] * affine_[2]);
  double xp = invdet * ((point2d(1) - center_[0]) -
                        affine_[1] * (point2d(0) - center_[1]));
  double yp = invdet * (-affine_[2] * (point2d(1) - center_[0]) +
                        affine_[0] * (point2d(0) - center_[1]));
  // distance [pixels] of  the point from the image center
  double r = sqrt(xp * xp + yp * yp);
  const double zp = cam2world_(r);

  // printf(" yp (%f, %f,%f, %f)\n", xp, yp, zp, xc);

  // normalize to unit norm
  double invnorm = 1 / sqrt(xp * xp + yp * yp + zp * zp);
  Eigen::Vector3d projection;
  projection(1) = invnorm * xp;
  projection(0) = invnorm * yp;
  projection(2) = -1 * invnorm * zp;
  return projection;
}

// std::shared_ptr<BaseCameraModel> CameraModelScara::get_camera_model() {
//   std::shared_ptr<PinholeCameraModel> camera_model(new PinholeCameraModel());
//   camera_model->set_width(width_);
//   camera_model->set_height(height_);
//   camera_model->set_intrinsic_params(intrinsic_params_);

//   return std::dynamic_pointer_cast<BaseCameraModel>(camera_model);
// }

void CameraModelScara::undistort_polyconic(cv::Mat &mapx, cv::Mat &mapy) {
  // std::string prefix = "/home/nvidia/params/";
  // cv::FileStorage fs(prefix + "mapx.xml", cv::FileStorage::READ);
  // fs["mapx"] >> mapx;
  // fs.release();
  // fs.open(prefix + "mapy.xml", cv::FileStorage::READ);
  // fs["mapy"] >> mapy;
  // fs.release();
  int H = image_height_;
  int W = image_width_;
  double pa[] = {1,    0.1,  0,    1.5,  0.1, 0, 1.55,
                 1.45, 1.45, -0.6, 0.05, 15,  2};  // 0~12, total 13

  Eigen::VectorXd arrayW = Eigen::VectorXd::LinSpaced(W, 0, W - 1);
  Eigen::VectorXd arrayH = Eigen::VectorXd::LinSpaced(H, 0, H - 1);
  Eigen::MatrixXd x_grid =
      arrayW.transpose().replicate(H, 1);           // Size H x W 1080, 1920
  Eigen::MatrixXd y_grid = arrayH.replicate(1, W);  // Size H x W

  double phi0 = pa[11] / 180.0 * M_PI;
  double lambda0 = pa[12] / 180.0 * M_PI;
  Eigen::MatrixXd y_proj = (1.0 - y_grid.array() / H) * pa[0];

  Eigen::VectorXd x = Eigen::VectorXd::LinSpaced(H, 0, M_PI / 2);
  Eigen::VectorXd y = x.array().sin();
  Eigen::VectorXd x1d = Eigen::VectorXd::LinSpaced(H, pa[1], pa[2]);
  Eigen::VectorXd y_times_x1d = pa[3] + y.array() * x1d.array();
  Eigen::MatrixXd deta_x2d = y_times_x1d.replicate(1, W);
  Eigen::MatrixXd x_proj = (x_grid.array() - W / 2.0) / W * deta_x2d.array();
  Eigen::MatrixXd A = phi0 + y_proj.array();
  Eigen::MatrixXd B = x_proj.array().square() + A.array().square();
  Eigen::MatrixXd phi = A;
  for (int i = 0; i < 6; ++i) {
    Eigen::MatrixXd tan_phi = phi.array().tan();
    Eigen::MatrixXd deta_den = (phi - A).array() / tan_phi.array() - 1.0;
    Eigen::MatrixXd deta_phi =
        -(A.array() * (phi.array() * tan_phi.array() + 1.0) - phi.array() -
          0.5 * (phi.array().square() + B.array()) * tan_phi.array()) /
        deta_den.array();
    phi += deta_phi;
  }
  Eigen::VectorXd y1d = Eigen::VectorXd::LinSpaced(W, pa[4], pa[5]);
  Eigen::MatrixXd deta_y2d = y1d.transpose().replicate(H, 1);
  // Compute phi_alt
  // Eigen::MatrixXd phi_alt = (Eigen::MatrixXd::Ones(H, W) -
  // phi).cwiseProduct(Eigen::MatrixXd::Constant(H, W, pa[6])) + deta_y2d;
  // phi_alt = phi_alt * pa[8] + Eigen::MatrixXd::Constant(H, W, pa[9]);
  // Compute theta_alt
  // Eigen::MatrixXd tan_phi = phi.array().tan();
  // Eigen::MatrixXd arcsin_input = x_proj.cwiseProduct(tan_phi);
  // Eigen::MatrixXd arcsin_result = arcsin_input.array().asin();
  // Eigen::MatrixXd sin_phi = phi.array().sin();
  // Eigen::MatrixXd theta_alt = arcsin_result.cwiseQuotient(sin_phi) +
  // Eigen::MatrixXd::Constant(H, W, lambda0); theta_alt *= pa[7];
  Eigen::MatrixXd phi_alt = (1.0 - phi.array()) * pa[6] + deta_y2d.array();
  phi_alt = phi_alt.array() * pa[8] + pa[9];
  Eigen::MatrixXd theta_alt =
      (x_proj.array() * phi.array().tan()).asin() / phi.array().sin() + lambda0;
  theta_alt *= pa[7];
  Eigen::VectorXd lx = Eigen::VectorXd::LinSpaced(W, M_PI / 3, 2 * M_PI / 3);
  Eigen::VectorXd ly = lx.array().sin();
  ly *= pa[8];
  Eigen::MatrixXd dx_3d = ly.transpose().replicate(H, 1);
  Eigen::MatrixXd x3d =
      (theta_alt.array().sin() * phi_alt.array().cos()) * dx_3d.array();
  Eigen::MatrixXd y3d = phi_alt.array().sin();
  Eigen::MatrixXd z3d =
      theta_alt.array().cos() * phi_alt.array().cos() + pa[10];
  // Stack x, y, z into a 3 x (H*W) matrix
  Eigen::MatrixXd pointsInCam(3, H * W);
  auto x3d_row =
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(
          x3d);
  auto y3d_row =
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(
          y3d);
  auto z3d_row =
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(
          z3d);
  pointsInCam.row(0) =
      Eigen::Map<Eigen::RowVectorXd>(x3d_row.data(), x3d.size());
  pointsInCam.row(1) =
      Eigen::Map<Eigen::RowVectorXd>(y3d_row.data(), y3d.size());
  pointsInCam.row(2) =
      Eigen::Map<Eigen::RowVectorXd>(z3d_row.data(), z3d.size());
  Eigen::VectorXd mapx_flat(H * W);
  Eigen::VectorXd mapy_flat(H * W);
  for (int j = 0; j < pointsInCam.cols(); j++) {
    Eigen::Vector3d pointInCam = pointsInCam.block<3, 1>(0, j);
    pointInCam.normalize();
    Eigen::Vector2d pointInPixel = this->Project(pointInCam);
    mapx_flat(j) = pointInPixel(0);
    mapy_flat(j) = pointInPixel(1);
  }
  assert(mapx_flat.size() == H * W &&
         "Input vector size doesn't match image dimensions");
  // Reshape to HxW matrices (using RowMajor for correct memory layout)
  LOG_DEBUG() << mapx_flat.size();
  LOG_DEBUG() << mapy_flat.size();
  Eigen::MatrixXd mapx_e = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      mapx_flat.data(), H, W);
  Eigen::MatrixXd mapy_e = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      mapy_flat.data(), H, W);
  cv::eigen2cv(mapx_e, mapx);
  mapx.convertTo(mapx, CV_32FC1);
  cv::eigen2cv(mapy_e, mapy);
  mapy.convertTo(mapy, CV_32FC1);
}

void CameraModelScara::undistort_perspective(cv::Mat &mapx, cv::Mat &mapy,
                                             float f) {
  int H = image_height_;
  int W = image_width_;
  float focal = std::abs(f);  // f is a function parameter
  focal = (focal < 0.2f) ? 0.2f : focal;
  float z = static_cast<float>(W) / focal;

  // Create 0 to W-1 and H-1 vectors, then center
  Eigen::VectorXd arrayW = Eigen::VectorXd::LinSpaced(W, 0, W - 1);
  Eigen::VectorXd arrayH = Eigen::VectorXd::LinSpaced(H, 0, H - 1);
  Eigen::VectorXd x = arrayW.array() - static_cast<float>(W) / 2.0f;  // 1920, 1
  Eigen::VectorXd y = arrayH.array() - static_cast<float>(H) / 2.0f;  // 1080, 1

  // Create meshgrid using Eigen's replication n rows m cols replicate(n, m)
  Eigen::MatrixXd x_grid =
      x.transpose().replicate(H, 1);           // Size H x W 1080, 1920
  Eigen::MatrixXd y_grid = y.replicate(1, W);  // Size H x W
  // Stack into 3D points and reshape to 3x(H*W)
  Eigen::MatrixXd pointsInCam(3, H * W);
  auto x_grid_row =
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(
          x_grid);
  Eigen::VectorXd temp_mapx =
      Eigen::Map<Eigen::VectorXd>(x_grid_row.data(), x_grid_row.size());
  pointsInCam.row(0) = temp_mapx.transpose();  // Explicit copy
  auto y_grid_row =
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(
          y_grid);
  Eigen::VectorXd temp_mapy =
      Eigen::Map<Eigen::VectorXd>(y_grid_row.data(), y_grid_row.size());
  pointsInCam.row(1) = temp_mapy.transpose();  // Explicit copy
  Eigen::VectorXd temp_mapz(H * W);
  temp_mapz.setConstant(z);
  pointsInCam.row(2) = temp_mapz.transpose();
  Eigen::VectorXd mapx_flat(H * W);
  Eigen::VectorXd mapy_flat(H * W);
  for (int j = 0; j < pointsInCam.cols(); j++) {
    Eigen::Vector3d pointInCam = pointsInCam.block<3, 1>(0, j);
    pointInCam.normalize();
    Eigen::Vector2d pointInPixel = this->Project(pointInCam);
    mapx_flat(j) = pointInPixel(0);
    mapy_flat(j) = pointInPixel(1);
  }
  // assert(mapx_flat.size() == H * W && "Input vector size doesn't match image
  // dimensions"); Reshape to HxW matrices (using RowMajor for correct memory
  // layout) Eigen::Map<Eigen::Matrix<double, 1080, 1920>>
  // mapx_e(mapx_flat.data(), H, W); Eigen::Map<Eigen::Matrix<double, 1080,
  // 1920>> mapy_e(mapy_flat.data(), H, W);
  LOG_DEBUG() << mapx_flat.size();
  LOG_DEBUG() << mapy_flat.size();
  Eigen::MatrixXd mapx_e = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      mapx_flat.data(), H, W);
  Eigen::MatrixXd mapy_e = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      mapy_flat.data(), H, W);

  cv::eigen2cv(mapx_e, mapx);
  mapx.convertTo(mapx, CV_32FC1);
  // cv::transpose(mapx, mapx);
  cv::eigen2cv(mapy_e, mapy);
  mapy.convertTo(mapy, CV_32FC1);
  // cv::transpose(mapy, mapy);
}

void CameraModelScara::undistort_bev(cv::Mat &mapx, cv::Mat &mapy, double mat33[9], float f) {
  int H = image_height_;
  int W = image_width_;
  Eigen::Matrix3d M = Eigen::Map<Eigen::Matrix3d>(mat33, 3, 3).transpose();
  // M << 3.25358679e-01, -6.89051449e-01, 5.93062013e+02, -5.87948257e-02,
  //     -2.84274759e-01, 4.88433284e+02, -6.05425658e-05, -7.40327129e-04,
  //     1.00000000e+00;

  Eigen::VectorXd arrayW = Eigen::VectorXd::LinSpaced(W, 0, W - 1);
  Eigen::VectorXd arrayH = Eigen::VectorXd::LinSpaced(H, 0, H - 1);
  Eigen::MatrixXd x_grid =
      arrayW.transpose().replicate(H, 1);           // Size H x W 1080, 1920
  Eigen::MatrixXd y_grid = arrayH.replicate(1, W);  // Size H x W
  float focal = std::abs(f);                        // f is a function parameter
  focal = (focal < 0.2f) ? 0.2f : focal;
  float z = static_cast<float>(W) / focal;
  // If M is NaN, assign default matrix
  // if (M.array().isNaN().any()) {
  //     M << 3.25358679e-01, -6.89051449e-01, 5.93062013e+02,
  //            -5.87948257e-02, -2.84274759e-01, 4.88433284e+02,
  //            -6.05425658e-05, -7.40327129e-04, 1.00000000e+00;
  //     M = M.transpose();
  // }
  // Flatten x_grid and y_grid
  auto xgridRow =
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(
          x_grid);
  auto ygridRow =
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(
          y_grid);
  Eigen::VectorXd x_flat =
      Eigen::Map<Eigen::VectorXd>(xgridRow.data(), x_grid.size());
  Eigen::VectorXd y_flat =
      Eigen::Map<Eigen::VectorXd>(ygridRow.data(), y_grid.size());
  // 构造齐次坐标点
  Eigen::MatrixXd points(3, x_flat.size());
  points.row(0) = x_flat.transpose();
  points.row(1) = y_flat.transpose();
  points.row(2) = Eigen::VectorXd::Ones(x_flat.size()).transpose();
  // 应用透视变换
  Eigen::MatrixXd transformed_points = M * points;
  // 齐次坐标归一化
  transformed_points.row(0).array() /= transformed_points.row(2).array();
  transformed_points.row(1).array() /= transformed_points.row(2).array();
  transformed_points.row(2).array() /= transformed_points.row(2).array();
  // 重新 reshape 为 H x W，并减去中心偏移
  // auto x3d = Eigen::Map<Eigen::MatrixXd,
  // Eigen::RowMajor>(transformed_points.row(0).data(), H, W).array() - W / 2.0;
  // auto y3d = Eigen::Map<Eigen::MatrixXd,
  // Eigen::RowMajor>(transformed_points.row(1).data(), H, W).array() - H / 2.0;
  Eigen::VectorXd x_vec = transformed_points.row(0).transpose();
  Eigen::VectorXd y_vec = transformed_points.row(1).transpose();
  // reshape 成 H x W
  Eigen::MatrixXd x_reshaped =
      Eigen::Map<Eigen::MatrixXd>(x_vec.data(), W, H).transpose();
  Eigen::MatrixXd y_reshaped =
      Eigen::Map<Eigen::MatrixXd>(y_vec.data(), W, H).transpose();
  // 减去中心偏移
  x_grid = x_reshaped.array() - W / 2.0;
  y_grid = y_reshaped.array() - H / 2.0;
  // Eigen::MatrixXd z_grid = Eigen::MatrixXd::Constant(H, W, z);
  Eigen::MatrixXd pointsInCam(3, H * W);
  auto x3d_row =
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(
          x_grid);
  auto y3d_row =
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(
          y_grid);
  pointsInCam.row(0) =
      Eigen::Map<Eigen::RowVectorXd>(x3d_row.data(), x3d_row.size());
  pointsInCam.row(1) =
      Eigen::Map<Eigen::RowVectorXd>(y3d_row.data(), y3d_row.size());
  Eigen::VectorXd z3d_row(H * W);
  z3d_row.setConstant(z);
  pointsInCam.row(2) = z3d_row.transpose();
  Eigen::VectorXd mapx_flat(H * W);
  Eigen::VectorXd mapy_flat(H * W);
  for (int j = 0; j < pointsInCam.cols(); j++) {
    Eigen::Vector3d pointInCam = pointsInCam.block<3, 1>(0, j);
    pointInCam.normalize();
    Eigen::Vector2d pointInPixel = this->Project(pointInCam);
    mapx_flat(j) = pointInPixel(0);
    mapy_flat(j) = pointInPixel(1);
  }
  // assert(mapx_flat.size() == H * W && "Input vector size doesn't match image
  // dimensions");
  Eigen::MatrixXd mapx_e = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      mapx_flat.data(), H, W);
  Eigen::MatrixXd mapy_e = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      mapy_flat.data(), H, W);
  cv::eigen2cv(mapx_e, mapx);
  mapx.convertTo(mapx, CV_32FC1);
  cv::eigen2cv(mapy_e, mapy);
  mapy.convertTo(mapy, CV_32FC1);
}

bool CameraModelScara::set_params(const std::string &yamlFile) {
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
  cv::FileStorage fs(dataString,
                     cv::FileStorage::READ | cv::FileStorage::MEMORY);

  cv::FileNode n = fs["image size"];
  n["width"] >> image_width_;
  n["height"] >> image_height_;

  n = fs["distortion center"];
  n["cx"] >> center_[1];
  n["cy"] >> center_[0];

  n = fs["affine coefficients"];
  n["c"] >> affine_[0];
  n["d"] >> affine_[1];
  n["e"] >> affine_[2];

  n = fs["camera2world"];
  // n["length_pol"] >> length_pol;

  cv::FileNode pol_node = n["pol"];
  int i = 0;
  for (cv::FileNodeIterator it = pol_node.begin(); it != pol_node.end(); ++it) {
    double val;
    *it >> val;
    cam2world_[static_cast<uint32_t>(i)] = val;
    i++;
  }

  n = fs["world2camera"];
  // n["length_invpol"] >> length_invpol;

  pol_node = n["invpol"];
  i = 0;
  for (cv::FileNodeIterator it = pol_node.begin(); it != pol_node.end(); ++it) {
    double val;
    *it >> val;
    world2cam_[static_cast<uint32_t>(i)] = val;
    i++;
  }
  return true;
}

// bool CameraModelScara::set_params(size_t width, size_t height,
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
