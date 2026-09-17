#include "zf_cam_model/zf_cam_model_truck.h"
#include "zf_cam_model/zf_cam_model_wide.h"

BEGIN_NS_ZF_DETECTION

template <typename T>
int Sign(T val) {
  return (T(0) < val) - (val < T(0));
}

double Rad2Deg(const double& rad) { return rad * 180 / M_PI; }
double Deg2Rad(const double& deg) { return deg * M_PI / 180; }

CameraModelTrcuk::CameraModelTrcuk(ECameraIntrExtr type) : m_camType(type) {
  std::string prefix = "/home/nvidia/params/"; 
  m_mapFileName = {
      {ECameraIntrExtr::LEFT_IN_VEHICLE,
       prefix + "extrinsic_left_front_fisheye_to_vehicle_ext.yaml"},
      {ECameraIntrExtr::LEFT_IN_FISHEYE,
       prefix + "extrinsic_vehicle_to_left_front_fisheye_ext.yaml"},
      {ECameraIntrExtr::RIGHT_IN_VEHICLE,
       prefix + "extrinsic_right_front_fisheye_to_vehicle_ext.yaml"},
      {ECameraIntrExtr::RIGHT_IN_FISHEYE,
       prefix + "extrinsic_vehicle_to_right_front_fisheye_ext.yaml"},
      {ECameraIntrExtr::WIDE_IN_VEHICLE,
       prefix + "extrinsic_front_wide_to_vehicle_ext.yaml"},
      {ECameraIntrExtr::VEHICLE_IN_WIDE,
       prefix + "extrinsic_vehicle_to_front_wide_ext.yaml"},
  };
  m_camTrans = std::make_unique<CoordinateTransform>();
  for (auto& yamls : m_mapFileName) {
    m_camTrans->ReadExtrinsicParam(static_cast<ECameraIntrExtr>(yamls.first),
                                   yamls.second);
  }
  m_mapFileName[ECameraIntrExtr::LEFT_INTR] =
      prefix + "intrinsic_left_front_fisheye_ocam.yaml";
  m_mapFileName[ECameraIntrExtr::RIGHT_INTR] =
      prefix + "intrinsic_right_front_fisheye_ocam.yaml";
  m_mapFileName[ECameraIntrExtr::WIDE_INTR] =
      prefix + "intrinsic_rear_wide_kbfisheye.yaml";
  
  switch (m_camType)
  {
  case WIDE:
    m_camModel = std::make_shared<CameraModelWide>();
    break;
  case LEFT:
  case RIGHT:
    m_camModel = std::make_shared<CameraModelScara>();
    break; 
  default:
    m_camModel = std::make_shared<CameraModelScara>();
    break;
  }

  // m_mapFileName[ECameraIntrExtr::RIGHT_TXT] = "params/calib_results.txt";
  m_camModel->set_params(m_mapFileName[m_camType]);
  // LOG_ERROR() << m_camModel->Project(Eigen::Vector3d{0.99743573, 0.0267914,
  // 0.06364}); // 1748 563
  this->SetPointZ(-0.7);
  this->SetCornerPixel0(Eigen::Vector2d(1780, 512));
}

void CameraModelTrcuk::GetMapXYPolyconic(cv::Mat &mapx, cv::Mat &mapy){
  m_camModel->undistort_polyconic(m_mapx, m_mapy);
  mapx = m_mapx;
  mapy = m_mapy;
}

void CameraModelTrcuk::GetMapXYPerspective(cv::Mat &mapx, cv::Mat &mapy){
  m_camModel->undistort_perspective(m_mapx, m_mapy, 15);
  mapx = m_mapx;
  mapy = m_mapy;
}

void CameraModelTrcuk::GetMapXYBev(cv::Mat &mapx, cv::Mat &mapy){
  double m[] = {3.25358679e-01, -6.89051449e-01, 5.93062013e+02, -5.87948257e-02,
      -2.84274759e-01, 4.88433284e+02, -6.05425658e-05, -7.40327129e-04,
      1.00000000e+00};
  m_camModel->undistort_bev(m_mapx, m_mapy, m ,10);
  mapx = m_mapx;
  mapy = m_mapy;
}


cv::Point CameraModelTrcuk::UndistortPixel2Origin(const float &x, const float &y) {
  int tx = static_cast<int>(y);
  int ty = static_cast<int>(x);
  auto img_h = m_mapx.rows - 1; //1080
  auto img_w = m_mapx.cols - 1; // 1920

  // Ensure we don't go out of bounds
  tx = std::max(0, std::min(tx, m_mapx.rows - 1)); // 1080
  ty = std::max(0, std::min(ty, m_mapx.cols - 1)); // 1920
  cv::Point rsl;
  rsl.x = limit_number(m_mapx.at<float>(tx, ty), 0, img_w);
  rsl.y = limit_number(m_mapy.at<float>(tx, ty), 0, img_h);
  return rsl;
}

void CameraModelTrcuk::SetRadarHeightInGround(float h) {
  m_camTrans->SetRadarHeightInGround(h);
}

void CameraModelTrcuk::SetPointZ(float z, bool inWorld) {
  if (inWorld) {
    Eigen::Vector3d point =
        m_camTrans->GetP_ground2cam(Eigen::Vector3d{0, 0, z}, m_camType);
    point = m_camTrans->GetP_cam2rectifyCam(point, m_camType);
    z = point(2);
  }
  m_fPointZ = z;
}

void CameraModelTrcuk::SetCamType(ECameraIntrExtr camType) {
  m_camType = camType;
}

void CameraModelTrcuk::SetCornerPixel0(const Eigen::Vector2d& point2d) {
  Eigen::Vector3d pGround = Pixel2Ground(point2d);
  m_props0.pGround = pGround;
  m_props0.length = pGround.block<2, 1>(0, 0).norm();
  m_props0.height = pGround(2);
  m_props0.angle = atan2(pGround(1), pGround(0));
  m_props0.init = true;
}

float CameraModelTrcuk::CalEgoAngleFromPixel(const Eigen::Vector2d& point2d) {
  if (!m_props0.init) {
    LOG_ERROR() << "Not init pixel 0";
    return 0.0;
  }
  Eigen::Vector3d pGround = Pixel2Ground(point2d);
  float len = pGround.block<2, 1>(0, 0).norm();
  Eigen::Vector3d p0 = ModifyCornerPixel0(len);
  float angle = Rad2Deg(atan2(pGround(1), pGround(0)) - atan2(p0(1), p0(0)));
  return angle;
}

Eigen::Vector2d CameraModelTrcuk::CalPixelFromEgoAngle(float angle,
                                                       bool isDegree) {
  if (!m_props0.init) {
    LOG_ERROR() << "Not init pixel 0";
    return Eigen::Vector2d();
  }
  if (isDegree) {
    angle = Deg2Rad(angle);  // deg2rad
  }
  angle += m_props0.angle;
  float x = m_props0.length * cos(angle);
  float y = m_props0.length * sin(angle);
  Eigen::Vector3d pGround(x, y, m_props0.height);
  return Ground2Pixel(pGround);
}

Eigen::Matrix2Xd CameraModelTrcuk::CalLengthPixelsByAngle(
    float angle, std::vector<float> lens, bool isDegree, bool isAbsolute) {
  if (!m_props0.init) {
    LOG_ERROR() << "Not init pixel 0";
    return Eigen::Vector2d();
  }
  if (isDegree) {
    angle = Deg2Rad(angle);  // deg2rad
  }
  Eigen::Matrix4Xd ptsGround(4, lens.size());
  int i = 0;
  for (auto& len : lens) {
    if (!isAbsolute) {
      len += m_props0.length;
    }
    Eigen::Vector3d p0 = ModifyCornerPixel0(len);
    float angle_modify = angle + atan2(p0(1), p0(0));
    ptsGround(0, i) = len * cos(angle_modify);
    ptsGround(1, i) = len * sin(angle_modify);
    ptsGround(2, i) = m_props0.height;
    ptsGround(3, i) = 1;
    i++;
  }
  Eigen::Matrix4Xd pointsInCam =
      m_camTrans->GetP_ground2cam(ptsGround, m_camType);
  Eigen::Matrix2Xd ptsPixel(2, lens.size());
  for (int j = 0; j < pointsInCam.cols(); j++) {
    Eigen::Vector3d pointInCam = pointsInCam.block<3, 1>(0, j);
    pointInCam.normalize();
    Eigen::Vector2d pointInPixel = m_camModel->Project(pointInCam);
    ptsPixel(0, j) = pointInPixel(0);
    ptsPixel(1, j) = pointInPixel(1);
  }
  return ptsPixel;
}

Eigen::Matrix2Xd CameraModelTrcuk::CalHeightPixelsByAngle(
    float angle, std::vector<float> heights, bool isDegree, bool isAbsolute) {
  if (!m_props0.init) {
    LOG_ERROR() << "Not init pixel 0";
    return Eigen::Vector2d();
  }
  if (isDegree) {
    angle = Deg2Rad(angle);  // deg2rad
  }
  angle += m_props0.angle;
  double x = m_props0.length * cos(angle);
  double y = m_props0.length * sin(angle);
  Eigen::Matrix4Xd ptsGround(4, heights.size());
  int i = 0;
  for (auto& height : heights) {
    if (!isAbsolute) {
      height += m_props0.height;
    }
    ptsGround(0, i) = x;
    ptsGround(1, i) = y;
    ptsGround(2, i) = height;
    ptsGround(3, i) = 1;
    i++;
  }
  Eigen::Matrix4Xd pointsInCam =
      m_camTrans->GetP_ground2cam(ptsGround, m_camType);
  Eigen::Matrix2Xd ptsPixel(2, heights.size());
  for (int j = 0; j < pointsInCam.cols(); j++) {
    Eigen::Vector3d pointInCam = pointsInCam.block<3, 1>(0, j);
    pointInCam.normalize();
    Eigen::Vector2d pointInPixel = m_camModel->Project(pointInCam);
    ptsPixel(0, j) = pointInPixel(0);
    ptsPixel(1, j) = pointInPixel(1);
  }
  return ptsPixel;
}

// world2cam
Eigen::Vector2d CameraModelTrcuk::Ground2Pixel(const Eigen::Vector3d& point3d) {
  Eigen::Vector3d pointInCam = m_camTrans->GetP_ground2cam(point3d, m_camType);
  pointInCam.normalize();
  Eigen::Vector2d pointInPixel = m_camModel->Project(pointInCam);
  return pointInPixel;
}

// input Matrix3Xd
Eigen::Matrix2Xd CameraModelTrcuk::Ground2Pixel(
    const Eigen::MatrixXd& points3d) {
  Eigen::Matrix4Xd points4n(4, points3d.cols());
  points4n.block(0, 0, 3, points3d.cols()) = points3d;
  points4n.row(2).setConstant(0);
  points4n.row(3).setConstant(1);
  Eigen::Matrix4Xd pointsInCam =
      m_camTrans->GetP_ground2cam(points4n, m_camType);
  Eigen::Matrix2Xd ptsPixel(2, points3d.cols());
  for (int i = 0; i < points3d.cols(); ++i) {
    // Eigen::Vector3d pointInCam = pointsInCam.col(i).block<3, 1>(0, 0);
    // pointInCam.normalize();
    ptsPixel.col(i) =
        m_camModel->Project(pointsInCam.col(i).block<3, 1>(0, 0).normalized());
  }
  return ptsPixel;
}
Eigen::Vector2d CameraModelTrcuk::Radar2Pixel(const Eigen::Vector3d& point3d) {
  Eigen::Vector3d pointInCam = m_camTrans->GetP_radar2cam(point3d, m_camType);
  pointInCam.normalize();
  Eigen::Vector2d pointInPixel = m_camModel->Project(pointInCam);
  return pointInPixel;
}

Eigen::Vector3d CameraModelTrcuk::Pixel2Ground(const Eigen::Vector2d& point2d) {
  return Pixel2Ground(point2d, m_fPointZ);
}

// cam2world
Eigen::Vector3d CameraModelTrcuk::Pixel2Ground(const Eigen::Vector2d& point2d,
                                               float pointZ) {
  Eigen::Vector3d pointInCam = m_camModel->UnProject(point2d);
  Eigen::Vector3d pointInRectifyCam =
      m_camTrans->GetP_cam2rectifyCam(pointInCam, m_camType);
  double norm_len = pointZ / pointInRectifyCam(2);
  pointInRectifyCam = norm_len * pointInRectifyCam;
  Eigen::Vector3d pointInGround =
      m_camTrans->GetP_rectifyCam2ground(pointInRectifyCam, m_camType);
  return pointInGround;
}

Eigen::Vector3d CameraModelTrcuk::ModifyCornerPixel0(const float& length) {
  Eigen::Vector3d pGround = m_props0.pGround;

  if (length <= abs(pGround(1))) {
    return pGround;
  }
  // # cal y coord based on len and x
  pGround(0) =
      Sign(pGround(0)) * sqrt(length * length - pGround(1) * pGround(1));
  return pGround;
}

END_NS_ZF_DETECTION
