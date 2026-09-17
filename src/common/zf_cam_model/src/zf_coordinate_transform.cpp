#include "zf_cam_model/zf_coordinate_transform.h"

#include <fstream>

BEGIN_NS_ZF_DETECTION

CoordinateTransform::CoordinateTransform(/* args */) {
  m_fFrontTireX = 3.830;          // unit: m
  SetRadarHeightInGround(0.505);  // radar height
}

CoordinateTransform::~CoordinateTransform() {}

Eigen::Matrix4d CoordinateTransform::InverseMatrix(const Eigen::Matrix4d& T) {
  Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();
  trans.block<3, 3>(0, 0) = T.block<3, 3>(0, 0).transpose();
  trans.block<3, 1>(0, 3) = -1 * trans.block<3, 3>(0, 0) * T.block<3, 1>(0, 3);
  return trans;
}

void CoordinateTransform::SetRadarHeightInGround(const float& h) {
  m_inGround_T_radar = Eigen::Matrix4d::Identity();
  m_inGround_T_radar.block<3, 1>(0, 3) = Eigen::Vector3d{m_fFrontTireX, 0, h};
}

Eigen::Vector3d CoordinateTransform::GetP_cam2rectifyCam(
    const Eigen::Vector3d& point, ECameraIntrExtr camera) {
  Eigen::Vector3d point31(point[0], point[1], point[2]);
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  switch (camera) {
    case ECameraIntrExtr::LEFT:
    case ECameraIntrExtr::LEFT_IN_VEHICLE:
    case ECameraIntrExtr::LEFT_IN_FISHEYE: {
      R = m_mapCamExtrT[ECameraIntrExtr::LEFT_IN_VEHICLE].block<3, 3>(0, 0);
      break;
    }
    case ECameraIntrExtr::RIGHT:
    case ECameraIntrExtr::RIGHT_IN_VEHICLE:
    case ECameraIntrExtr::RIGHT_IN_FISHEYE: {
      R = m_mapCamExtrT[ECameraIntrExtr::RIGHT_IN_VEHICLE].block<3, 3>(0, 0);
      break;
    }
    default:
      break;
  }
  // std::cout << "R \n" << R << std::endl;
  Eigen::Vector3d rectified_point = R * point31;
  return rectified_point;
}

Eigen::Vector3d CoordinateTransform::GetP_rectifyCam2ground(
    const Eigen::Vector3d& point, ECameraIntrExtr camera) {
  Eigen::Vector4d point41;
  point41 << point(0), point(1), point(2), 1;
  Eigen::Matrix4d inGround_T_rectifyCam;
  switch (camera) {
    case ECameraIntrExtr::LEFT:
    case ECameraIntrExtr::LEFT_IN_VEHICLE:
    case ECameraIntrExtr::LEFT_IN_FISHEYE: {
      inGround_T_rectifyCam = m_mapCamExtrT[ECameraIntrExtr::LEFT_IN_VEHICLE];
      break;
    }
    case ECameraIntrExtr::RIGHT:
    case ECameraIntrExtr::RIGHT_IN_VEHICLE:
    case ECameraIntrExtr::RIGHT_IN_FISHEYE: {
      inGround_T_rectifyCam = m_mapCamExtrT[ECameraIntrExtr::RIGHT_IN_VEHICLE];
      break;
    }
    default:
      break;
  }
  inGround_T_rectifyCam.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  Eigen::Vector4d point_in_ground = inGround_T_rectifyCam * point41;
  return point_in_ground.block<3, 1>(0, 0);
}

Eigen::Vector3d CoordinateTransform::GetP_cam2ground(
    const Eigen::Vector3d& point, ECameraIntrExtr camera) {
  Eigen::Vector4d point41;
  point41 << point(0), point(1), point(2), 1;
  Eigen::Vector4d pointGround;
  switch (camera) {
    case ECameraIntrExtr::LEFT:
    case ECameraIntrExtr::LEFT_IN_VEHICLE:
    case ECameraIntrExtr::LEFT_IN_FISHEYE: {
      pointGround = m_mapCamExtrT[ECameraIntrExtr::LEFT_IN_VEHICLE] * point41;
      break;
    }
    case ECameraIntrExtr::RIGHT:
    case ECameraIntrExtr::RIGHT_IN_VEHICLE:
    case ECameraIntrExtr::RIGHT_IN_FISHEYE: {
      pointGround = m_mapCamExtrT[ECameraIntrExtr::RIGHT_IN_VEHICLE] * point41;
      break;
    }
    default:
      break;
  }
  return pointGround.block<3, 1>(0, 0);
  ;
}

Eigen::Vector3d CoordinateTransform::GetP_ground2cam(
    const Eigen::Vector3d& point, ECameraIntrExtr camera) {
  Eigen::Vector4d point41;
  point41 << point(0), point(1), point(2), 1;
  Eigen::Vector4d pointCam;
  switch (camera) {
    case ECameraIntrExtr::LEFT:
    case ECameraIntrExtr::LEFT_IN_VEHICLE:
    case ECameraIntrExtr::LEFT_IN_FISHEYE: {
      pointCam = m_mapCamExtrT[ECameraIntrExtr::LEFT_IN_FISHEYE] * point41;
      break;
    }
    case ECameraIntrExtr::RIGHT:
    case ECameraIntrExtr::RIGHT_IN_VEHICLE:
    case ECameraIntrExtr::RIGHT_IN_FISHEYE: {
      pointCam = m_mapCamExtrT[ECameraIntrExtr::RIGHT_IN_FISHEYE] * point41;
      break;
    }
    default:
      break;
  }
  return pointCam.block<3, 1>(0, 0);
}

Eigen::Matrix4Xd CoordinateTransform::GetP_ground2cam(
    const Eigen::Matrix4Xd& points, ECameraIntrExtr camera) {
  Eigen::Matrix4Xd pointsCam;
  switch (camera) {
    case ECameraIntrExtr::LEFT:
    case ECameraIntrExtr::LEFT_IN_VEHICLE:
    case ECameraIntrExtr::LEFT_IN_FISHEYE: {
      pointsCam = m_mapCamExtrT[ECameraIntrExtr::LEFT_IN_FISHEYE] * points;
      break;
    }
    case ECameraIntrExtr::RIGHT:
    case ECameraIntrExtr::RIGHT_IN_VEHICLE:
    case ECameraIntrExtr::RIGHT_IN_FISHEYE: {
      pointsCam = m_mapCamExtrT[ECameraIntrExtr::RIGHT_IN_FISHEYE] * points;
      break;
    }
    default:
      break;
  }
  return pointsCam;
}

Eigen::Vector3d CoordinateTransform::GetP_radar2cam(
    const Eigen::Vector3d& point, ECameraIntrExtr camera) {
  Eigen::Vector4d point41;
  point41 << point(0), point(1), point(2), 1;
  Eigen::Vector4d pointCam;
  switch (camera) {
    case ECameraIntrExtr::LEFT:
    case ECameraIntrExtr::LEFT_IN_VEHICLE:
    case ECameraIntrExtr::LEFT_IN_FISHEYE: {
      pointCam = m_mapCamExtrT[ECameraIntrExtr::LEFT_IN_FISHEYE] *
                 m_inGround_T_radar * point41;
      break;
    }
    case ECameraIntrExtr::RIGHT:
    case ECameraIntrExtr::RIGHT_IN_VEHICLE:
    case ECameraIntrExtr::RIGHT_IN_FISHEYE: {
      pointCam = m_mapCamExtrT[ECameraIntrExtr::RIGHT_IN_FISHEYE] *
                 m_inGround_T_radar * point41;
      break;
    }
    default:
      break;
  }
  return pointCam.block<3, 1>(0, 0);
}

Eigen::Vector3d CoordinateTransform::GetP_radar2ground(
    const Eigen::Vector3d& point) {
  Eigen::Vector4d point41;
  point41 << point(0), point(1), point(2), 1;
  Eigen::Vector4d pointGround = m_inGround_T_radar * point41;
  return pointGround.block<3, 1>(0, 0);
}
Eigen::Vector3d CoordinateTransform::GetP_ground2radar(
    const Eigen::Vector3d& point) {
  Eigen::Vector4d point41;
  point41 << point(0), point(1), point(2), 1;
  Eigen::Vector4d pointGround = InverseMatrix(m_inGround_T_radar) * point41;
  return pointGround.block<3, 1>(0, 0);
}
void CoordinateTransform::ReadExtrinsicParam(ECameraIntrExtr type,
                                             const std::string& filename) {
  std::ifstream file(filename);
  std::vector<float> data;
  std::string line;
  while (std::getline(file, line)) {
    // std::string key;
    double value;
    if (line.find(":") != std::string::npos) {
      size_t pos = line.find(":");
      //   key = line.substr(0, pos);
      value = std::stod(line.substr(pos + 1));
      data.push_back(value);
    }
  }
  SCamExtrParam param(data);
  m_mapCamExtrParam[type] = param;
  // 如果我们人为定义的顺序为wxyz，则使用直接法，Eigen输出的结果为xyzw
  // 如果我们人为定义的顺序为xyzw
  // ，则使用Vector4d赋值或数组赋值，Eigen输出的结果为xyzw
  Eigen::Quaterniond quaterniond(param.qw, param.qx, param.qy, param.qz);
  Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();
  trans.block<3, 3>(0, 0) = quaterniond.matrix();
  trans.block<3, 1>(0, 3) = Eigen::Vector3d{param.x, param.y, param.z};
  LOG_DEBUG() << "type: " << type << ", " << trans;
  m_mapCamExtrT[type] = trans;
  file.close();
}

END_NS_ZF_DETECTION