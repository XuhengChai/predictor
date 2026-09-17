#ifndef ZF_CAM_COORDINATE_TRANS_H
#define ZF_CAM_COORDINATE_TRANS_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <unordered_map>

#include "zf_global/in/zf_detect_global.h"
BEGIN_NS_ZF_DETECTION

enum ECameraIntrExtr : uint32_t {
  LEFT_IN_VEHICLE = 0,
  LEFT_IN_FISHEYE = 1,
  RIGHT_IN_VEHICLE = 2,
  RIGHT_IN_FISHEYE = 3,
  WIDE_IN_VEHICLE = RIGHT_IN_FISHEYE + 1,
  VEHICLE_IN_WIDE = WIDE_IN_VEHICLE + 1,
  LEFT_INTR = 6,
  RIGHT_INTR = LEFT_INTR + 1,
  WIDE_INTR = RIGHT_INTR + 1,
  LEFT = LEFT_INTR,
  RIGHT = RIGHT_INTR,
  WIDE = WIDE_INTR,
  RIGHT_TXT = WIDE_INTR + 1
};

struct SCamExtrParam {
  float x = 0.0;
  float y = 0.0;
  float z = 0.0;
  float qw = 0.0;
  float qx = 0.0;
  float qy = 0.0;
  float qz = 0.0;
  float yaw = 0.0;
  float pitch = 0.0;
  float roll = 0.0;
  SCamExtrParam(const std::vector<float>& data) {
    if (data.size() < 10) {
      return;
    }
    this->x = data[0];
    this->y = data[1];
    this->z = data[2];
    this->qw = data[3];
    this->qx = data[4];
    this->qy = data[5];
    this->qz = data[6];
    this->yaw = data[7];
    this->pitch = data[8];
    this->roll = data[9];
  }
  SCamExtrParam() = default;
  SCamExtrParam& operator=(const SCamExtrParam& other) {
    if (this != &other) {
      x = other.x;
      y = other.y;
      z = other.z;
      qw = other.qw;
      qx = other.qx;
      qy = other.qy;
      qz = other.qz;
      yaw = other.yaw;
      pitch = other.pitch;
      roll = other.roll;
    }
    return *this;
  }
};

class CoordinateTransform {
 private:
  /* data */
 public:
  CoordinateTransform(/* args */);
  ~CoordinateTransform();
  Eigen::Matrix4d InverseMatrix(const Eigen::Matrix4d& T);
  void SetRadarHeightInGround(const float& h);
  Eigen::Vector3d GetP_cam2rectifyCam(const Eigen::Vector3d& point,
                                      ECameraIntrExtr camera);

  Eigen::Vector3d GetP_rectifyCam2ground(const Eigen::Vector3d& point,
                                         ECameraIntrExtr camera);
  Eigen::Vector3d GetP_cam2ground(const Eigen::Vector3d& point,
                                  ECameraIntrExtr camera);
  Eigen::Vector3d GetP_ground2cam(const Eigen::Vector3d& point,
                                  ECameraIntrExtr camera);
  Eigen::Matrix4Xd GetP_ground2cam(const Eigen::Matrix4Xd& points,
                                   ECameraIntrExtr camera);
  Eigen::Vector3d GetP_radar2cam(const Eigen::Vector3d& point,
                                 ECameraIntrExtr camera);
  Eigen::Vector3d GetP_radar2ground(const Eigen::Vector3d& point);
  Eigen::Vector3d GetP_ground2radar(const Eigen::Vector3d& point);
  void ReadExtrinsicParam(ECameraIntrExtr type, const std::string& filename);

 private:
  std::unordered_map<uint32_t, SCamExtrParam> m_mapCamExtrParam;
  std::unordered_map<uint32_t, Eigen::Matrix4d> m_mapCamExtrT;
  float m_fFrontTireX;
  Eigen::Matrix4d m_inGround_T_radar;
};

END_NS_ZF_DETECTION
#endif  // ZF_CAM_COORDINATE_TRANS_H
