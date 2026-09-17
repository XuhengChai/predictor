#ifndef ZF_CAM_MODEL_TRUCK_H
#define ZF_CAM_MODEL_TRUCK_H

#include <memory>
#include <opencv2/opencv.hpp>

#include "zf_cam_model/zf_cam_model_scara.h"
#include "zf_cam_model/zf_coordinate_transform.h"

BEGIN_NS_ZF_DETECTION

class CameraModelTrcuk {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  struct ZeroProps {
    Eigen::Vector3d pGround;
    float length = 0.0;
    float height = 0.0;
    float angle = 0.0;
    bool init = false;
  };
  CameraModelTrcuk(ECameraIntrExtr type = ECameraIntrExtr::RIGHT);
  ~CameraModelTrcuk() = default;
  void GetMapXYPolyconic(cv::Mat& mapx, cv::Mat& mapy);
  void GetMapXYPerspective(cv::Mat& mapx, cv::Mat& mapy);
  void GetMapXYBev(cv::Mat& mapx, cv::Mat& mapy);
  // x, y is the start piexls to put text
  static void DrawTextLines(const cv::Mat& img,
                            std::vector<cv::String>& textLines, int x = 100,
                            int y = 100) {
    float fontScale = 0.6;
    int thickness = 1;
    int fontFace = cv::LINE_AA;
    int baseline = 0;
    for (size_t i = 0; i < textLines.size(); i++) {
      std::string text = textLines[i];
      if (!text.empty()) {
        cv::Size textSize =
            cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);
        cv::Point drawPoint(x, y + (textSize.height + 10 + baseline) * i);
        cv::putText(img, text, drawPoint, cv::FONT_HERSHEY_DUPLEX, fontScale,
                    cv::Scalar(0, 0, 255), thickness, fontFace);
      }
    }
  };
  float limit_number(float num, float minv, float maxv) {
    if (num < minv) {
      return minv;
    }
    if (num > maxv) {
      return maxv;
    }
    return num;
  }

  // Undistort pixel to origin function
  cv::Point UndistortPixel2Origin(const float &x, const float &y);
  void SetRadarHeightInGround(float h);
  void SetPointZ(float z, bool inWorld = false);
  void SetCamType(ECameraIntrExtr camType);
  void SetCornerPixel0(const Eigen::Vector2d& point2d);
  float CalEgoAngleFromPixel(const Eigen::Vector2d& point2d);
  Eigen::Vector2d CalPixelFromEgoAngle(float angle, bool isDegree = true);
  Eigen::Matrix2Xd CalLengthPixelsByAngle(float angle, std::vector<float> lens,
                                          bool isDegree = true,
                                          bool isAbsolute = false);
  Eigen::Matrix2Xd CalHeightPixelsByAngle(float angle,
                                          std::vector<float> heights,
                                          bool isDegree = true,
                                          bool isAbsolute = false);
  Eigen::Vector2d Ground2Pixel(const Eigen::Vector3d& point3d);
  Eigen::Matrix2Xd Ground2Pixel(const Eigen::MatrixXd& points3d);
  Eigen::Vector2d Radar2Pixel(const Eigen::Vector3d& point3d);
  Eigen::Vector3d Pixel2Ground(const Eigen::Vector2d& point2d);
  Eigen::Vector3d Pixel2Ground(const Eigen::Vector2d& point2d, float pointZ);

 private:
  Eigen::Vector3d ModifyCornerPixel0(const float& len);
  std::shared_ptr<BaseCameraModel> m_camModel;
  std::unique_ptr<CoordinateTransform> m_camTrans;
  ECameraIntrExtr m_camType;
  std::unordered_map<uint32_t, std::string> m_mapFileName;
  float m_fPointZ;  //  point height in world frame / in camera frame
  ZeroProps m_props0;
  cv::Mat m_mapx;
  cv::Mat m_mapy;
};

END_NS_ZF_DETECTION
#endif  // ZF_CAM_MODEL_TRUCK_H
