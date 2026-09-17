/******************************************************************************
 * Copyright 2024 ZF. All Rights Reserved.
 * Author:
 *        xuheng.chai@zf.com
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
/**
 * @file
 * @brief Image converter ros node, include save or show image.
 * @todo Image undistort
 * @return
 */

#ifndef ZF_DETECT_EGO_ANGLE_H
#define ZF_DETECT_EGO_ANGLE_H

// #include <eigen3/Eigen/Core>
// #include <eigen3/Eigen/Dense>

#include "memory.h"
#include "opencv2/opencv.hpp"
#include "zf_cam_model/zf_cam_model_truck.h"
#include "zf_global/in/zf_detect_global.h"

BEGIN_NS_ZF_DETECTION

struct SPropEgoAngle {
  float angle = 0.0f;
  float confidence = 0.0f;
  std::vector<uint32_t> u;
  std::vector<uint32_t> v;
};

// class CameraModelTrcuk;

class DetcetEgoAngle {
 public:
  DetcetEgoAngle();
  ~DetcetEgoAngle();
  SPropEgoAngle CalAngle(const cv::Mat& image, float fusedAngle);
  void Reset();
  bool Enabled() const { return m_bInitGradient0; };
  SPropEgoAngle GetEgoProp() const { return m_curProps; };

 private:
  void FindNearGradientPixel();
  void UpdatePropUV();
  void UpdatePropUV(SPropEgoAngle& prop, Eigen::Matrix2Xi& pixel);
  Eigen::VectorXf CalRoiGradient(const Eigen::Vector2i& pixel, int size = 5);
  float CalRoiGradientF(const Eigen::Vector2i& pixel, int size = 5);
  void CalAngleDict(int num = 10);

 private:
  std::vector<float> m_vecR;
  std::vector<float> m_vecH;
  float m_fChangeThreshold;
  float m_fMinMaxThreshold[2];
  std::shared_ptr<CameraModelTrcuk> m_pTruckCam;
  Eigen::Matrix2Xi m_pixelsCur;
  cv::Mat m_imgCur;
  cv::Mat m_imgCurGray;
  SPropEgoAngle m_curProps;
  SPropEgoAngle m_PropsDeg0;
  Eigen::VectorXf m_gradientWeight;
  Eigen::VectorXf m_gradientZero;
  bool m_bInitGradient0;

 private:
  std::unordered_map<float, Eigen::Matrix2Xi> m_mapAnglePixels;
};

END_NS_ZF_DETECTION

#endif  // ZF_DETECT_TRACKING_NODE_H