#include "zf_detect_tracking/zf_det_ego_angle.h"

// #include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

// #include "zf_cam_model/zf_cam_model_truck.h"
#include "zf_framework_transport/transport.h"
#include "zf_global/zf_global_topic_name.h"

// int main(int argc, char ** argv)
// {
//   (void) argc;
//   (void) argv;

//   printf("hello world zf_detect_tracking package\n");
//   return 0;
// }

BEGIN_NS_ZF_DETECTION

// float EcludSim(const Eigen::VectorXf& inA, const Eigen::VectorXf& inB) {
//   float diff_sum = 0;
//   int cnt = 0;
//   for (int i = 0; i < inA.size(); i++) {
//     if (inA[i] == 0 && inB[i] == 0) {
//       continue;
//     } else if (inA[i] == 0 || inB[i] == 0) {
//       diff_sum += 1;
//     } else {
//       diff_sum += std::pow((inA[i] / inB[i] + inB[i] / inA[i] - 2), 2);
//       cnt += 1;
//     }
//   }
//   return std::sqrt(diff_sum) / cnt;
// }

DetcetEgoAngle::DetcetEgoAngle() : m_pTruckCam(new CameraModelTrcuk) {
  m_vecR = std::vector<float>{0.0, -1.0, -2.0, -3.0, -4.0, -5.0};
  m_vecH = std::vector<float>{0.1, 0.2, 0.3, 0.4};
  m_fChangeThreshold = 1;
  m_fMinMaxThreshold[0] = 0.05;
  m_fMinMaxThreshold[1] = 0.5;
  m_pTruckCam->SetCamType(ECameraIntrExtr::RIGHT);
  m_pTruckCam->SetPointZ(-0.7);
  m_pTruckCam->SetCornerPixel0(Eigen::Vector2d{1780, 512});
  auto pixs = m_pTruckCam->CalLengthPixelsByAngle(0.0, m_vecR);
  m_pixelsCur = (pixs.array()+0.5).cast<int>();
  UpdatePropUV(m_PropsDeg0, m_pixelsCur);
  // LOG_DEBUG() << "m_pixelsCur " << m_pixelsCur;
  // init m_gradientWeight
  m_gradientWeight = Eigen::MatrixXf(m_vecR.size(), 1);
  m_gradientZero = Eigen::MatrixXf(m_vecR.size(), 1);
  for (int i = 0; i < m_vecR.size(); i++) {
    m_gradientWeight(i) = i + 1;
  }
  m_gradientWeight.normalize();
  m_gradientWeight(0) = m_gradientWeight(2);
  m_gradientWeight(2) = m_gradientWeight(3);
  Reset();
}

DetcetEgoAngle::~DetcetEgoAngle() {}

SPropEgoAngle DetcetEgoAngle::CalAngle(const cv::Mat& image, float fusedAngle) {
  m_imgCur = image;
  cv::cvtColor(m_imgCur, m_imgCurGray, cv::COLOR_BGR2GRAY);
  if (!m_bInitGradient0) {
    m_bInitGradient0 = true;
    for (size_t i = 0; i < m_curProps.u.size(); i++) {
      int u = m_curProps.u[i];
      int v = m_curProps.v[i];
      m_gradientZero(i) = CalRoiGradientF(Eigen::Vector2i(u, v));
    }
  }
  FindNearGradientPixel();
  auto tProp = m_curProps;
  if (m_curProps.confidence < 0.99) {
    m_curProps.angle = (fusedAngle + m_curProps.angle)/2;
    m_pixelsCur = m_pTruckCam->CalLengthPixelsByAngle(m_curProps.angle, m_vecR)
                      .cast<int>();
    UpdatePropUV(m_curProps, m_pixelsCur);
  }
  return tProp;
}

void DetcetEgoAngle::FindNearGradientPixel() {
  CalAngleDict();
  std::unordered_map<float, double> mapPixelMean;
  std::unordered_map<float, Eigen::VectorXf> mapPixelGrad;

  for (auto const& elem : m_mapAnglePixels) {
    float key = elem.first;
    Eigen::Matrix2Xi val = elem.second;
    Eigen::VectorXf gradientAllPixel(val.cols());
    for (int i = 0; i < val.cols(); i++) {
      gradientAllPixel(i) = CalRoiGradientF(val.col(i));
    }
    mapPixelGrad[key] = gradientAllPixel;

    float diff_sum = 0;
    int cnt = 0;
    for (int i = 0; i < gradientAllPixel.size(); i++) {
      bool isNear0 = abs(gradientAllPixel[i]) < 1e-10;
      bool isNear1 = abs(m_gradientZero[i]) < 1e-10;
      if (isNear0 && isNear1) {
        continue;
      } else if (isNear0 || isNear1) {
        diff_sum += m_gradientWeight(i);
        cnt += 1;
      } else {
        diff_sum +=
            m_gradientWeight(i) * (gradientAllPixel[i] / m_gradientZero[i] +
                                   m_gradientZero[i] / gradientAllPixel[i] - 2);
        cnt += 1;
      }
    }
    mapPixelMean[key] = diff_sum / cnt;
  }

  float max_key = std::min_element(mapPixelMean.begin(), mapPixelMean.end(),
                                   [](const auto& l, const auto& r) {
                                     return l.second < r.second;
                                   })
                      ->first;

  // std::cout << max_key << " " << mapPixelMean[max_key] << std::endl;
  m_curProps.confidence = 2.0 - 2.0 / (1.0 + exp(-mapPixelMean[max_key] * 0.5));

  // if (abs(max_key - m_curProps.angle) < 0.5 && mapPixelMean[max_key] < 0.1) {
  //   m_gradientZero = 0.25 * mapPixelGrad[max_key] + 0.75 * m_gradientZero;
  // }
  m_gradientZero = 0.25 * mapPixelGrad[max_key] + 0.75 * m_gradientZero;

  if (mapPixelMean[max_key] > 10) {
    m_fChangeThreshold += 0.05;
    if (m_fChangeThreshold > m_fMinMaxThreshold[1]) {
      m_fChangeThreshold = m_fMinMaxThreshold[1];
    }
    return;
  } else {
    if (abs(abs(max_key - m_curProps.angle) - m_fChangeThreshold) > 0.01) {
      m_fChangeThreshold -= 0.05;
    } else {
      m_fChangeThreshold += 0.05;
    }
    if (m_fChangeThreshold < m_fMinMaxThreshold[0]) {
      m_fChangeThreshold = m_fMinMaxThreshold[0];
    }
  }
  m_curProps.angle = max_key;
  UpdatePropUV(m_curProps, m_mapAnglePixels[max_key]);
}

void DetcetEgoAngle::Reset() {
  m_curProps = m_PropsDeg0;
  m_bInitGradient0 = false;
}

void DetcetEgoAngle::UpdatePropUV() { UpdatePropUV(m_curProps, m_pixelsCur); }

void DetcetEgoAngle::UpdatePropUV(SPropEgoAngle& prop,
                                  Eigen::Matrix2Xi& pixel) {
  Eigen::Matrix<uint32_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> mat_uint =
      pixel.cast<uint32_t>();
  // Create two std::vector<uint32_t> from the data of the casted matrix
  std::vector<uint32_t> vecU(mat_uint.data(),
                             mat_uint.data() + mat_uint.size() / 2);
  std::vector<uint32_t> vecV(mat_uint.data() + mat_uint.size() / 2,
                             mat_uint.data() + mat_uint.size());
  prop.u.swap(vecU);
  prop.v.swap(vecV);
}

void DetcetEgoAngle::CalAngleDict(int num) {
  m_mapAnglePixels.clear();
  float low = m_curProps.angle - m_fChangeThreshold;
  low = low < 0 ? 0 : low;
  double high = m_curProps.angle - m_fChangeThreshold;
  Eigen::VectorXf angles = Eigen::VectorXf::LinSpaced(num, low, high);
  Eigen::Matrix2Xi lenPixels;
  Eigen::Matrix2Xi heightPixels;
  for (int i = 0; i < angles.size(); ++i) {
    lenPixels = m_pTruckCam->CalLengthPixelsByAngle(0.0, m_vecR).cast<int>();
    heightPixels = m_pTruckCam->CalHeightPixelsByAngle(0.0, m_vecH).cast<int>();
    Eigen::Matrix2Xi concatenatedPixels(2,
                                        lenPixels.cols() + heightPixels.cols());
    concatenatedPixels << lenPixels, heightPixels;
    m_mapAnglePixels[angles(i)] = concatenatedPixels;
  }
}

Eigen::VectorXf DetcetEgoAngle::CalRoiGradient(const Eigen::Vector2i& pixel,
                                               int size) {
  int shift = 3;

  int x1 = pixel(0) - size + shift;
  int x2 = pixel(0) + size + shift;
  int y1 = pixel(1) - size + shift;
  int y2 = pixel(1) + size + shift;
  cv::Rect roiRect(x1, y1, x2 - x1, y2 - y1);
  cv::Mat src_gray, dst, dst_norm, dst_norm_scaled;
  src_gray = m_imgCurGray(roiRect);
  // Harris corner detect
  cv::cornerHarris(src_gray, dst, 2, 3, 0.04);
  // cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1);
  // cv::convertScaleAbs(dst_norm, dst_norm_scaled);
  // for (int j = 0; j < dst_norm.rows; j++) {
  //   for (int i = 0; i < dst_norm.cols; i++) {
  //     if ((int)dst_norm.at<float>(j, i) > 30) {
  //       dst.at<float>(j, i) = 0.0;
  //     }
  //   }
  // }
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> dstEigen;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> roiEigen;
  // cv::cv2eigen(dst, dstEigen);
  // cv::cv2eigen(src_gray, roiEigen);
  Eigen::VectorXf col_list = Eigen::VectorXf::Zero(dst.cols + src_gray.cols);
  // col_list << dstEigen.rowwise().maxCoeff(),
  //     (roiEigen.array().colwise().mean() * 0.1).matrix();
  Eigen::VectorXf row_list = Eigen::VectorXf::Zero(dst.rows + src_gray.rows);
  // row_list << dstEigen.colwise().maxCoeff() * 0.5,
  //     (roiEigen.array().rowwise().mean() * 0.1).matrix();
  // return (col_list.transpose() << row_list.transpose()).finished();
  return col_list;
}

float DetcetEgoAngle::CalRoiGradientF(const Eigen::Vector2i& pixel, int size) {
  int shift = 0;
  int x1 = pixel(0) - size + shift;
  int x2 = pixel(0) + size + shift;
  int y1 = pixel(1) - size + shift;
  int y2 = pixel(1) + size + shift;
  cv::Rect roiRect(x1, y1, x2 - x1, y2 - y1);
  cv::Mat src_gray, dst, dst_norm, dst_norm_scaled;
  src_gray = m_imgCurGray(roiRect);
  // Harris corner detect
  cv::cornerHarris(src_gray, dst, 2, 3, 0.04);
  double minVal, maxVal;  // 用于存放矩阵中的最大值和最小值
  cv::Point minIdx, maxIdx;  ////用于存放矩阵中的最大值和最小值在矩阵中的位置
  cv::minMaxLoc(src_gray, &minVal, &maxVal, &minIdx, &maxIdx);
  return 0.5 * maxVal + cv::mean(dst).val[0];
}

END_NS_ZF_DETECTION
