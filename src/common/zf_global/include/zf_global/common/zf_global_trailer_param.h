#ifndef ZF_TRAILER_PARAM_H
#define ZF_TRAILER_PARAM_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include "zf_global/common/zf_global_macros.h"
#include "zf_global/util/string_util.h"

BEGIN_NS_ZF

class GlobalParams {
 public:
  // using MapContainer = std::unordered_map<uint64_t, std::string>;
  typedef Eigen::Matrix<double, 3, 4> Matrix34d;
  static GlobalParams &Instance() {
    static GlobalParams ins;
    return ins;
  }
  ~GlobalParams() {};
  /*
  rect :  3X4
          1.27 -11.73
          1.339 -1.339 
            1      1
  result: 3rows 4xnumber cols
          1.27   -1.98   -5.23   -8.48  -11.73 
          1.339  0.6695       0 -0.6695  -1.339  
          1       1       1       1       1   
  */
  Eigen::MatrixXd GetLinSpacePoints(Eigen::MatrixXd rect, int number) {
    Eigen::MatrixXd result(rect.rows(), number * 4);
    Eigen::MatrixXd rect_trans = rect.transpose();
    for (int i = 0; i < rect.rows(); ++i) {
      Eigen::VectorXd linspace1 =
          Eigen::VectorXd::LinSpaced(number, rect.row(i)(0), rect.row(i)(1));
      Eigen::VectorXd linspace2 =
          Eigen::VectorXd::LinSpaced(number, rect.row(i)(1), rect.row(i)(2));
      Eigen::VectorXd linspace3 =
          Eigen::VectorXd::LinSpaced(number, rect.row(i)(2), rect.row(i)(3));
      Eigen::VectorXd linspace4 =
          Eigen::VectorXd::LinSpaced(number, rect.row(i)(3), rect.row(i)(0));
      result.block(i, 0, 1, number) = linspace1.transpose();
      result.block(i, number, 1, number) = linspace2.transpose();
      result.block(i, 2 * number, 1, number) = linspace3.transpose();
      result.block(i, 3 * number, 1, number) = linspace4.transpose();
    }
    return result;
  }

  Matrix34d GetCornersTrailer() { return m_trailerCorners; }
  Matrix34d GetTrailerOffestHigh() { return m_trailerOffestHigh; }
  Matrix34d GetTrailerOffestMid() { return m_trailerOffestMid; }
  Matrix34d GetTrailerOffestLow() { return m_trailerOffestLow; }
  Matrix34d GetCornersHead() { return m_headCorners; }
  Matrix34d GetHeadOffestHigh() { return m_headOffestHigh; }
  Matrix34d GetHeadOffestMid() { return m_headOffestMid; }
  Matrix34d GetHeadOffestLow() { return m_headOffestLow; }
  // uint64_t GenerateHashId(const std::string &name) {
  //   return NS_ZF_STRING_UTIL::Hash(name);
  // }
 private:
  Matrix34d m_trailerCorners;
  Matrix34d m_trailerOffestHigh;
  Matrix34d m_trailerOffestMid;
  Matrix34d m_trailerOffestLow;
  Matrix34d m_headCorners;
  Matrix34d m_headOffestHigh;
  Matrix34d m_headOffestMid;
  Matrix34d m_headOffestLow;
  float m_hml[3];
  void Init() {
    m_hml[0] = 1.5;  // high
    m_hml[1] = 4.5;  // mid
    m_hml[2] = 6.0;  // low
    /*
    (p3)|-------------|(p4) left          y
        |             |                 |
        |          o  |                 |------------ x
        |             |
    (p2)|-------------|(p1) right
    */
    // m_trailerCorners << 1.27, -11.73, -11.73, 1.27, -1.339, -1.339, 1.337,
    //     1.337, 1, 1, 1, 1;
    m_trailerCorners.row(0) << 1.27, -11.73, -11.73, 1.27;
    m_trailerCorners.row(1) << -1.339, -1.339, 1.337, 1.337;
    m_trailerCorners.row(2) << 1, 1, 1, 1;
    /* 7.18 -1.81 1.0
    (p3)|-------------|(p4) left          y
        |             |                 |
        |   o         |                 |------------ x
        |             |
    (p2)|-------------|(p1) right
    */
    m_headCorners.row(0) << 5.37, -1.81, -1.81, 5.37;
    m_headCorners.row(1) << -1.339, -1.339, 1.337, 1.337;
    m_headCorners.row(2) << 1, 1, 1, 1;
    /*
      [0,   -1.5, -1.5, 0],
      [-1.5, -1.5, -1.5,  0],
      [-1, -1, -1, -1],
     */
    m_trailerOffestHigh.row(0) << 0, -m_hml[0], -m_hml[0], 0;
    m_trailerOffestHigh.row(1) << -m_hml[0], -m_hml[0], -m_hml[0], 0;
    m_trailerOffestHigh.row(2) << -1, -1, -1, -1;
    /*
      [0,   -4.5, -4.5, 0],
      [-4.5, -4.5, -4.5,  0],
      [-1, -1, -1, -1],
     */
    m_trailerOffestMid.row(0) << 0, -m_hml[1], -m_hml[1], 0;
    m_trailerOffestMid.row(1) << -m_hml[1], -m_hml[1], -m_hml[1], 0;
    m_trailerOffestMid.row(2) << -1, -1, -1, -1;
    /*
      [0,  -6, -6, 0],
      [-6, -6, -6,  0],
      [-1, -1, -1, -1],
     */
    m_trailerOffestLow.row(0) << 0, -m_hml[2], -m_hml[2], 0;
    m_trailerOffestLow.row(1) << -m_hml[2], -m_hml[2], -m_hml[2], 0;
    m_trailerOffestLow.row(2) << -1, -1, -1, -1;
    /*
      [1.5,   0, 0, 1.5],
      [-1.5, -1.5, -1.5,  0],
      [-1, -1, -1, -1],
     */
    m_headOffestHigh.row(0) << m_hml[0], 0, 0, m_hml[0];
    m_headOffestHigh.row(1) << -m_hml[0], -m_hml[0], -m_hml[0], 0;
    m_headOffestHigh.row(2) << -1, -1, -1, -1;
    /*
      [4.5,   0, 0, 4.5],
      [-4.5, -4.5, -4.5,  0],
      [-1, -1, -1, -1],
     */
    m_headOffestMid.row(0) << m_hml[1], 0, 0, m_hml[1];
    m_headOffestMid.row(1) << -m_hml[1], -m_hml[1], -m_hml[1], 0;
    m_headOffestMid.row(2) << -1, -1, -1, -1;
    /*
      [6, 0, 0, 6],
      [-6, -6, -6, 0],
      [-1, -1, -1, -1],
     */
    m_headOffestLow.row(0) << m_hml[2], 0, 0, m_hml[2];
    m_headOffestLow.row(1) << -m_hml[2], -m_hml[2], -m_hml[2], 0;
    m_headOffestLow.row(2) << -1, -1, -1, -1;
  }

  GlobalParams() { Init(); };
  DISALLOW_COPY_AND_ASSIGN(GlobalParams)
};

END_NS_ZF

/* Uint test
  Eigen::MatrixXd rect(3, 4);
  rect.row(0) << 1.27, -11.73, -11.73, 1.27;
  rect.row(1) << 1.339, -1.339, 1.337, 1.337;
  rect.row(2) << 1, 1, 1, 1;
  LOG_DEBUG() << rect;

  auto ttt = GlobalParams::Instance().GetLinSpacePoints(rect, 5);
  LOG_INFO() << ttt;
*/

#endif  // !ZF_GLOBAL_COMMON_MACROS_H
