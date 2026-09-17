#include <cstdio>
#include "zf_cam_model/zf_cam_model_truck.h"


int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  printf("hello world zf_cam_model package\n");

  // std::shared_ptr<NS_ZF_DETECTION::CameraModelTrcuk> m_pTruckCam(new NS_ZF_DETECTION::CameraModelTrcuk());
  NS_ZF_DETECTION::CameraModelTrcuk m_pTruckCam;
  std::vector<std::string> m_postfix;
  cv::Mat m_mapx;
  cv::Mat m_mapy;
  // m_pTruckCam.GetMapXYPerspective(m_mapx, m_mapy);
  m_pTruckCam.GetMapXYPolyconic(m_mapx, m_mapy);
  printf("hello world zf_cam_model package end\n");

  return 0;
}

