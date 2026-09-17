#ifndef ZF_BSD_STRUCT_H
#define ZF_BSD_STRUCT_H
#include <QString>

#include "zf_global/in/zf_ui_global.h"
#include <opencv2/imgproc/imgproc.hpp>

BEGIN_NS_ZF_UI

struct FusionObjListItem {
    QString type;
    int bsdlevel;
    int icastatus;
    double posx;
    double posy;
    double width;
    double length;
    int heading;
    double velx;
    double vely;
    QString name;
    int itype;
};

const cv::Scalar CV_COLOR_RED(255, 0, 0);   // Red color
const cv::Scalar CV_COLOR_BLUE(0, 0, 255);  // Blue color
const cv::Scalar CV_COLOR_GREEN(0, 255, 0);
const cv::Scalar CV_COLOR_YELLOW(255, 255, 0);
const cv::Scalar CV_COLOR_DEEPPINK(147, 20, 255);
const cv::Scalar CV_COLOR_ORANGERED(255, 69, 0);

END_NS_ZF_UI
#endif // ZF_BSD_STRUCT_H
