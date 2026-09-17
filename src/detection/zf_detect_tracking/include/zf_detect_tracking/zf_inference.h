#ifndef ZF_JETSON_INFERENCE_H
#define ZF_JETSON_INFERENCE_H

// #include "zf_global/in/zf_detect_global.h"
#include "zf_detect_tracking/zf_infer_struct.h"
#include "opencv2/opencv.hpp"

BEGIN_NS_ZF_DETECTION

class Inference
{
public:
    Inference() = default;

    virtual ~Inference() = default;

    virtual bool Init(const std::string & engine_file_path, bool warmup = true) = 0;

    virtual void Infer(const cv::Mat& image, std::vector<DetectObject>& objs) = 0;
    virtual void SetMapXY(const cv::Mat& mx, const cv::Mat& my) = 0;
    bool GetUndistortFlag() {return m_bUsingUndistort;};
    bool m_bUsingUndistort = {};

//     void set_max_batch_size(const int &batch_size);

//     void set_gpu_id(const int &gpu_id);

// protected:
//     int max_batch_size_ = 1;
//     int gpu_id_ = 0;
};

END_NS_ZF_DETECTION // namespace det

#endif // ZF_JETSON_INFERENCE_H