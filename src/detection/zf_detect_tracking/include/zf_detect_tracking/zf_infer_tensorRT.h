#ifndef ZF_JETSON_INFER_TENSORRT_H
#define ZF_JETSON_INFER_TENSORRT_H
#include "NvInferPlugin.h"
#include "zf_detect_tracking/zf_inference.h"
#include "fstream"
#include "opencv2/cvconfig.h"
#include "opencv2/core/cuda.hpp"

#ifdef HAVE_CUDA
    #include "opencv2/cudawarping.hpp"
#endif // #ifdef HAVE_CUDA



#define CHECK(call)                                                         \
    do                                                                      \
    {                                                                       \
        const cudaError_t error_code = call;                                \
        if (error_code != cudaSuccess)                                      \
        {                                                                   \
            printf("CUDA Error:\n");                                        \
            printf("    File:       %s\n", __FILE__);                       \
            printf("    Line:       %d\n", __LINE__);                       \
            printf("    Error code: %d\n", error_code);                     \
            printf("    Error text: %s\n", cudaGetErrorString(error_code)); \
            exit(1);                                                        \
        }                                                                   \
    } while (0)

BEGIN_NS_ZF_DETECTION

class Infer_YOLOv8 : public Inference
{
public:
    explicit Infer_YOLOv8();
    ~Infer_YOLOv8();

    virtual bool Init(const std::string &engine_file_path, bool warmup = true) override;
    virtual void Infer(const cv::Mat &image, std::vector<DetectObject> &objs) override;
    virtual void SetMapXY(const cv::Mat& mx, const cv::Mat& my) override;

    void MakePipe(bool warmup = true);
    void Preprocess(const cv::Mat &image);
    void Preprocess(const cv::Mat &image, cv::Size &size);
    void Letterbox(const cv::Mat &image, cv::Mat &out, cv::Size &size);
    // void Postprocess(std::vector<DetectObject> &objs);
    void Postprocess(std::vector<DetectObject> &objs,
                     float score_thres = 0.25f,
                     float iou_thres = 0.65f,
                     int topk = 100,
                     int num_labels = 80);
    void InferInternal();
    // static void          draw_objects(const cv::Mat&                                image,
    //                                   cv::Mat&                                      res,
    //                                   const std::vector<DetectObject>&                    objs,
    //                                   const std::vector<std::string>&               CLASS_NAMES,
    //                                   const std::vector<std::vector<unsigned int>>& COLORS);
    int num_bindings;
    int num_inputs = 0;
    int num_outputs = 0;
    std::vector<Binding> input_bindings;
    std::vector<Binding> output_bindings;
    std::vector<void *> host_ptrs;
    std::vector<void *> device_ptrs;

    PreParam pparam;

private:
    nvinfer1::ICudaEngine *engine = nullptr;
    nvinfer1::IRuntime *runtime = nullptr;
    nvinfer1::IExecutionContext *context = nullptr;
    cudaStream_t stream = nullptr;
    Logger gLogger{nvinfer1::ILogger::Severity::kERROR};
    #ifdef HAVE_CUDA
        cv::cuda::GpuMat m_mapx;
        cv::cuda::GpuMat m_mapy;
    #else
        cv::Mat m_mapx;
        cv::Mat m_mapy;
    #endif // #ifdef HAVE_CUDA
    int m_bCudaSupport;
};

Infer_YOLOv8::Infer_YOLOv8() : Inference()
{
    m_bCudaSupport = cv::cuda::getCudaEnabledDeviceCount();
    LOG_DEBUG() << "SUPPORT CUDA " << m_bCudaSupport;
}

Infer_YOLOv8::~Infer_YOLOv8()
{
    this->context->destroy();
    this->engine->destroy();
    this->runtime->destroy();
    cudaStreamDestroy(this->stream);
    for (auto &ptr : this->device_ptrs)
    {
        CHECK(cudaFree(ptr));
    }

    for (auto &ptr : this->host_ptrs)
    {
        CHECK(cudaFreeHost(ptr));
    }
}

bool Infer_YOLOv8::Init(const std::string &engine_file_path, bool warmup)
{
    std::ifstream file(engine_file_path, std::ios::binary);
    assert(file.good());
    file.seekg(0, std::ios::end);
    auto size = file.tellg();
    file.seekg(0, std::ios::beg);
    char *trtModelStream = new char[size];
    assert(trtModelStream);
    file.read(trtModelStream, size);
    file.close();
    initLibNvInferPlugins(&this->gLogger, "");
    this->runtime = nvinfer1::createInferRuntime(this->gLogger);
    assert(this->runtime != nullptr);

    this->engine = this->runtime->deserializeCudaEngine(trtModelStream, size);
    assert(this->engine != nullptr);
    delete[] trtModelStream;
    this->context = this->engine->createExecutionContext();

    assert(this->context != nullptr);
    cudaStreamCreate(&this->stream);
    this->num_bindings = this->engine->getNbBindings();

    for (int i = 0; i < this->num_bindings; ++i)
    {
        Binding binding;
        nvinfer1::Dims dims;
        nvinfer1::DataType dtype = this->engine->getBindingDataType(i);
        std::string name = this->engine->getBindingName(i);
        binding.name = name;
        binding.dsize = type_to_size(dtype);

        bool IsInput = engine->bindingIsInput(i);
        if (IsInput)
        {
            this->num_inputs += 1;
            dims = this->engine->getProfileDimensions(i, 0, nvinfer1::OptProfileSelector::kMAX);
            binding.size = get_size_by_dims(dims);
            binding.dims = dims;
            this->input_bindings.push_back(binding);
            // set max opt shape
            this->context->setBindingDimensions(i, dims);
        }
        else
        {
            dims = this->context->getBindingDimensions(i);
            binding.size = get_size_by_dims(dims);
            binding.dims = dims;
            this->output_bindings.push_back(binding);
            this->num_outputs += 1;
        }
    }
    this->MakePipe(warmup);
    return true;
}

void Infer_YOLOv8::Infer(const cv::Mat &image, std::vector<DetectObject> &objs)
{
    cv::Size ss(640, 640);
    this->Preprocess(image, ss);
    // auto end = std::chrono::system_clock::now();
    // auto tc = (double)std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.;
    // printf("1 Preprocess cost %2.4lf ms\n", tc);
    // auto start = std::chrono::system_clock::now();
    this->InferInternal();

    // end = std::chrono::system_clock::now();
    // tc = (double)std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.;
    // printf("2 InferInternal cost %2.4lf ms\n", tc);
    this->Postprocess(objs);
    // end = std::chrono::system_clock::now();
    // tc = (double)std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.;
    // printf("3 InferInternal cost %2.4lf ms\n", tc);
}

void Infer_YOLOv8::SetMapXY(const cv::Mat& mx, const cv::Mat& my)
{
    cv::Size ss(640, 360);
    #ifdef HAVE_CUDA
        cv::Mat mapx, mapy;
        cv::resize(mx, mapx, ss, 0, 0, cv::INTER_LINEAR);
        cv::resize(my, mapy, ss, 0, 0, cv::INTER_LINEAR);
        mapx /= (1920/640);
        mapy /= (1080/360);
        m_mapx.upload(mapx);
        m_mapy.upload(mapy);
    #else
        cv::resize(mx, m_mapx, ss, 0, 0, cv::INTER_LINEAR);
        cv::resize(my, m_mapy, ss, 0, 0, cv::INTER_LINEAR);
        m_mapx /= (1920/640);
        m_mapy /= (1080/360);
    #endif // #ifdef HAVE_CUDA
    m_bUsingUndistort = true;
}

void Infer_YOLOv8::MakePipe(bool warmup)
{
    for (auto &bindings : this->input_bindings)
    {
        void *d_ptr;
        CHECK(cudaMallocAsync(&d_ptr, bindings.size * bindings.dsize, this->stream));
        this->device_ptrs.push_back(d_ptr);
    }
    for (auto &bindings : this->output_bindings)
    {
        void *d_ptr, *h_ptr;
        size_t size = bindings.size * bindings.dsize;
        CHECK(cudaMallocAsync(&d_ptr, size, this->stream));
        CHECK(cudaHostAlloc(&h_ptr, size, 0));
        this->device_ptrs.push_back(d_ptr);
        this->host_ptrs.push_back(h_ptr);
    }
    if (warmup)
    {
        for (int i = 0; i < 10; i++)
        {
            for (auto &bindings : this->input_bindings)
            {
                size_t size = bindings.size * bindings.dsize;
                void *h_ptr = malloc(size);
                memset(h_ptr, 0, size);
                CHECK(cudaMemcpyAsync(this->device_ptrs[0], h_ptr, size, cudaMemcpyHostToDevice, this->stream));
                free(h_ptr);
            }
            this->InferInternal();
        }
        printf("model warmup 10 times\n");
    }
}

void Infer_YOLOv8::Letterbox(const cv::Mat &image, cv::Mat &out, cv::Size &size)
{
    const float inp_h = size.height;
    const float inp_w = size.width;
    float height = image.rows;
    float width = image.cols;

    float r = std::min(inp_h / height, inp_w / width);
    int padw = std::round(width * r);
    int padh = std::round(height * r);

    cv::Mat tmp;
    if ((int)width != padw || (int)height != padh)
    {
        cv::resize(image, tmp, cv::Size(padw, padh));
        // cv::imwrite("/home/nvidia/Pictures/camera/3.jpg", image, {cv::IMWRITE_JPEG_QUALITY, 50});
        if (m_bUsingUndistort)
        {
        #ifdef HAVE_CUDA
            if (m_bCudaSupport > 0)
            {
                cv::cuda::GpuMat dst, src;
                src.upload(tmp);
                cv::cuda::remap(src, dst, m_mapx, m_mapy, cv::INTER_LINEAR);
                dst.download(tmp);
            }
        #else
            cv::remap(tmp, tmp, m_mapx, m_mapy, cv::INTER_LINEAR);
        #endif // #ifdef HAVE_CUDA
        }
        // cv::imwrite("/home/nvidia/Pictures/camera/2.jpg", tmp, {cv::IMWRITE_JPEG_QUALITY, 50});
    }
    else
    {
        tmp = image.clone();
    }

    float dw = inp_w - padw;
    float dh = inp_h - padh;

    dw /= 2.0f;
    dh /= 2.0f;
    int top = int(std::round(dh - 0.1f));
    int bottom = int(std::round(dh + 0.1f));
    int left = int(std::round(dw - 0.1f));
    int right = int(std::round(dw + 0.1f));

    cv::copyMakeBorder(tmp, tmp, top, bottom, left, right, cv::BORDER_CONSTANT, {114, 114, 114});

    // cv::dnn::blobFromImage(tmp, out, 1 / 255.f, cv::Size(), cv::Scalar(0, 0, 0), true, false, CV_32F);
    out.create({1, 3, (int)inp_h, (int)inp_w}, CV_32F);
    std::vector<cv::Mat> channels;
    cv::split(tmp, channels);
    cv::Mat c0((int)inp_h, (int)inp_w, CV_32F, (float *)out.data);
    cv::Mat c1((int)inp_h, (int)inp_w, CV_32F, (float *)out.data + (int)inp_h * (int)inp_w);
    cv::Mat c2((int)inp_h, (int)inp_w, CV_32F, (float *)out.data + (int)inp_h * (int)inp_w * 2);
    channels[0].convertTo(c2, CV_32F, 1 / 255.f);
    channels[1].convertTo(c1, CV_32F, 1 / 255.f);
    channels[2].convertTo(c0, CV_32F, 1 / 255.f);

    this->pparam.ratio = 1 / r;
    this->pparam.dw = dw;
    this->pparam.dh = dh;
    this->pparam.height = height;
    this->pparam.width = width;
}

void Infer_YOLOv8::Preprocess(const cv::Mat &image)
{
    cv::Mat nchw;
    auto &in_binding = this->input_bindings[0];
    auto width = in_binding.dims.d[3];
    auto height = in_binding.dims.d[2];
    cv::Size size{width, height};
    this->Letterbox(image, nchw, size);
    this->context->setBindingDimensions(0, nvinfer1::Dims{4, {1, 3, size.height, size.width}});
    CHECK(cudaMemcpyAsync(
        this->device_ptrs[0], nchw.ptr<float>(), nchw.total() * nchw.elemSize(), cudaMemcpyHostToDevice, this->stream));
}

void Infer_YOLOv8::Preprocess(const cv::Mat &image, cv::Size &size)
{
    cv::Mat nchw;
    this->Letterbox(image, nchw, size);
    this->context->setBindingDimensions(0, nvinfer1::Dims{4, {1, 3, size.height, size.width}});
    CHECK(cudaMemcpyAsync(
        this->device_ptrs[0], nchw.ptr<float>(), nchw.total() * nchw.elemSize(), cudaMemcpyHostToDevice, this->stream));
}

void Infer_YOLOv8::InferInternal()
{
    this->context->enqueueV2(this->device_ptrs.data(), this->stream, nullptr);
    for (int i = 0; i < this->num_outputs; i++)
    {
        size_t osize = this->output_bindings[i].size * this->output_bindings[i].dsize;
        CHECK(cudaMemcpyAsync(
            this->host_ptrs[i], this->device_ptrs[i + this->num_inputs], osize, cudaMemcpyDeviceToHost, this->stream));
    }
    cudaStreamSynchronize(this->stream);
}

// void Infer_YOLOv8::Postprocess(std::vector<DetectObject> &objs)
// {
//     objs.clear();
//     int *num_dets = static_cast<int *>(this->host_ptrs[0]);
//     auto *boxes = static_cast<float *>(this->host_ptrs[1]);
//     auto *scores = static_cast<float *>(this->host_ptrs[2]);
//     int *labels = static_cast<int *>(this->host_ptrs[3]);
//     auto &dw = this->pparam.dw;
//     auto &dh = this->pparam.dh;
//     auto &width = this->pparam.width;
//     auto &height = this->pparam.height;
//     auto &ratio = this->pparam.ratio;
//     LOG_DEBUG() << "num_dets[0] IS " << num_dets[0];

//     for (int i = 0; i < num_dets[0]; i++)
//     {
//         float *ptr = boxes + i * 4;

//         float x0 = *ptr++ - dw;
//         float y0 = *ptr++ - dh;
//         float x1 = *ptr++ - dw;
//         float y1 = *ptr - dh;

//         x0 = clamp(x0 * ratio, 0.f, width);
//         y0 = clamp(y0 * ratio, 0.f, height);
//         x1 = clamp(x1 * ratio, 0.f, width);
//         y1 = clamp(y1 * ratio, 0.f, height);
//         DetectObject obj;
//         obj.x = x0;
//         obj.y = y0;
//         obj.width = x1 - x0;
//         obj.height = y1 - y0;
//         obj.prob = *(scores + i);
//         obj.label = *(labels + i);
//         objs.push_back(obj);
//     }
// }

void Infer_YOLOv8::Postprocess(std::vector<DetectObject> &objs, float score_thres, float iou_thres, int topk, int num_labels)
{
    objs.clear();
    auto num_channels = this->output_bindings[0].dims.d[1];
    auto num_anchors = this->output_bindings[0].dims.d[2];

    auto &dw = this->pparam.dw;
    auto &dh = this->pparam.dh;
    auto &width = this->pparam.width;
    auto &height = this->pparam.height;
    auto &ratio = this->pparam.ratio;

    std::vector<cv::Rect> bboxes;
    std::vector<float> scores;
    std::vector<int> labels;
    std::vector<int> indices;

    cv::Mat output = cv::Mat(num_channels, num_anchors, CV_32F, static_cast<float *>(this->host_ptrs[0]));
    output = output.t();
    for (int i = 0; i < num_anchors; i++)
    {
        auto row_ptr = output.row(i).ptr<float>();
        auto bboxes_ptr = row_ptr;
        auto scores_ptr = row_ptr + 4;
        auto max_s_ptr = std::max_element(scores_ptr, scores_ptr + num_labels);
        float score = *max_s_ptr;
        if (score > score_thres)
        {
            float x = *bboxes_ptr++ - dw;
            float y = *bboxes_ptr++ - dh;
            float w = *bboxes_ptr++;
            float h = *bboxes_ptr;

            float x0 = clamp((x - 0.5f * w) * ratio, 0.f, width);
            float y0 = clamp((y - 0.5f * h) * ratio, 0.f, height);
            float x1 = clamp((x + 0.5f * w) * ratio, 0.f, width);
            float y1 = clamp((y + 0.5f * h) * ratio, 0.f, height);

            int label = max_s_ptr - scores_ptr;
            cv::Rect_<float> bbox;
            bbox.x = x0;
            bbox.y = y0;
            bbox.width = x1 - x0;
            bbox.height = y1 - y0;

            bboxes.push_back(bbox);
            labels.push_back(label);
            scores.push_back(score);
        }
    }
#ifdef BATCHED_NMS
    cv::dnn::NMSBoxesBatched(bboxes, scores, labels, score_thres, iou_thres, indices);
#else
    cv::dnn::NMSBoxes(bboxes, scores, score_thres, iou_thres, indices);
#endif
    int cnt = 0;
    for (auto &i : indices)
    {
        if (cnt >= topk)
        {
            break;
        }
        DetectObject obj;
        obj.x = bboxes[i].x;
        obj.y = bboxes[i].y;
        obj.width = bboxes[i].width;
        obj.height = bboxes[i].height;
        obj.prob = scores[i];
        obj.label = labels[i];
        objs.push_back(obj);
        cnt += 1;
    }
}

// void Infer_YOLOv8::draw_objects(const cv::Mat&                                image,
//                           cv::Mat&                                      res,
//                           const std::vector<DetectObject>&                    objs,
//                           const std::vector<std::string>&               CLASS_NAMES,
//                           const std::vector<std::vector<unsigned int>>& COLORS)
// {
//     res = image.clone();
//     for (auto& obj : objs) {
//         cv::Scalar color = cv::Scalar(COLORS[obj.label][0], COLORS[obj.label][1], COLORS[obj.label][2]);
//         cv::rectangle(res, cv::Rect(obj.x, obj.y, obj.width, obj.height), color, 2);

//         char text[256];
//         sprintf(text, "%s %.1f%%", CLASS_NAMES[obj.label].c_str(), obj.prob * 100);

//         int      baseLine   = 0;
//         cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseLine);

//         int x = (int)obj.x;
//         int y = (int)obj.y + 1;

//         if (y > res.rows)
//             y = res.rows;

//         cv::rectangle(res, cv::Rect(x, y, label_size.width, label_size.height + baseLine), {0, 0, 255}, -1);

//         cv::putText(res, text, cv::Point(x, y + label_size.height), cv::FONT_HERSHEY_SIMPLEX, 0.4, {255, 255, 255}, 1);
//     }
// }

END_NS_ZF_DETECTION
#endif // ZF_JETSON_INFER_TENSORRT_H