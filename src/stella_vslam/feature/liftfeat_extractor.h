#ifndef STELLA_VSLAM_FEATURE_LIFTFEAT_EXTRACTOR_H
#define STELLA_VSLAM_FEATURE_LIFTFEAT_EXTRACTOR_H

#include <opencv2/core.hpp>
#include <vector>
#include <string>
#include <memory>

#include <onnxruntime_cxx_api.h>

namespace stella_vslam {
namespace feature {

class liftfeat_extractor {
public:
    liftfeat_extractor() = delete;

    //! Constructor
    //! @param model_path Path to ONNX model (onnx.data must be in same directory)
    //! @param conf_thresh Confidence threshold for keypoint detection
    //! @param max_keypoints Maximum number of keypoints to extract
    liftfeat_extractor(const std::string& model_path,
                       float conf_thresh = 0.015f,
                       int max_keypoints = 4096);

    virtual ~liftfeat_extractor();

    //! Extract keypoints and descriptors
    //! @param in_image Input image (BGR or grayscale, CV_8UC1/CV_8UC3)
    //! @param in_image_mask Mask image (unused, empty matrix if not used)
    //! @param keypts Extracted keypoints (octave=0, size=31.0f, response=score)
    //! @param descriptors Output descriptors (CV_32F, Nx64)
    //! @return Number of extracted keypoints (0 = no extraction)
    int extract(const cv::_InputArray& in_image, const cv::_InputArray& in_image_mask,
                std::vector<cv::KeyPoint>& keypts, cv::Mat& descriptors);

private:
    //! ONNX Runtime related variables
    Ort::Env env_;
    std::unique_ptr<Ort::Session> session_;
    Ort::MemoryInfo memory_info_;

    //! Input/output name strings (kept alive via AllocatedStringPtr)
    Ort::AllocatedStringPtr input_name_{nullptr, Ort::detail::AllocatedFree(nullptr)};
    Ort::AllocatedStringPtr output_name_0_{nullptr, Ort::detail::AllocatedFree(nullptr)};
    Ort::AllocatedStringPtr output_name_1_{nullptr, Ort::detail::AllocatedFree(nullptr)};
    Ort::AllocatedStringPtr output_name_2_{nullptr, Ort::detail::AllocatedFree(nullptr)};
    Ort::AllocatedStringPtr output_name_3_{nullptr, Ort::detail::AllocatedFree(nullptr)};

    //! Allocator for name allocation
    Ort::AllocatorWithDefaultOptions allocator_;

    float conf_thresh_;
    int max_keypoints_;

    //! Decode function: Softmax -> NMS -> GridSample -> L2 normalization
    void decode_outputs(const float* score_map, const float* desc_map,
                        int Hc, int Wc, int H, int W,
                        std::vector<cv::KeyPoint>& keypts, cv::Mat& descriptors);
};

} // namespace feature
} // namespace stella_vslam

#endif // STELLA_VSLAM_FEATURE_LIFTFEAT_EXTRACTOR_H
