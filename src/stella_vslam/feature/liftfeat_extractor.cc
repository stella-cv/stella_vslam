#include "stella_vslam/feature/liftfeat_extractor.h"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <numeric>

#include <spdlog/spdlog.h>

namespace stella_vslam {
namespace feature {

liftfeat_extractor::liftfeat_extractor(const std::string& model_path,
                                       float conf_thresh,
                                       int max_keypoints)
    : memory_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)),
      conf_thresh_(conf_thresh), max_keypoints_(max_keypoints) {
    // Initialize ONNX Runtime environment
    env_ = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "LiftFeat");
    Ort::SessionOptions session_opts;
    session_opts.SetIntraOpNumThreads(1);
    session_opts.SetInterOpNumThreads(1);

    OrtCUDAProviderOptions cuda_options;
    session_opts.AppendExecutionProvider_CUDA(cuda_options);

    // Load ONNX model
    try {
        session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), session_opts);
    }
    catch (const Ort::Exception& e) {
        SPDLOG_ERROR("Failed to load ONNX model: {}", e.what());
        throw;
    }

    // Get input/output info (unused, but validates the model structure)
    session_->GetInputTypeInfo(0);
    session_->GetOutputTypeInfo(0);
    session_->GetOutputTypeInfo(1);

    // Get name strings — AllocatedStringPtr keeps them alive for the lifetime of this object
    allocator_ = Ort::AllocatorWithDefaultOptions();
    input_name_ = session_->GetInputNameAllocated(0, allocator_);
    output_name_0_ = session_->GetOutputNameAllocated(0, allocator_);
    output_name_1_ = session_->GetOutputNameAllocated(1, allocator_);
    output_name_2_ = session_->GetOutputNameAllocated(2, allocator_);
    output_name_3_ = session_->GetOutputNameAllocated(3, allocator_);

    SPDLOG_INFO("LiftFeat model loaded: {} (input: {}, outputs: {}+{}+{}+{})",
                model_path,
                input_name_.get(),
                output_name_0_.get(),
                output_name_1_.get(),
                output_name_2_.get(),
                output_name_3_.get());
}

int liftfeat_extractor::extract(const cv::_InputArray& in_image,
                                const cv::_InputArray& in_image_mask,
                                std::vector<cv::KeyPoint>& keypts,
                                cv::Mat& descriptors) {
    // Ignore mask (not used in LiftFeat)
    (void)in_image_mask;

    if (in_image.empty()) {
        keypts.clear();
        descriptors.release();
        return 0;
    }

    const auto image = in_image.getMat();

    // Convert to grayscale if needed
    cv::Mat gray_image;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
    }
    else {
        gray_image = image;
    }

    // Convert to RGB (LiftFeat expects RGB)
    cv::Mat rgb_image;
    if (image.channels() == 3) {
        cv::cvtColor(image, rgb_image, cv::COLOR_BGR2RGB);
    }
    else {
        cv::cvtColor(gray_image, rgb_image, cv::COLOR_GRAY2RGB);
    }

    // Resize to multiple of 32
    const int H = rgb_image.rows;
    const int W = rgb_image.cols;
    const int new_H = (H / 32) * 32;
    const int new_W = (W / 32) * 32;

    cv::Mat resized_image;
    float scale_x = 1.0f;
    float scale_y = 1.0f;

    if (H != new_H || W != new_W) {
        scale_x = static_cast<float>(new_W) / W;
        scale_y = static_cast<float>(new_H) / H;
        cv::resize(rgb_image, resized_image, cv::Size(new_W, new_H), 0, 0, cv::INTER_LINEAR);
    }
    else {
        resized_image = rgb_image;
    }

    // Normalize to [0, 1] and transpose to [1, 3, H, W]
    cv::Mat float_image;
    resized_image.convertTo(float_image, CV_32F, 1.0f / 255.0f);

    // Split into channels and concatenate into [C, H, W] = [3, H, W] layout for the tensor
    std::vector<cv::Mat> channels(3);
    cv::split(float_image, channels);
    std::vector<float> channel_data(3 * new_H * new_W);
    for (int c = 0; c < 3; ++c) {
        const float* src = channels[c].ptr<float>();
        std::copy(src, src + new_H * new_W, channel_data.data() + c * new_H * new_W);
    }

    // Create input tensor [1, 3, H, W]
    const int64_t input_dims[4] = {1, 3, static_cast<int64_t>(new_H), static_cast<int64_t>(new_W)};
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info_, channel_data.data(), channel_data.size(), input_dims, 4);

    // Run inference
    const char* input_names[] = {input_name_.get()};
    const char* output_names[] = {
        output_name_0_.get(),
        output_name_1_.get(),
        output_name_2_.get(),
        output_name_3_.get()};

    auto output_tensors = session_->Run(
        Ort::RunOptions{nullptr}, input_names, &input_tensor, 1, output_names, 4);

    const auto& desc_tensor = output_tensors[1].GetTensorData<float>();
    const auto& score_tensor = output_tensors[2].GetTensorData<float>();

    // GetShape() returns std::vector<int64_t>
    const auto desc_dims = output_tensors[1].GetTensorTypeAndShapeInfo().GetShape();
    const auto score_dims = output_tensors[2].GetTensorTypeAndShapeInfo().GetShape();

    const int Hc = static_cast<int>(desc_dims[2]);
    const int Wc = static_cast<int>(desc_dims[3]);

    const float* desc_map = desc_tensor;
    const float* score_map = score_tensor;

    // Decode outputs
    keypts.clear();
    decode_outputs(score_map, desc_map, Hc, Wc, new_H, new_W, keypts, descriptors);

    // Scale coordinates back to original image size
    if (!keypts.empty()) {
        for (auto& kp : keypts) {
            kp.pt.x /= scale_x;
            kp.pt.y /= scale_y;
        }
    }

    // Set keypoint metadata
    for (auto& kp : keypts) {
        kp.octave = 0;
        kp.size = 31.0f;
    }

    return static_cast<int>(keypts.size());
}

void liftfeat_extractor::decode_outputs(const float* score_map,
                                        const float* desc_map,
                                        int Hc, int Wc,
                                        int H, int W,
                                        std::vector<cv::KeyPoint>& keypts,
                                        cv::Mat& descriptors) {
    // Softmax over 65 channels, drop background (last channel)
    // Then PixelShuffle 8x8 to get heatmap [H, W]
    std::vector<float> heatmap(H * W, 0.0f);

    for (int h = 0; h < Hc; ++h) {
        for (int w = 0; w < Wc; ++w) {
            // Find max for numerical stability
            float max_val = -FLT_MAX;
            for (int c = 0; c < 65; ++c) {
                const float val = score_map[c * Hc * Wc + h * Wc + w];
                if (val > max_val) {
                    max_val = val;
                }
            }

            // Softmax over all 65 channels, then drop background (channel 64).
            float exp_vals[65];
            float sum_exp = 0.0f;
            for (int c = 0; c < 65; ++c) {
                const float val = score_map[c * Hc * Wc + h * Wc + w];
                exp_vals[c] = std::exp(val - max_val);
                sum_exp += exp_vals[c];
            }

            // PixelShuffle: [64, Hc, Wc] -> [8*Hc, 8*Wc] = [H, W]
            // Only channels 0..63 are used; background (64) is dropped.
            for (int ph = 0; ph < 8; ++ph) {
                for (int pw = 0; pw < 8; ++pw) {
                    const int ch = ph * 8 + pw;
                    const int idx = (h * 8 + ph) * W + (w * 8 + pw);
                    if (idx < H * W) {
                        heatmap[idx] = exp_vals[ch] / sum_exp;
                    }
                }
            }
        }
    }

    cv::Mat heatmap_mat(H, W, CV_32F, heatmap.data());

    std::vector<cv::KeyPoint> candidate_keypts;

    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            const float val = heatmap_mat.at<float>(y, x);
            if (val > conf_thresh_) {
                cv::KeyPoint kp(static_cast<float>(x), static_cast<float>(y), 31.0f, 0.0f, static_cast<float>(val));
                candidate_keypts.push_back(kp);
            }
        }
    }

    if (candidate_keypts.empty()) {
        descriptors.release();
        return;
    }

    // Top-K by score
    std::sort(candidate_keypts.begin(), candidate_keypts.end(),
              [](const cv::KeyPoint& a, const cv::KeyPoint& b) {
                  return a.response > b.response;
              });

    const int num_keypts = std::min(static_cast<int>(candidate_keypts.size()), max_keypoints_);
    keypts.resize(num_keypts);
    std::copy(candidate_keypts.begin(), candidate_keypts.begin() + num_keypts, keypts.begin());

    // Sample descriptors using bilinear interpolation (cv::getRectSubPix)
    // Grid coordinates for grid_sample: (pt / (W-1)) * 2.0 - 1.0
    descriptors.create(num_keypts, 64, CV_32F);

    for (int i = 0; i < num_keypts; ++i) {
        const float cx = keypts[i].pt.x;
        const float cy = keypts[i].pt.y;

        // Convert to grid_sample coordinates
        const float grid_x = (cx / std::max(1, W - 1)) * 2.0f - 1.0f;
        const float grid_y = (cy / std::max(1, H - 1)) * 2.0f - 1.0f;

        // Sample 64-dim descriptor from desc_map
        // desc_map is [1, 64, H, W] -> need to interpolate at (grid_x, grid_y)
        const float* desc_ptr = desc_map;

        for (int c = 0; c < 64; ++c) {
            // Bilinear interpolation at (grid_y, grid_x) for channel c.
            // desc_map is [1, 64, Hc, Wc] — grid_sample operates on desc_map's actual size.
            // grid_x/y are in [-1, 1] range (align_corners=True); map to [0, Wc-1], [0, Hc-1].
            const float x = (grid_x + 1.0f) * 0.5f * (Wc - 1);
            const float y = (grid_y + 1.0f) * 0.5f * (Hc - 1);

            const int x0 = std::max(0, std::min(Wc - 2, static_cast<int>(std::floor(x))));
            const int y0 = std::max(0, std::min(Hc - 2, static_cast<int>(std::floor(y))));
            const int x1 = x0 + 1;
            const int y1 = y0 + 1;

            const float fx = x - x0;
            const float fy = y - y0;

            const float val00 = desc_ptr[(c * Hc + y0) * Wc + x0];
            const float val10 = desc_ptr[(c * Hc + y0) * Wc + x1];
            const float val01 = desc_ptr[(c * Hc + y1) * Wc + x0];
            const float val11 = desc_ptr[(c * Hc + y1) * Wc + x1];

            const float bilinear = val00 * (1 - fx) * (1 - fy)
                                   + val10 * fx * (1 - fy)
                                   + val01 * (1 - fx) * fy
                                   + val11 * fx * fy;

            descriptors.at<float>(i, c) = bilinear;
        }
    }

    // L2 normalize descriptors
    for (int i = 0; i < num_keypts; ++i) {
        float norm = 0.0f;
        for (int j = 0; j < 64; ++j) {
            norm += descriptors.at<float>(i, j) * descriptors.at<float>(i, j);
        }
        norm = std::sqrt(norm) + 1e-10f;
        for (int j = 0; j < 64; ++j) {
            descriptors.at<float>(i, j) /= norm;
        }
    }
}

liftfeat_extractor::~liftfeat_extractor() {}

} // namespace feature
} // namespace stella_vslam
