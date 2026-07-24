// TensorRT semantic segmentation on the CSI camera stream.
//
// Runs the standard pretrained FCN-ResNet18 ONNX models from the dusty-nv
// jetson-inference zoo (engine pre-built by ensure_model.sh via trtexec) and
// mirrors the topic interface of ros_deep_learning's segnet node:
// subscribes image_in, publishes overlay / color_mask / class_mask.
// GPU libraries resolve through the phase-1 compute-slice mounts; this file
// compiles with the stock jammy toolchain (no nvcc, no custom kernels).

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <NvInfer.h>
#include <cuda_runtime_api.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace
{

class TrtLogger : public nvinfer1::ILogger
{
public:
  void log(Severity severity, const char * msg) noexcept override
  {
    auto logger = rclcpp::get_logger("segnet.trt");
    switch (severity) {
      case Severity::kINTERNAL_ERROR:
      case Severity::kERROR:
        RCLCPP_ERROR(logger, "%s", msg);
        break;
      case Severity::kWARNING:
        RCLCPP_WARN(logger, "%s", msg);
        break;
      default:
        RCLCPP_DEBUG(logger, "%s", msg);
        break;
    }
  }
};

void cudaCheck(cudaError_t err, const char * what)
{
  if (err != cudaSuccess) {
    throw std::runtime_error(std::string(what) + ": " + cudaGetErrorString(err));
  }
}

size_t volume(const nvinfer1::Dims & d)
{
  size_t v = 1;
  for (int i = 0; i < d.nbDims; ++i) {
    v *= static_cast<size_t>(d.d[i]);
  }
  return v;
}

// Standard Pascal VOC colormap (bit-reversal construction); fallback for
// classes beyond what colors.txt provides.
cv::Vec3b vocColor(int idx)
{
  uint8_t r = 0, g = 0, b = 0;
  int c = idx;
  for (int j = 0; j < 8; ++j) {
    r |= ((c >> 0) & 1) << (7 - j);
    g |= ((c >> 1) & 1) << (7 - j);
    b |= ((c >> 2) & 1) << (7 - j);
    c >>= 3;
  }
  return cv::Vec3b(r, g, b);
}

}  // namespace

class SegNetNode : public rclcpp::Node
{
public:
  SegNetNode()
  : Node("segnet")
  {
    const auto engine_path = declare_parameter<std::string>("engine_path", "");
    const auto labels_path = declare_parameter<std::string>("labels_path", "");
    const auto colors_path = declare_parameter<std::string>("colors_path", "");
    overlay_alpha_ = declare_parameter<double>("overlay_alpha", 120.0);

    if (engine_path.empty()) {
      throw std::runtime_error("engine_path parameter is required");
    }

    loadEngine(engine_path);
    loadClassInfo(labels_path, colors_path);

    overlay_pub_ = create_publisher<sensor_msgs::msg::Image>("overlay", rclcpp::QoS(1));
    color_pub_ = create_publisher<sensor_msgs::msg::Image>("color_mask", rclcpp::QoS(1));
    class_pub_ = create_publisher<sensor_msgs::msg::Image>("class_mask", rclcpp::QoS(1));

    sub_ = create_subscription<sensor_msgs::msg::Image>(
      "image_in", rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {onImage(std::move(msg));});

    RCLCPP_INFO(
      get_logger(), "segnet ready: %dx%d input -> %dx%d grid, %d classes (%s)",
      in_w_, in_h_, out_w_, out_h_, num_classes_, engine_path.c_str());
  }

  ~SegNetNode() override
  {
    if (d_input_) {cudaFree(d_input_);}
    if (d_output_) {cudaFree(d_output_);}
    if (stream_) {cudaStreamDestroy(stream_);}
    // TensorRT 7 object lifecycle (delete is TRT8+)
    if (context_) {context_->destroy();}
    if (engine_) {engine_->destroy();}
    if (runtime_) {runtime_->destroy();}
  }

private:
  void loadEngine(const std::string & path)
  {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
      throw std::runtime_error("cannot open engine file: " + path);
    }
    std::vector<char> blob(
      (std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());

    runtime_ = nvinfer1::createInferRuntime(trt_logger_);
    if (!runtime_) {
      throw std::runtime_error("createInferRuntime failed");
    }
    engine_ = runtime_->deserializeCudaEngine(blob.data(), blob.size(), nullptr);
    if (!engine_) {
      throw std::runtime_error("deserializeCudaEngine failed: " + path);
    }
    context_ = engine_->createExecutionContext();
    if (!context_) {
      throw std::runtime_error("createExecutionContext failed");
    }

    // Nothing is model-specific below: binding indices and tensor shapes are
    // read from the engine (dusty-nv exports use input_0/output_0, 1x3xHxW in,
    // 1xCxHxW class scores out).
    int input_idx = -1;
    int output_idx = -1;
    for (int i = 0; i < engine_->getNbBindings(); ++i) {
      if (engine_->bindingIsInput(i)) {
        if (input_idx < 0) {input_idx = i;}
      } else if (output_idx < 0) {
        output_idx = i;
      }
    }
    if (input_idx < 0 || output_idx < 0) {
      throw std::runtime_error("engine lacks an input/output binding");
    }

    const auto in_dims = engine_->getBindingDimensions(input_idx);
    const auto out_dims = engine_->getBindingDimensions(output_idx);
    if (in_dims.nbDims != 4 || in_dims.d[0] != 1 || in_dims.d[1] != 3 ||
      out_dims.nbDims != 4 || out_dims.d[0] != 1)
    {
      throw std::runtime_error("unexpected binding shapes (want 1x3xHxW in, 1xCxHxW out)");
    }
    in_h_ = in_dims.d[2];
    in_w_ = in_dims.d[3];
    num_classes_ = out_dims.d[1];
    out_h_ = out_dims.d[2];
    out_w_ = out_dims.d[3];
    if (num_classes_ < 1 || num_classes_ > 255) {
      throw std::runtime_error("class count out of mono8 range: " + std::to_string(num_classes_));
    }

    h_input_.resize(volume(in_dims));
    h_output_.resize(volume(out_dims));
    cudaCheck(cudaStreamCreate(&stream_), "cudaStreamCreate");
    cudaCheck(cudaMalloc(&d_input_, h_input_.size() * sizeof(float)), "cudaMalloc input");
    cudaCheck(cudaMalloc(&d_output_, h_output_.size() * sizeof(float)), "cudaMalloc output");
    bindings_.assign(engine_->getNbBindings(), nullptr);
    bindings_[input_idx] = d_input_;
    bindings_[output_idx] = d_output_;
  }

  void loadClassInfo(const std::string & labels_path, const std::string & colors_path)
  {
    if (!labels_path.empty()) {
      std::ifstream f(labels_path);
      std::string line;
      while (f && std::getline(f, line)) {
        if (!line.empty() && line.back() == '\r') {
          line.pop_back();
        }
        labels_.push_back(line);
      }
      if (labels_.empty()) {
        RCLCPP_WARN(get_logger(), "no labels read from %s", labels_path.c_str());
      }
    }
    while (static_cast<int>(labels_.size()) < num_classes_) {
      labels_.push_back("class " + std::to_string(labels_.size()));
    }

    if (!colors_path.empty()) {
      std::ifstream f(colors_path);
      std::string line;
      while (f && std::getline(f, line)) {
        std::istringstream ss(line);
        int r, g, b;
        if (ss >> r >> g >> b) {
          colors_.push_back(cv::Vec3b(r, g, b));
        }
      }
      if (colors_.empty()) {
        RCLCPP_WARN(
          get_logger(), "no colors read from %s, using generated palette", colors_path.c_str());
      }
    }
    while (static_cast<int>(colors_.size()) < num_classes_) {
      colors_.push_back(vocColor(colors_.size()));
    }
  }

  void onImage(sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    const bool want_class = class_pub_->get_subscription_count() > 0;
    const bool want_color = color_pub_->get_subscription_count() > 0;
    const bool want_overlay = overlay_pub_->get_subscription_count() > 0;
    if (!want_class && !want_color && !want_overlay) {
      return;  // nobody listening: skip inference entirely
    }

    cv_bridge::CvImageConstPtr in;
    try {
      in = cv_bridge::toCvShare(msg, "rgb8");
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "cv_bridge: %s", e.what());
      return;
    }

    // Preprocess: resize to the network input, imagenet-normalize into NCHW
    // (segNet.cpp ONNX path: RGB in [0,1], mean/std below).
    cv::Mat net_in;
    cv::resize(in->image, net_in, cv::Size(in_w_, in_h_), 0, 0, cv::INTER_LINEAR);
    static constexpr float kMean[3] = {0.485f, 0.456f, 0.406f};
    static constexpr float kStd[3] = {0.229f, 0.224f, 0.225f};
    const int hw = in_h_ * in_w_;
    const uint8_t * px = net_in.ptr<uint8_t>(0);
    for (int i = 0; i < hw; ++i) {
      for (int c = 0; c < 3; ++c) {
        h_input_[c * hw + i] = (px[3 * i + c] / 255.0f - kMean[c]) / kStd[c];
      }
    }

    const auto t0 = std::chrono::steady_clock::now();
    cudaCheck(
      cudaMemcpyAsync(
        d_input_, h_input_.data(), h_input_.size() * sizeof(float),
        cudaMemcpyHostToDevice, stream_), "H2D copy");
    if (!context_->enqueueV2(bindings_.data(), stream_, nullptr)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "TensorRT enqueueV2 failed");
      return;
    }
    cudaCheck(
      cudaMemcpyAsync(
        h_output_.data(), d_output_, h_output_.size() * sizeof(float),
        cudaMemcpyDeviceToHost, stream_), "D2H copy");
    cudaCheck(cudaStreamSynchronize(stream_), "stream sync");
    const double ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

    // Argmax over class scores at the output grid resolution
    cv::Mat grid(out_h_, out_w_, CV_8UC1);
    const int ohw = out_h_ * out_w_;
    uint8_t * gp = grid.ptr<uint8_t>(0);
    for (int i = 0; i < ohw; ++i) {
      int best = 0;
      float best_v = h_output_[i];
      for (int c = 1; c < num_classes_; ++c) {
        const float v = h_output_[c * ohw + i];
        if (v > best_v) {
          best_v = v;
          best = c;
        }
      }
      gp[i] = static_cast<uint8_t>(best);
    }

    const cv::Size out_size(in->image.cols, in->image.rows);
    if (want_class) {
      cv::Mat class_mask;
      cv::resize(grid, class_mask, out_size, 0, 0, cv::INTER_NEAREST);
      class_pub_->publish(*cv_bridge::CvImage(msg->header, "mono8", class_mask).toImageMsg());
    }
    if (want_color || want_overlay) {
      cv::Mat color_grid(out_h_, out_w_, CV_8UC3);
      cv::Vec3b * cp = color_grid.ptr<cv::Vec3b>(0);
      for (int i = 0; i < ohw; ++i) {
        cp[i] = colors_[gp[i]];
      }
      // Linear upscale blends class colors at boundaries, matching the segnet
      // node's default 'linear' mask/overlay filter.
      cv::Mat color_mask;
      cv::resize(color_grid, color_mask, out_size, 0, 0, cv::INTER_LINEAR);
      if (want_color) {
        color_pub_->publish(*cv_bridge::CvImage(msg->header, "rgb8", color_mask).toImageMsg());
      }
      if (want_overlay) {
        const double a = std::clamp(overlay_alpha_, 0.0, 255.0) / 255.0;
        cv::Mat overlay;
        cv::addWeighted(in->image, 1.0 - a, color_mask, a, 0.0, overlay);
        overlay_pub_->publish(*cv_bridge::CvImage(msg->header, "rgb8", overlay).toImageMsg());
      }
    }

    infer_ms_sum_ += ms;
    if (++infer_count_ >= 50) {
      RCLCPP_INFO(
        get_logger(), "inference %.1f ms avg over last %d frames",
        infer_ms_sum_ / infer_count_, infer_count_);
      infer_ms_sum_ = 0.0;
      infer_count_ = 0;
    }
  }

  TrtLogger trt_logger_;
  nvinfer1::IRuntime * runtime_{nullptr};
  nvinfer1::ICudaEngine * engine_{nullptr};
  nvinfer1::IExecutionContext * context_{nullptr};
  cudaStream_t stream_{};
  void * d_input_{nullptr};
  void * d_output_{nullptr};
  std::vector<void *> bindings_;
  std::vector<float> h_input_;
  std::vector<float> h_output_;
  int in_w_{0}, in_h_{0}, out_w_{0}, out_h_{0}, num_classes_{0};
  double overlay_alpha_{120.0};
  std::vector<std::string> labels_;
  std::vector<cv::Vec3b> colors_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlay_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr class_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  double infer_ms_sum_{0.0};
  int infer_count_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<SegNetNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("segnet"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
