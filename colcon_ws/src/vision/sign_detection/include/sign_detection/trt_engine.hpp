// TensorRT 추론 (젯슨 GPU) - OpenCV DNN 의 net.setInput + forward 자리를 대신한다
//
// 엔진(.engine)은 ONNX 를 젯슨에서 trtexec 로 바꾼 것. 기기 + TensorRT 버전에 묶여 있어 노트북에서 만들면 안 되고,
// JetPack 을 다시 깔면 다시 만들어야 한다 (config/sign_detection.yaml 의 engine_path 설명)
//   2026-10-27 v2_416 FP16: 추론 3.3ms (OpenCV CPU 68ms)
// 입력/출력 모양은 ONNX 와 같다: 입력 1 x 3 x S x S (blobFromImage 결과 그대로), 출력 1 x (4 + 클래스 수) x N
#pragma once

#include <cstdio>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <cuda_runtime_api.h>
#include <NvInfer.h>
#include <opencv2/core.hpp>

namespace sign_detection
{

class TrtEngine
{
public:
  // 엔진 파일을 읽어 GPU 버퍼까지 잡는다. 실패하면 std::runtime_error
  explicit TrtEngine(const std::string & path)
  {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
      throw std::runtime_error("엔진 파일 없음: " + path);
    }
    const std::vector<char> data((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());

    runtime_.reset(nvinfer1::createInferRuntime(logger_));
    engine_.reset(runtime_ ? runtime_->deserializeCudaEngine(data.data(), data.size()) : nullptr);
    if (!engine_) {
      throw std::runtime_error("엔진을 읽지 못함 (다른 기기/TensorRT 버전에서 만든 엔진?): " + path);
    }
    context_.reset(engine_->createExecutionContext());
    if (!context_ || engine_->getNbIOTensors() != 2) {
      throw std::runtime_error("입력 1개 + 출력 1개 엔진이 아님: " + path);
    }
    check(cudaStreamCreate(&stream_), "cudaStreamCreate");

    for (int i = 0; i < 2; ++i) {
      const char * name = engine_->getIOTensorName(i);
      const nvinfer1::Dims d = engine_->getTensorShape(name);
      if (engine_->getTensorDataType(name) != nvinfer1::DataType::kFLOAT) {
        throw std::runtime_error(std::string("float32 텐서가 아님: ") + name);
      }
      std::vector<int> shape(d.d, d.d + d.nbDims);
      size_t bytes = sizeof(float);
      for (int s : shape) {
        bytes *= s;
      }
      void * buf = nullptr;
      check(cudaMalloc(&buf, bytes), "cudaMalloc");
      context_->setTensorAddress(name, buf);
      if (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT) {
        in_ = buf; in_bytes_ = bytes; in_shape_ = shape;
      } else {
        out_ = buf; out_bytes_ = bytes; out_shape_ = shape;
      }
    }
    if (in_shape_.size() != 4 || in_shape_[0] != 1 || in_shape_[1] != 3 || out_shape_.size() != 3) {
      throw std::runtime_error("입력 1x3xSxS / 출력 3차원 모양이 아님: " + path);
    }
    // 첫 추론은 CUDA 초기화로 3초 넘게 걸려서 (2026-10-27 재생 3349ms) 시작할 때 한 번 미리 돌린다
    infer(cv::Mat(in_shape_.size(), in_shape_.data(), CV_32F, cv::Scalar(0)));
  }

  ~TrtEngine()
  {
    cudaFree(in_);
    cudaFree(out_);
    if (stream_) {
      cudaStreamDestroy(stream_);
    }
  }

  TrtEngine(const TrtEngine &) = delete;
  TrtEngine & operator=(const TrtEngine &) = delete;

  // 엔진 입력 한 변 크기 (export 때 imgsz)
  int inputSize() const {return in_shape_[2];}

  // blob: blobFromImage 결과 (1 x 3 x S x S, CV_32F). 반환: 1 x (4 + nc) x N (CV_32F, 새 메모리)
  cv::Mat infer(const cv::Mat & blob)
  {
    if (!blob.isContinuous() || blob.total() * blob.elemSize() != in_bytes_) {
      throw std::runtime_error("입력 blob 크기가 엔진 입력과 다름");
    }
    cv::Mat out(static_cast<int>(out_shape_.size()), out_shape_.data(), CV_32F);
    check(cudaMemcpyAsync(in_, blob.ptr(), in_bytes_, cudaMemcpyHostToDevice, stream_), "입력 복사");
    if (!context_->enqueueV3(stream_)) {
      throw std::runtime_error("enqueueV3 실패");
    }
    check(cudaMemcpyAsync(out.ptr(), out_, out_bytes_, cudaMemcpyDeviceToHost, stream_), "출력 복사");
    check(cudaStreamSynchronize(stream_), "cudaStreamSynchronize");
    return out;
  }

private:
  // TensorRT 경고 이상만 stderr 로 (ROS 로그에 같이 찍힌다)
  struct Logger : nvinfer1::ILogger
  {
    void log(Severity s, const char * msg) noexcept override
    {
      if (s <= Severity::kWARNING) {
        std::fprintf(stderr, "[TensorRT] %s\n", msg);
      }
    }
  };

  static void check(cudaError_t e, const char * what)
  {
    if (e != cudaSuccess) {
      throw std::runtime_error(std::string(what) + ": " + cudaGetErrorString(e));
    }
  }

  Logger logger_;
  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> context_;
  cudaStream_t stream_ = nullptr;
  void * in_ = nullptr;
  void * out_ = nullptr;
  size_t in_bytes_ = 0, out_bytes_ = 0;
  std::vector<int> in_shape_, out_shape_;
};

}  // namespace sign_detection
