// TensorRT 추론 (젯슨 GPU): 입력 1개 + 출력 여러 개 (YOLO-seg 는 박스 output0 + proto output1)
//   sign_detection/trt_engine.hpp (출력 1개) 를 넓힌 것. 엔진은 젯슨에서 만든다 (기기 + TensorRT 버전에 묶임)
//   2026-10-28 road_r2_320_fp16.engine: 추론 2.96ms
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

namespace grid_drive
{

class TrtEngine
{
public:
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
    if (!context_) {
      throw std::runtime_error("createExecutionContext 실패: " + path);
    }
    check(cudaStreamCreate(&stream_), "cudaStreamCreate");

    for (int i = 0; i < engine_->getNbIOTensors(); ++i) {
      const char * name = engine_->getIOTensorName(i);
      const nvinfer1::Dims d = engine_->getTensorShape(name);
      if (engine_->getTensorDataType(name) != nvinfer1::DataType::kFLOAT) {
        throw std::runtime_error(std::string("float32 텐서가 아님: ") + name);
      }
      Tensor t;
      t.shape.assign(d.d, d.d + d.nbDims);
      t.bytes = sizeof(float);
      for (int s : t.shape) {
        t.bytes *= s;
      }
      check(cudaMalloc(&t.buf, t.bytes), "cudaMalloc");
      context_->setTensorAddress(name, t.buf);
      if (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT) {
        if (in_.buf) {
          throw std::runtime_error("입력이 2개 이상: " + path);
        }
        in_ = t;
      } else {
        outs_.push_back(t);   // 엔진 순서 (output0, output1, ...)
      }
    }
    if (in_.shape.size() != 4 || in_.shape[0] != 1 || in_.shape[1] != 3 || outs_.empty()) {
      throw std::runtime_error("입력 1x3xSxS / 출력 있음 모양이 아님: " + path);
    }
    // 첫 추론은 CUDA 초기화로 수 초 걸려서 시작할 때 한 번
    infer(cv::Mat(static_cast<int>(in_.shape.size()), in_.shape.data(), CV_32F, cv::Scalar(0)));
  }

  ~TrtEngine()
  {
    cudaFree(in_.buf);
    for (auto & t : outs_) {
      cudaFree(t.buf);
    }
    if (stream_) {
      cudaStreamDestroy(stream_);
    }
  }

  TrtEngine(const TrtEngine &) = delete;
  TrtEngine & operator=(const TrtEngine &) = delete;

  int inputSize() const {return in_.shape[2];}

  // blob: blobFromImage 결과 (1 x 3 x S x S, CV_32F). 반환: 출력들 (엔진 순서, 새 메모리)
  std::vector<cv::Mat> infer(const cv::Mat & blob)
  {
    if (!blob.isContinuous() || blob.total() * blob.elemSize() != in_.bytes) {
      throw std::runtime_error("입력 blob 크기가 엔진 입력과 다름");
    }
    check(cudaMemcpyAsync(in_.buf, blob.ptr(), in_.bytes, cudaMemcpyHostToDevice, stream_), "입력 복사");
    if (!context_->enqueueV3(stream_)) {
      throw std::runtime_error("enqueueV3 실패");
    }
    std::vector<cv::Mat> res;
    for (auto & t : outs_) {
      res.emplace_back(static_cast<int>(t.shape.size()), t.shape.data(), CV_32F);
      check(cudaMemcpyAsync(res.back().ptr(), t.buf, t.bytes, cudaMemcpyDeviceToHost, stream_), "출력 복사");
    }
    check(cudaStreamSynchronize(stream_), "cudaStreamSynchronize");
    return res;
  }

private:
  struct Logger : nvinfer1::ILogger
  {
    void log(Severity s, const char * msg) noexcept override
    {
      if (s <= Severity::kWARNING) {
        std::fprintf(stderr, "[TensorRT] %s\n", msg);
      }
    }
  };
  struct Tensor
  {
    std::vector<int> shape;
    size_t bytes = 0;
    void * buf = nullptr;
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
  Tensor in_;
  std::vector<Tensor> outs_;
};

}  // namespace grid_drive
