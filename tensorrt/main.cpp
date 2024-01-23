#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <NvInferLegacyDims.h>
#include <NvInferRuntime.h>
#include <NvInferImpl.h>

class Logger : public nvinfer1::ILogger {
    void log(Severity severity, const char* msg) noexcept override {
        // suppress info-level messages
        if (severity <= Severity::kWARNING)
            std::cout << msg << std::endl;
    }
} logger;

int main() {
    try {
        /// 构建期
        auto builder = std::unique_ptr<nvinfer1::IBuilder>(nvinfer1::createInferBuilder(logger));
        uint32_t explicitBatch = 1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
        auto network = std::unique_ptr<nvinfer1::INetworkDefinition>(builder->createNetworkV2(explicitBatch));

        std::string onnx_filename = "/home/icv/workspaces/inf_onsite/test/onnxfiles/3d_det_all.onnx";
        auto parser = std::unique_ptr<nvonnxparser::IParser>(nvonnxparser::createParser(*network, logger));
        parser->parseFromFile(onnx_filename.c_str(), static_cast<int32_t>(nvinfer1::ILogger::Severity::kWARNING));

        auto config = std::unique_ptr<nvinfer1::IBuilderConfig>(builder->createBuilderConfig());
        config->setFlag(nvinfer1::BuilderFlag::kFP16);
        nvinfer1::IOptimizationProfile* profile = builder ->createOptimizationProfile();
        profile->setDimensions("input", nvinfer1::OptProfileSelector::kMIN, nvinfer1::Dims4{1, 3, 1080, 1920});
        profile->setDimensions("input", nvinfer1::OptProfileSelector::kOPT, nvinfer1::Dims4{1, 3, 1080, 1920});
        profile->setDimensions("input", nvinfer1::OptProfileSelector::kMAX, nvinfer1::Dims4{1, 3, 1080, 1920});
        config->addOptimizationProfile(profile);
        auto serializedModel = std::unique_ptr<nvinfer1::IHostMemory>(builder->buildSerializedNetwork(*network, *config));

        std::ofstream trtFile("your_model_fp16.trt", std::ios::binary);
        trtFile.write(static_cast<const char*>(serializedModel->data()), serializedModel->size());
        trtFile.close();


    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
