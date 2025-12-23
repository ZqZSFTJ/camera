#include <iostream>
#include <opencv4/opencv2/core/types.hpp>
#include <vector>
#include <getopt.h>
#include <tuple>
#include <functional>
#include <thread>
#include <algorithm>
#include <unordered_map>
#include <nlohmann/json.hpp>
#include <fstream>
#include <string>


#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/highgui.hpp>
#include <chrono>
#include "BYTETracker.h"
#include "HikCamera.h"
#include "config.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/image.hpp>
#include "geometry_msgs/msg/point.hpp"
#include "tutorial_interfaces/msg/detection.hpp"
#include "tutorial_interfaces/msg/target.hpp"

#include <NvOnnxParser.h>
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <cuda.h>


class Logger : public nvinfer1::ILogger
{
    void log(nvinfer1::ILogger::Severity severity, const char* msg) noexcept override
    {
        // suppress info-level messages
        if (severity <= nvinfer1::ILogger::Severity::kWARNING)
        {
            std::cout << "[TensorRt]" << msg << std::endl;
        }
    }
};
static Logger gLogger;

nvinfer1::ICudaEngine* load_engine(const std::string& engine_path,nvinfer1::IRuntime*& runtime)
{
    std::ifstream file(enginePath, std::ios::binary);
    if (!file) {
        return nullptr;
    }

    file.seekg(0, std::ios::end);
    size_t size = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<char> buffer(size);
    file.read(buffer.data(), size);
    file.close();

    runtime = nvinfer1::createInferRuntime(gLogger);
    return rutime->deserializeCudaEngine(buffer.data(), size);
}    

size_t volume(const nvinfer1::Dims& dims)
{
    size_t vol = 1;
    for (int i = 0; i < dims.nbDims; i++)
    {
        vol *= dims.d[i];
    }
    return vol;
}

size_t element_size(nvifer1::DataType type)
{
    if (type == nvifer1::DataType::kFLOAT)
    {
        return 4;
    }
    else if (type == nvifer1::DataType::kHALF)
    {
        return 2;
    }
    else if (type == nvifer1::DataType::kINT8)
    {
        return 1;
    }
    else if (type == nvifer1::DataType::kINT32)
    {
        return 4;
    }
    throw std::runtime_error("Unknown data type");
}



void inference(std::string &engine_path, std::string &video_path, cv::Mat &frame)
{
    cudaStream_t stream;
    nvinfer1::IRuntime* runtime{ nullptr };
    nvinfer1::ICudaEngine* engine = load_engine(std::string &engine_path, int *&runtime)//""处填入.engine文件路径
    if (!engine)
    {
        throw std::runtime_error("Failed to load Engine");
    }
    nvinfer1::IExecutionContext* context = engine->createExecuteContext();
    if (!context)
    {
        throw std::runtime_error("Failed to create execution context")
    }

    int io_count = engine->getNbIOTensors();//input与output的总数量
    std::string input_name;
    for (int i = 0; i < io_count; i++)
    {
        const char* tensor_name = engine->getIOTensorName(i);
        //tensor 名字
        if (engine->getTensorIOMode(tensor_name) == nvifer1::TensorIOMode::kINPUT)
        {
            input_name = tensor_name;
        }
        nvifer1::TensorIOMode mode = engine->getTensorIOMode(tensor_name);
        //确认是输入还是输出:kINPUT/kOUTPUT
        nvifer1::DataType data_type = engine->getTensorDataType(tensor_name);
        //tensor的数据类型:float32/float16/int8等
        nvifer1::Dims dims = engine->getTensorShape(tensor_name);
        //tensor的维度信息
        std::cout << "Tensor Name: " << tensor_name << std::endl;
        std::cout << "Tensor Mode: " << (mode == nvifer1::TensorIOMode::kINPUT ? "Input" : "Output") << std::endl;
        std::cout << "Tensor Data Type: " << static_cast<int>(data_type) << std::endl;
        std::cout << "Tensor Dims: ";
        for (int j = 0; j < dims.nbDims; j++)
        {
            std::cout << dims.d[j] << " ";
        }
        std::cout << std::endl;
    }//调试信息，打印所有输入输出tensor的信息

    nvifer1::Dims4 input_shape;
    input_shape.d[0] = 1; //batch size
    input_shape.d[1] = 3; //channels
    input_shape.d[2] = 640; //height
    input_shape.d[3] = 640; //width

    bool ok = context->setInputShape(intput_name.c_str(), input_shape);
    //设置输入tensor的形状
    if (!ok)
    {
        throw std::runtime_error("Failed to set input shape");
    }

    std::unordered_map<std::string, void*> buffers;
    std::unordered_map<std::string, size_t> sizes;
    for (int i = 0; i < io_count; i++)
    {
        const char* name = engine->getIOTensorName(i);
        nvifer1::Dims dims = context->getTensorShape(name);
        nvifer1::DataType data_type = engine->getTensorDataType(name);
        size_t bytes = volume(dims) * elemen_size(type);
        void* device_ptr = nullptr;
        CUDA_CHECK(cudaMalloc(device_ptr, bytes));
        context->setTensorAddress(name, device_ptr);

        buffers[name] = device_ptr;
        sizes[name] = bytes;
    }
    
    CUDA_CHECK(cudaStreamCreate(&stream));
    cv::Mat frame2;
    cv::Mat resized;
    std::vector<float> input_host(1 * 3 * 640 * 640);
    while(1)
    {
        cv::resize(frame, resized, cv::Size(640,640));
        //预处理：调整大小，归一化等
        resized.convertTo(resized, CV_32F, 1.0 / 255);
        
        int idx = 0;
        for(int c = 0; c < 3; c++)
        {
            for(int h = 0; h < 640; h++)
            {
                for(int w = 0; w < 640; w++)
                {
                    input_host[idx++] = resized.at<cv::Vec3f>(h,w)[c];
                }
            }
        }

        CUDA_CHECK(cudaMemcpyAsync(buffers[input_name], input_host.data(), sizes[input_name], cudaMemcpyHostToDevice, stream));
        context->enqueueV3(stream);
        //处理输出
        CUDA_CHECK(cudaStreamSynchronize(stream));
        // NMS




        //
    }
}














