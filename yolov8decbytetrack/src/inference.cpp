// Ultralytics 🚀 AGPL-3.0 License - https://ultralytics.com/license

#include "inference.h"
#include <opencv4/opencv2/core/types.hpp>

Inference::Inference(const std::string &onnxModelPath, const cv::Size &modelInputShape, const std::vector<std::string> &classes_, const bool &runWithCuda)
{
    modelPath = onnxModelPath;
    modelShape = modelInputShape;
    classes = classes_;
    cudaEnabled = runWithCuda;

    loadOnnxNetwork();
    // loadClassesFromFile(); The classes are hard-coded for this example
}

Inference_trt::Inference_trt(const std::string &enginePath,const cv::Size &modelInputShape, const std::vector<std::string> &classes_, const bool &runWithCuda)
{
    modelPath = enginePath;
    modelShape = modelInputShape;
    classes = classes_;
    cudaEnabled = runWithCuda;

    loadTensorRTEngine(enginePath);
    //cudaStreamCreate(&stream);
}

static void print_dims(const nvinfer1::Dims& d)
{
    std::cout << "Dims.nbDims=" << d.nbDims << "[";
    for (int i = 0; i < d.nbDims; i++)
    {
        std::cout << d.d[i];
        if (i<d.nbDims-1) std::cout << ",";
    }
    std::cout << "]" << std::endl;
}

std::vector<Detection> Inference::runInference(const cv::Mat &input)
{
    cv::Mat modelInput = input;
    int pad_x, pad_y;
    float scale;
    if (letterBoxForSquare && modelShape.width == modelShape.height)
        modelInput = formatToSquare(modelInput, &pad_x, &pad_y, &scale);

    cv::Mat blob;
    cv::dnn::blobFromImage(modelInput, blob, 1.0/255.0, modelShape, cv::Scalar(), true, false);
    net.setInput(blob);

    std::vector<cv::Mat> outputs;
    net.forward(outputs, net.getUnconnectedOutLayersNames());

    int rows = outputs[0].size[1];
    int dimensions = outputs[0].size[2];

    bool yolov8 = false;
    // yolov5 has an output of shape (batchSize, 25200, 85) (Num classes + box[x,y,w,h] + confidence[c])
    // yolov8 has an output of shape (batchSize, 84,  8400) (Num classes + box[x,y,w,h])
    if (dimensions > rows) // Check if the shape[2] is more than shape[1] (yolov8)
    {
        yolov8 = true;
        rows = outputs[0].size[2];
        dimensions = outputs[0].size[1];

        outputs[0] = outputs[0].reshape(1, dimensions);
        cv::transpose(outputs[0], outputs[0]);
    }
    float *data = (float *)outputs[0].data;

    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    for (int i = 0; i < rows; ++i)
    {
        if (yolov8)
        {
            float *classes_scores = data+4;
            cv::Mat scores(1, classes.size(), CV_32FC1, classes_scores);
            cv::Point class_id;
            double maxClassScore;

            minMaxLoc(scores, 0, &maxClassScore, 0, &class_id);

            if (maxClassScore > modelScoreThreshold)
            {
                confidences.push_back(maxClassScore);
                class_ids.push_back(class_id.x);

                float x = data[0];
                float y = data[1];
                float w = data[2];
                float h = data[3];

                int left = int((x - 0.5 * w - pad_x) / scale);
                int top = int((y - 0.5 * h - pad_y) / scale);

                int width = int(w / scale);
                int height = int(h / scale);

                boxes.push_back(cv::Rect(left, top, width, height));
            }
            //std::cout << "is v8" << std::endl;
        }
        else // yolov5
        {
            float confidence = data[4];

            if (confidence >= modelConfidenceThreshold)
            {
                float *classes_scores = data+5;

                cv::Mat scores(1, classes.size(), CV_32FC1, classes_scores);
                cv::Point class_id;
                double max_class_score;

                minMaxLoc(scores, 0, &max_class_score, 0, &class_id);

                if (max_class_score > modelScoreThreshold)
                {
                    confidences.push_back(confidence);
                    class_ids.push_back(class_id.x);

                    float x = data[0];
                    float y = data[1];
                    float w = data[2];
                    float h = data[3];

                    int left = int((x - 0.5 * w - pad_x) / scale);
                    int top = int((y - 0.5 * h - pad_y) / scale);

                    int width = int(w / scale);
                    int height = int(h / scale);

                    boxes.push_back(cv::Rect(left, top, width, height));
                }
            }
            //std::cout << "is v5" << std::endl;
        }

        data += dimensions;
    }

    std::vector<int> nms_result;
    cv::dnn::NMSBoxes(boxes, confidences, modelScoreThreshold, modelNMSThreshold, nms_result);

    std::vector<Detection> detections{};
    for (unsigned long i = 0; i < nms_result.size(); ++i)
    {
        int idx = nms_result[i];

        Detection result;
        result.class_id = class_ids[idx];
        result.confidence = confidences[idx];

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<int> dis(100, 255);
        result.color = cv::Scalar(dis(gen),
                                  dis(gen),
                                  dis(gen));

        result.className = classes[result.class_id];
        result.box = boxes[idx];

        detections.push_back(result);
    }

    return detections;
}

size_t Inference_trt::getSizeByDims(const nvinfer1::Dims& dims) {
    size_t size = 1;
    for (int i = 0; i < dims.nbDims; ++i) {
        size *= dims.d[i];
    }
    return size;
}

size_t Inference_trt::getElementSize(nvinfer1::DataType t) {
    switch (t) {
        case nvinfer1::DataType::kFLOAT: return 4;
        case nvinfer1::DataType::kHALF: return 2;
        case nvinfer1::DataType::kINT8: return 1;
        case nvinfer1::DataType::kINT32: return 4;
        case nvinfer1::DataType::kBOOL: return 1;
        default: throw std::runtime_error("Invalid DataType");
    }
}

void Inference_trt::loadTensorRTEngine(const std::string &enginePath)
{
    runtime = std::shared_ptr<nvinfer1::IRuntime>(nvinfer1::createInferRuntime(gLogger));
    std::ifstream engineFile(enginePath, std::ios::binary);
    if (!engineFile.good())
    {
        throw std::runtime_error("TensorRT engine file not found");
    }

    engineFile.seekg(0, std::ios::end);
    size_t modelsize = engineFile.tellg();
    engineFile.seekg(0, std::ios::beg);
    std::vector<char> modelData(modelsize);
    engineFile.read(modelData.data(), modelsize);
    engineFile.close();

     // 创建runtime
    runtime = std::shared_ptr<nvinfer1::IRuntime>(
        nvinfer1::createInferRuntime(gLogger)
    );
    
    if (!runtime) {
        throw std::runtime_error("Failed to create TensorRT runtime");
    }
    
    // 反序列化引擎 - TensorRT 10.x版本只需要2个参数
    trtEngine = std::shared_ptr<nvinfer1::ICudaEngine>(
        runtime->deserializeCudaEngine(modelData.data(), modelsize)
    );
    
    if (!trtEngine) {
        throw std::runtime_error("Failed to deserialize CUDA engine");
    }
    
    // 创建执行上下文
    trtContext = std::shared_ptr<nvinfer1::IExecutionContext>(
        trtEngine->createExecutionContext()
    );
    
    if (!trtContext) {
        throw std::runtime_error("Failed to create execution context");
    }
    
     // 获取IO tensor数量
    int numIOTensors = trtEngine->getNbIOTensors();
    deviceBuffers.resize(numIOTensors);
    
    // 查找输入输出tensor
    for (int i = 0; i < numIOTensors; ++i) {
        const char* tensorName = trtEngine->getIOTensorName(i);
        nvinfer1::TensorIOMode ioMode = trtEngine->getTensorIOMode(tensorName);
        
        if (ioMode == nvinfer1::TensorIOMode::kINPUT) {
            inputIndex = i;
            inputTensorName = tensorName;
            std::cout << "Input tensor: " << inputTensorName << std::endl;
        } else if (ioMode == nvinfer1::TensorIOMode::kOUTPUT) {
            outputIndex = i;
            outputTensorName = tensorName;
            std::cout << "Output tensor: " << outputTensorName << std::endl;
        }
    }
    
    // 获取输入输出维度
    nvinfer1::Dims inputDims = trtEngine->getTensorShape(inputTensorName.c_str());
    nvinfer1::Dims outputDims = trtEngine->getTensorShape(outputTensorName.c_str());
    std::cout << "input dims:" ;print_dims(inputDims);
    std::cout <<"output dims:" ;print_dims(outputDims);
    
    // 获取数据类型
    nvinfer1::DataType inputDataType = trtEngine->getTensorDataType(inputTensorName.c_str());
    nvinfer1::DataType outputDataType = trtEngine->getTensorDataType(outputTensorName.c_str());
    
    // 计算缓冲区大小
    inputSize = getSizeByDims(inputDims) * getElementSize(inputDataType);
    outputSize = getSizeByDims(outputDims) * getElementSize(outputDataType);
    
    // 分配设备内存
    cudaMalloc(&deviceBuffers[inputIndex], inputSize);
    cudaMalloc(&deviceBuffers[outputIndex], outputSize);

    buffers = deviceBuffers.data();
    
    // 设置执行上下文的tensor地址
    trtContext->setInputTensorAddress(inputTensorName.c_str(), deviceBuffers[inputIndex]);
    trtContext->setOutputTensorAddress(outputTensorName.c_str(), deviceBuffers[outputIndex]);
    
    std::cout << "TensorRT engine loaded successfully" << std::endl;
    std::cout << "" << std::endl;
    std::cout << "Input tensor: " << inputTensorName << ", size: " << inputSize << " bytes" << std::endl;
    std::cout << "Output tensor: " << outputTensorName << ", size: " << outputSize << " bytes" << std::endl;
    cudaStreamCreate(&cudaStream);

}

std::vector<Detection> Inference_trt::runInference_TensorRT(const cv::Mat &input)
{

    if (!trtContext) 
    {
        throw std::runtime_error("TensorRT engine not loaded");
    }
    cv::Mat inputBlob;
    cv::dnn::blobFromImage(input, inputBlob, 1.0/255.0, modelShape, cv::Scalar(), true, false);
    size_t blob_bytes = inputBlob.total() * inputBlob.elemSize();
    if (blob_bytes > inputSize)
    {
        std::cerr << "Warning: input blob size (" << blob_bytes << ") > expected inputSize (" << inputSize << "). Using min size to copy.\n";
    }
    size_t copy_bytes = std::min(blob_bytes, inputSize);

    std::vector<float> inputData(inputSize / sizeof(float), 0.0f);
    memcpy(inputData.data(), inputBlob.data, copy_bytes);

    cudaError_t err_mem1 = cudaMemcpyAsync(deviceBuffers[inputIndex], inputData.data(), inputSize, cudaMemcpyHostToDevice, cudaStream);
    if(err_mem1 != cudaSuccess)
    {
        std::cerr << "cudamem failed" << cudaGetErrorString(err_mem1) << std::endl;
        throw std::runtime_error("cudamem failed");
    }

    if (!trtContext->enqueueV3(cudaStream)) {
        throw std::runtime_error("Failed to execute inference");
    }

    std::vector<float> outputData(outputSize / sizeof(float));
    cudaError_t err_mem2 = cudaMemcpyAsync(outputData.data(), deviceBuffers[outputIndex], outputSize, cudaMemcpyDeviceToHost, cudaStream);
    if(err_mem2 != cudaSuccess)
    {
        std::cerr << "cudamem failed" << cudaGetErrorString(err_mem2) << std::endl;
        throw std::runtime_error("cudamem failed");
    }
    cudaError_t err_stream = cudaStreamSynchronize(cudaStream);
    if (err_stream != cudaSuccess)
    {
        std::cerr << "cudastream failed" << cudaGetErrorString(err_stream) << std::endl;
        throw std::runtime_error("cudastream failed");
    }
    //nvinfer1::Dims outputDims = trtEngine->getTensorShape(outputTensorName.c_str());
    //float* output = static_cast<float*>(buffers[outputIndex]);
    float* output = outputData.data();
    std::vector<Detection> detections{};
    
    // 在loadTensorRTEngine函数中添加
    nvinfer1::Dims outputDims = trtEngine->getTensorShape(outputTensorName.c_str());
    print_dims(outputDims);

    size_t total_elems = outputSize / sizeof(float);
    int inferred_dims = 0;
    int inferred_rows = 0;
    bool has_objectness = false;
    int num_classes_known = static_cast<int>(classes.size());

    if (num_classes_known > 0 )
    {
        int cand1 = num_classes_known + 4;
        int cand2 = num_classes_known + 5;
        if (cand1 > 0 && (total_elems % cand1) == 0)
        {
            inferred_dims = cand1;
            inferred_rows = static_cast<int>(total_elems / inferred_dims);
            has_objectness = false;
        }
        else if (cand2 > 0 && (total_elems & cand2) == 0)
        {
            inferred_dims = cand2;
            inferred_rows = static_cast<int>(total_elems / inferred_dims);
            has_objectness = true;
        }
    }

    for (int i = 0; i< inferred_rows; i++)
    {
        float* ptr = output + i * inferred_dims;

        if(has_objectness)
        {
            float objectness = ptr[4];
            float* classes_scores = ptr + 5;
            int classes_count = inferred_dims - 5;
            if(classes_count <= 0)
            {
                continue;
            }
            cv::Mat scores(1,classes_count, CV_32FC1, classes_scores);
            cv::Point class_id;
            double maxClassScore;
            minMaxLoc(scores, 0, &maxClassScore, 0, &class_id);

            if(objectness >= modelConfidenceThreshold && maxClassScore > modelScoreThreshold)
            {
                Detection det;
                det.class_id = class_id.x;
                det.confidence = static_cast<float>(objectness * maxClassScore);

                float cx = ptr[0];
                float cy = ptr[1];
                float w = ptr[2];
                float h = ptr[3];

                det.box = cv::Rect(static_cast<int>(cx - w/2.0f), static_cast<int>(cy - h/2.0f), static_cast<int>(w), static_cast<int>(h));
                if (det.class_id >=0 && det.class_id < static_cast<int>(classes.size()))
                {
                    det.className = classes[det.class_id];
                }
                detections.push_back(det);
            }
            else 
            {
                {
                    float* classes_scores = ptr + 4;
                    int classes_count = inferred_dims - 4;
                    if (classes_count <= 0 )
                    {
                        continue;
                    }
                    cv::Mat scores(1, classes_count, CV_32FC1, classes_scores);
                    cv::Point class_id;
                    double maxClassScore;
                    minMaxLoc(scores, 0, &maxClassScore, 0, &class_id);

                    if (maxClassScore > modelScoreThreshold)
                    {
                        Detection det;
                        det.class_id = class_id.x;
                        det.confidence = static_cast<float>(maxClassScore);

                        float cx = ptr[0];
                        float cy = ptr[1];
                        float w = ptr[2];
                        float h = ptr[3];

                        det.box = cv::Rect(static_cast<int>(cx - w/2.0f), static_cast<int>(cy - h/2.0f), static_cast<int>(w), static_cast<int>(h));
                        if (det.class_id >=0 && det.class_id < static_cast<int>(classes.size()))
                        {
                            det.className = classes[det.class_id];
                        }
                        detections.push_back(det);
                    }
                }
            }
        }
    }
    //if (outputDims.nbDims == 3 && outputDims.d[1] > 4) 
    //{
    //    numClasses = outputDims.d[1] - 4;
    //    std::cout << "Detected " << numClasses << " classes from model output" << std::endl;
    //} 
    //else 
    //{
    //    numClasses = 80;
    //    std::cout << "Using default " << numClasses << " classes" << std::endl;
    //}

    //int numAnchors = outputDims.d[2]; // 8400 for YOLOv8
    
    //for (int i = 0; i < numAnchors; ++i) {
    //    float* ptr = output + i * (4 + numClasses);
        
        // 找到最大置信度的类别
    //    int classId = std::max_element(ptr + 4, ptr + 4 + numClasses) - (ptr + 4);
    //    float confidence = ptr[4 + classId];
        
    //    if (confidence > 0.5) { // 置信度阈值
    //        Detection det;
    //        det.class_id = classId;
    //        det.confidence = confidence;
            
            // 解析边界框 (cx, cy, w, h)
    //        float cx = ptr[0];
    //        float cy = ptr[1];
    //        float w = ptr[2];
    //        float h = ptr[3];
            
    //        det.box = cv::Rect(cx - w/2, cy - h/2, w, h);
    //        detections.push_back(det);
    //    }
    //}


    
    return detections;
}



void Inference::loadClassesFromFile()
{
    std::ifstream inputFile(classesPath);
    if (inputFile.is_open())
    {
        std::string classLine;
        while (std::getline(inputFile, classLine))
            classes.push_back(classLine);
        inputFile.close();
    }
}

void Inference::loadOnnxNetwork()
{
    net = cv::dnn::readNetFromONNX(modelPath);
    if (cudaEnabled)
    {
        //std::cout << "\nRunning on CUDA" << std::endl;
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA_FP16);
    }
    else
    {
        //std::cout << "\nRunning on CPU" << std::endl;
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    }
}

cv::Mat Inference::formatToSquare(const cv::Mat &source, int *pad_x, int *pad_y, float *scale)
{
    int col = source.cols;
    int row = source.rows;
    int m_inputWidth = modelShape.width;
    int m_inputHeight = modelShape.height;

    *scale = std::min(m_inputWidth / (float)col, m_inputHeight / (float)row);
    int resized_w = col * *scale;
    int resized_h = row * *scale;
    *pad_x = (m_inputWidth - resized_w) / 2;
    *pad_y = (m_inputHeight - resized_h) / 2;

    cv::Mat resized;
    cv::resize(source, resized, cv::Size(resized_w, resized_h));
    cv::Mat result = cv::Mat::zeros(m_inputHeight, m_inputWidth, source.type());
    resized.copyTo(result(cv::Rect(*pad_x, *pad_y, resized_w, resized_h)));
    resized.release();
    return result;
}