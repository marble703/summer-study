#include <opencv2/opencv.hpp>
#include <iostream>
#include <sys/types.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <vector>

    typedef struct {
        unsigned short Start = 0x0D00; //起始位     0~1
        unsigned short MessageType;    //消息类型   2~3
        unsigned int DataID;           //数据ID     4~7
        unsigned int DataTotalLenth;   //数据总长度 8~11
        unsigned int Offset;           //偏移量     12~15
        unsigned int DataLenth;        //数据长度   16~19
        // 数据
        unsigned char Data[10218];     //数据       20~10237
        //数据尾部
        unsigned short End = 0x0721;    //数据尾部   10238~10239
    } MessageBuffer;

    enum MessageType {
        STRING_MSG = 0x0000,
        IMAGE_MSG = 0x1145,
        CAMERA_INFO = 0x1419,
        TRANSFORM = 0x1981,
        TRANSFORM_REQUEST = 0x1982
    };

    // MessageType 为 IMAGE_MSG 时的 Data 数据结构
    typedef struct{
        unsigned int Length;
        unsigned int Width;
        unsigned int Type;
        unsigned char ImageData[10218 - 12];
    } ImageData;

    // MessageType 为 CAMERA_INFO
    typedef struct{
        double CameraMaterix[9];
        double DistortionCoefficients[5];
    } CameraInfoData;

    // MessageType 为 Transform
    typedef struct{
        double Translation[3];
        double Roation[4];
    } TransformData;

    // MessageType 为 TRANSFORM_REQUEST
    typedef struct{
        char From[10218 / 2];
        char To[10218 / 2];
    }TransformRequestData;

// 处理数据
void processData(MessageType type, unsigned char* data) {
    switch (type) {
        case IMAGE_MSG: {
            // 处理图像数据
            ImageData* imgData = reinterpret_cast<ImageData*>(data);
            // 获取图像参数
            unsigned int width = imgData->Width;
            unsigned int height = imgData->Length / (width * 3);

            // 使用OpenCV解码图像数据
            cv::Mat frame(cv::Size(width, height), CV_8UC3, imgData->ImageData);
            cv::Mat decodedImage = cv::imdecode(frame, cv::IMREAD_COLOR);

            // 显示图片
            cv::imshow("decodedImage", decodedImage);

            //将接收到的图片保存进视频
            cv::VideoWriter writer("test.mp4", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, cv::Size(width, height));
            writer.write(decodedImage);
            cv::waitKey(1);

            break;
        }
        case CAMERA_INFO: {
            // 处理相机信息
            CameraInfoData* cameraInfo = reinterpret_cast<CameraInfoData*>(data);
            // 使用相机矩阵和畸变系数
            cv::Mat cameraMatrix = cv::Mat(3, 3, CV_64F, cameraInfo->CameraMaterix);
            cv::Mat distortionCoefficients = cv::Mat(1, 5, CV_64F, cameraInfo->DistortionCoefficients);
            // 使用cameraMatrix和distortionCoefficients进行相机校正等操作
        }
        case TRANSFORM: {
            // 处理变换数据
            TransformData* transform = reinterpret_cast<TransformData*>(data);
            // 应用变换
            break;
        }
        case TRANSFORM_REQUEST: {
            // 处理变换请求
            TransformRequestData* request = reinterpret_cast<TransformRequestData*>(data);
            // 处理请求
            break;
        }
        case STRING_MSG:{
            // 直接输出
            std::cout << "STRING_MSG: " << data << std::endl;
        }
        default:
            std::cout << "unknown" << std::endl;
            std::cout << data << std::endl; 
    }
}

int main() {
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd == -1) {
        std::cout << "Failed to create socket. errno: " << errno << std::endl;
        return -1;
    }

    sockaddr_in sockaddr;
    sockaddr.sin_family = AF_INET;
    sockaddr.sin_port = htons(8000); // 服务器监听的端口
    // 将IP地址从点分十进制转换为网络字节序
    if(inet_pton(AF_INET, "10.2.20.66", &sockaddr.sin_addr) <= 0) { // 服务器的IP地址
        std::cout << "Invalid address/ Address not supported" << std::endl;
        return -1;
    }

    // 尝试连接到服务器
    if (connect(sockfd, (struct sockaddr*)&sockaddr, sizeof(sockaddr)) < 0) {
        std::cout << "Connection Failed. errno: " << errno << std::endl;
        return -1;
    }

    std::cout << "Connected to the server!" << std::endl;

    cv::Mat image;
    unsigned int DataTotalLenth = 0;
    unsigned int DataLenth = 0;
    unsigned int DataLengthNow = 0;
    std::map<unsigned int, std::vector<unsigned char>> data_temp;

    while(1){
        // 获取数据，格式为MessageBuffer
        MessageBuffer buffer;
        int bytesRead = recv(sockfd, &buffer, sizeof(buffer), 0);

        if (bytesRead == 0) {
            std::cout << "Connection closed by the server." << std::endl;
            break; // 退出循环
        }
        if (bytesRead == -1) {
            std::cout << "Failed to receive data. errno: " << errno << std::endl;
            return -1;
        }

        // 检查起始位和结束位
        if (buffer.Start != 0x0D00 || buffer.End != 0x0721) {
            std::cout << "Invalid start or end marker" << std::endl;
            continue;
        }

        /*
        */

        // 处理接收到的数据
        unsigned int dataID = buffer.DataID;
        unsigned int total_lenth = buffer.DataTotalLenth;
        unsigned int offset = buffer.Offset;
        unsigned int lenth = buffer.DataLenth;
        unsigned char* data = buffer.Data;

        // 检查dataID是否已存在于data_temp中
        if (data_temp.find(dataID) == data_temp.end()) {
            data_temp[dataID] = std::vector<unsigned char>(total_lenth);
        }

        // 将接收到的数据片段追加到对应的vector中
        std::copy(data, data + lenth, data_temp[dataID].begin() + offset);

        // 检查是否已接收完当前dataID的所有数据片段
        if (offset + lenth == total_lenth) {
            // 处理完整的数据
            processData(static_cast<MessageType>(dataID), data_temp[dataID].data());

            // 从data_temp中移除该dataID的记录
            data_temp.erase(dataID);
        }

        // 清空buffer
        memset(&buffer, 0, sizeof(buffer));
    }
    // 使用完毕后关闭socket连接
    close(sockfd);

    return 0;
}
