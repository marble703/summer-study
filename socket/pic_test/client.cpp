
#include <opencv2/opencv.hpp>
#include <iostream>
#include <sys/types.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <vector>
#include "MessageBuffer.hpp"
#include "Message.hpp"


const char* serverIP = "10.2.20.66";

int main(){
    // 创建socket和初始化地址，端口
    int clientSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (clientSocket == -1) {
        std::cerr << "Failed to create clientSocket." << std::endl;
        return -1;
    }
    int camclientSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (camclientSocket == -1) {
        std::cerr << "Failed to create camclientSocket." << std::endl;
        return -1;
    }
    int transformSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (transformSocket == -1) {
        std::cerr << "Failed to create transformSocket." << std::endl;
        return -1;
    }
    int reportSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (reportSocket == -1) {
        std::cerr << "Failed to create reportSocket." << std::endl;
        return -1;
    }

    sockaddr_in serverAddress{};
    sockaddr_in camseverAddress{};
    sockaddr_in transformAddress{};
    sockaddr_in reportAddress{};

    serverAddress.sin_family = AF_INET;
    camseverAddress.sin_family = AF_INET;
    transformAddress.sin_family = AF_INET;
    reportAddress.sin_family = AF_INET;

    serverAddress.sin_port = htons(8000);
    camseverAddress.sin_port = htons(5140);
    transformAddress.sin_port = htons(4399);
    reportAddress.sin_port = htons(8003);

    // 转换IP地址
    if(inet_pton(AF_INET, serverIP, &serverAddress.sin_addr) <= 0) {
        std::cerr << "Invalid serverAddress/ ServerAddress not supported" << std::endl;
        close(clientSocket);
        return -1;
    }

    if(inet_pton(AF_INET, serverIP, &camseverAddress.sin_addr) <= 0) {
        std::cerr << "Invalid camseverAddress/ CamseverAddress not supported" << std::endl;
        close(camclientSocket);
        return -1;
    }

    if(inet_pton(AF_INET, serverIP, &transformAddress.sin_addr) <= 0) {
        std::cerr << "Invalid transformAddress/ TransformAddress not supported" << std::endl;
        close(transformSocket);
        return -1;
    }

    if(inet_pton(AF_INET, serverIP, &reportAddress.sin_addr) <= 0) {
        std::cerr << "Invalid reportAddress/ ReportAddress not supported" << std::endl;
        close(reportSocket);
        return -1;
    }

    Information information;

    // 连接到服务器,读取相机信息
    if (connect(camclientSocket, (struct sockaddr*)&camseverAddress, sizeof(camseverAddress)) < 0) {
        std::cerr << "Connection failed." << std::endl;
        close(camclientSocket);
        return -1;
    }
    else{
        std::cout << "Connected to caminfo!" << std::endl;

        std::vector<unsigned char> completeMessageBuffer;
        DecodeMessage decodeMessage;
        

        double CameraMatrix[9];
        double DistortionCoefficients[5];

        while (true){
            unsigned char recvBuffer[10240] = {0};
            int bytesRead = recv(camclientSocket, recvBuffer, sizeof(recvBuffer) - completeMessageBuffer.size(), 0);
            // 没连上
            if(bytesRead == -1){
                std::cerr << "Failed to receive data. errno: " << errno << std::endl;
                break;
            }
            // 断了
            if(bytesRead == 0){
                std::cout << "Connection closed by the server." << std::endl;
                break;
            }

            completeMessageBuffer.insert(completeMessageBuffer.end(), recvBuffer, recvBuffer + bytesRead);

            // 检查是否接受完整的消息
            while(completeMessageBuffer.size() >= sizeof(MessageBuffer)){
                MessageBuffer receivedMessage;
                unsigned char* buffer = completeMessageBuffer.data();
                classifyMessage(buffer, receivedMessage);

                int messageSize = sizeof(MessageBuffer) - sizeof(receivedMessage.Data) + receivedMessage.DataLength;
                if (completeMessageBuffer.size() < messageSize) {
                    break;
                }
                completeMessageBuffer.clear();
                // 处理相机内参消息
                if (receivedMessage.MessageType == CAMERA_INFO){
                    CameraInfoData cameraInfoData;
                    unsigned char* data = decodeMessage.handle(receivedMessage);
                    if (data != nullptr){
                        memcpy(&cameraInfoData, data, sizeof(CameraInfoData));
                        delete[] data;
                    }
                    memcpy(CameraMatrix, cameraInfoData.CameraMaterix, sizeof(CameraMatrix));
                    memcpy(DistortionCoefficients, cameraInfoData.DistortionCoefficients, sizeof(DistortionCoefficients));

                    cv::Mat_<double> cameraMatrix(3, 3);
                    cv::Mat_<double> distCoeffs(5, 1);
                    for (int i = 0; i < 3; i++){
                        for (int j = 0; j < 3; j++){
                            std::cout << CameraMatrix[i * 3 + j] << std::endl;
                            cameraMatrix(i, j) = CameraMatrix[i * 3 + j];
                            }
                    }
                    information.set_cameraMatrix(cameraMatrix);

                    for (int i = 0; i < 5; i++){
                        std::cout<<DistortionCoefficients[i]<<std::endl;
                        distCoeffs(i) = DistortionCoefficients[i];
                    }

                    information.set_distCoeffs(distCoeffs);

                    std::cout<<"cameramatrix: "<<information.cameraMatrix<<std::endl;
                    std::cout<<"distCoeffs: "<<information.distCoeffs<<std::endl;

                    close(camclientSocket);
                }
            }
        }
    }

    // 连接到服务器，读取变换信息
    if (connect(transformSocket, (struct sockaddr*)&transformAddress, sizeof(transformAddress)) < 0) {
        std::cerr << "Connection failed." << std::endl;
        close(transformSocket);
        return 1;
    }
    else{
        std::cout<<"Connected to transform."<<std::endl;
        MessageBuffer sendBuffer;
        TransformData transformData;
        sendBuffer.MessageType = TRANSFORM_REQUEST;
        sendBuffer.Data[10218] = {0};
        sendBuffer.Start = 0x0D00;
        sendBuffer.DataLength = sizeof(TransformRequestData);
    }

    // 连接到服务器，读取视频流
    if (connect(clientSocket, (struct sockaddr*)&serverAddress, sizeof(serverAddress)) < 0) {
        std::cout << "Connection Failed. errno: " << errno << std::endl;
        close(clientSocket);
        return -1;
    }
    else{
        std::cout << "Connected to the server!" << std::endl;
    }

    std::vector<unsigned char> completeMessageBuffer;

    DecodeMessage decodeMessage;

    while (true){
        unsigned char recvBuffer[10240] = {0};
        int bytesRead = recv(clientSocket, recvBuffer, sizeof(recvBuffer) - completeMessageBuffer.size(), 0);
        if(bytesRead == -1){
            std::cerr << "Failed to receive data. errno: " << errno << std::endl;
            return -1;
        }
        if(bytesRead == 0){
            std::cout << "Connection closed by the server." << std::endl;
            break;
        }
        std::cout << "Received package." << std::endl;
        completeMessageBuffer.insert(completeMessageBuffer.end(), recvBuffer, recvBuffer + bytesRead);

        // 检查是否接受完整的消息
        while (completeMessageBuffer.size() >= sizeof(MessageBuffer)){
            MessageBuffer receivedMessage;
            unsigned char* buffer = completeMessageBuffer.data();
            classifyMessage(buffer, receivedMessage);

            // 如果没接受完，继续等下一个包
            int messageSize = sizeof(MessageBuffer) - sizeof(receivedMessage.Data) + receivedMessage.DataLength;
            if (completeMessageBuffer.size() < messageSize) {
                break;
            }
            // 接受完了，清空缓存区
            completeMessageBuffer.clear();

            // 处理字符串消息
            if (receivedMessage.MessageType == STRING_MSG){
                std::string str_msg = decodeMessage.decode_string_msg(receivedMessage);
            }

            if (receivedMessage.MessageType == IMAGE_MSG){
                cv::Mat img = decodeMessage.decode_image_msg(receivedMessage);
                if (!img.empty()){
                    cv::imshow("Image", img);
                    information.image = img;
                    cv::waitKey(1);
                }
            }
        }

    }

    return 0;
}