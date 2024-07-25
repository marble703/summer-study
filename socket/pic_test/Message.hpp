#pragma once

#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <unordered_map>
#include "MessageBuffer.hpp"

class DecodeMessage{
private:
    // 存储数据
    std::unordered_map<unsigned int, std::vector<unsigned char>> data_temp;

    /**
     * @brief 解码数据
     * @param message 数据结构体
     * @return 解码后的数据
     */
    unsigned char* decode(MessageBuffer& message){
        unsigned int offset = message.Offset;
        unsigned int length = message.DataLength;
        unsigned int total_length = message.DataTotalLength;
        unsigned int dataID = message.DataID;

        // 如果没有该数据ID，则创建一个并分配内存
        if (data_temp.find(dataID) == data_temp.end()) {
            data_temp[dataID] = std::vector<unsigned char>(total_length);
        }
        // 将数据拷贝到对应的数据ID中
        std::memcpy(data_temp[dataID].data() + offset, message.Data, length);

        // 如果数据长度达到总长度，则返回数据
        if (offset + length >= total_length) {
            unsigned char* data = new unsigned char[total_length];
            std::memcpy(data, data_temp[dataID].data(), total_length);
            data_temp.erase(dataID);
            return data;
        } 
        // 否则返回空指针
        else {
            return nullptr;
        }
    }

public:
    /**
     * @brief 处理消息
     * @param buffer 数据缓存区
     * @param message 数据结构体
     * @return 解码后的数据
     */
    unsigned char* handle(MessageBuffer& buffer){
        return decode(buffer);
    }

    /**
     * @brief 释放内存
     * @param data 数据
     */
    void release(unsigned char* data){
        delete[] data;
    }

    // 处理字符串
    std::string decode_string_msg(MessageBuffer& buffer){
        unsigned char* data = decode(buffer);
        if (data != nullptr) {
            std::string str(reinterpret_cast<char*>(data), buffer.DataTotalLength);
            release(data);
            return str;
        }
        return "";
    }

    // 处理图片
    cv::Mat decode_image_msg(MessageBuffer& buffer){
        unsigned char* data = decode(buffer);
        if (data != nullptr) {
            std::vector<unsigned char> img_data(data, data + buffer.DataTotalLength);
            cv::Mat img = cv::imdecode(img_data, cv::IMREAD_COLOR);
            release(data);
            return img;
        }
        return cv::Mat();
    }
};

class Information{
public:
    Information () = default;
    ~Information() = default;

    cv::Mat_<double> cameraMatrix;
    cv::Mat_<double> distCoeffs;
    cv::Mat image;

    void set_cameraMatrix(cv::Mat_<double> cameraMatrix){
        this->cameraMatrix = cameraMatrix;
    }

    void set_distCoeffs(cv::Mat_<double> distCoeffs){
        this->distCoeffs = distCoeffs;
    }
};