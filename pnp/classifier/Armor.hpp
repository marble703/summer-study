#pragma once

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

struct Light{
    cv::Point2f bottom;
    cv::Point2f top;
};

class ArmorType{
public:
    enum Type {
        SMALL,
        LARGE
    };
};


class Armor {
private:

public:
    cv::Mat image;
    cv::Mat number_image;
    std::string classification_result;
    std::string number;

    enum ArmorType {
        SMALL,
        LARGE
    } type;

    Light left_light;
    Light right_light;

    double classification_confidence;

    Armor() = default;
    ~Armor() = default;

    Armor(cv::Mat image)

    {
    type = ArmorType::SMALL;

        cv::Mat gray, binary;
        //灰度化
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        //二值化
        cv::threshold(gray, binary, 160, 255, cv::THRESH_BINARY); 
        //定义核
        int x_erode_kernal_size = 3;
        int y_erode_kernal_size = 3;
        cv::Mat erode_kernal = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(x_erode_kernal_size, y_erode_kernal_size));

        int x_dilate_kernal_size = 7;
        int y_dilate_kernal_size = 7;
        cv::Mat dilate_kernal = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(x_dilate_kernal_size, y_dilate_kernal_size));

        //腐蚀
        cv::Mat eroded_image;
        cv::erode(binary, eroded_image, erode_kernal);

        //膨胀
        cv::Mat dilated_image;
        cv::dilate(eroded_image, dilated_image, dilate_kernal);
        
        // 查找轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(eroded_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        int flag;
        for (size_t i = 0; i < contours.size(); i++) {
            // 对每个轮廓计算最小外接旋转矩形
            cv::RotatedRect minRect = cv::minAreaRect(contours[i]);
            
            // 获取矩形的宽度和高度
            float width = minRect.size.width;
            float height = minRect.size.height;
            
            // 判断是长条方向
            if (width > height) {
                flag = 1;
                // 绘制旋转矩形
                cv::Point2f rectPoints[4];
                minRect.points(rectPoints);
                for (int j = 0; j < 4; j++) {
                    cv::line(image, rectPoints[j], rectPoints[(j+1)%4], cv::Scalar(0,255,0), 2);
                }
            }
            else{
                flag = 0;
            }
        }

        // 使用两个外接矩形的短边中点作为边界点
        std::vector<cv::Point> points;
        for (size_t i = 0; i < contours.size(); i++) {
            cv::RotatedRect minRect = cv::minAreaRect(contours[i]);
            cv::Point2f rectPoints[4];
            minRect.points(rectPoints);

            cv::Point2f p1 = (rectPoints[0] + rectPoints[1]) / 2;
            cv::Point2f p2 = (rectPoints[2] + rectPoints[3]) / 2;

            //std::cout << flag << std::endl;
            if(flag == 1){
                p1 = (rectPoints[0] + rectPoints[1]) / 2;
                p2 = (rectPoints[2] + rectPoints[3]) / 2;
            }
            if(flag == 0){
                p1 = (rectPoints[1] + rectPoints[2]) / 2;
                p2 = (rectPoints[0] + rectPoints[3]) / 2;}

            points.push_back(p1);
            points.push_back(p2);
        }

        // 目标边缘的四个顶点坐标
        std::vector<cv::Point2f> srcPoints = {
            points[0], 
            points[2], 
            points[3], 
            points[1]
        };

        if(points[0].x < points[2].x){
            cv::Point2f point = points[0];
            points[0] = points[2];
            points[2] = point;

            point = points[1];
            points[1] = points[3];
            points[3] = point;
        }

        // 左右灯条的四个顶点坐标
        left_light.bottom = points[3];
        left_light.top = points[2];
        right_light.top = points[0];
        right_light.bottom = points[1];

        std::cout<<left_light.bottom<<std::endl;
        std::cout<<left_light.top<<std::endl;
        std::cout<<right_light.top<<std::endl;
        std::cout<<right_light.bottom<<std::endl;


        number_image = image;
    }



};
