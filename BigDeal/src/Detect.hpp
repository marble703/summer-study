#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include "Armor.hpp"
#include "number_classifier.hpp"
#include "process.hpp"

class Detect {
public:
    Detect(cv::Mat image) : image(image) {}

    int detect(bool wait = false){
    if (image.empty()) {
            std::cerr << "Failed to read image." << std::endl;
            return -1;
        }
        cv::Mat image_copy = image.clone();

        cv::Mat eroded_image = image_preprocess(image);

        // 查找轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(eroded_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        //std::cout << contours.size() << std::endl;

        int flag = 0;
        std::vector<int> flags;
        std::vector<double> angles;
        std::vector<double> areas;
        for (int i = 0; i < contours.size(); i++) {
            // 对每个轮廓计算最小外接旋转矩形
            cv::RotatedRect minRect = cv::minAreaRect(contours[i]);

            // 获取矩形的宽度和高度和角度
            double width = minRect.size.width;
            double height = minRect.size.height;
            double angle = minRect.angle;
            double area = minRect.size.area();
            
            angles.push_back(angle);
            areas.push_back(area);
            //std::cout << "width: " << width << ", height: " << height << std::endl;

            // 判断是长条方向
            if (width > height) {
                flag += 1;
                flags.push_back(1);
   
            }
            else{
                flag -= 1;
                flags.push_back(-1);

            }
        }
/**/
        //排除部分灯条
        //std::cout << "contours.size: " << contours.size() << std::endl;
        for (int i = 0; i < contours.size(); i++){
            cv::RotatedRect minRect = cv::minAreaRect(contours[i]);
            float width = minRect.size.width;
            float height = minRect.size.height;
            std::cout << "width: " << width << ", height: " << height << std::endl;

            //删除条件：1.横着的灯条，2.灯条的宽高比

            int longer = width > height ? width : height;
            int shorter = width > height ? height : width;
            if ((longer / shorter > 5 || longer / shorter < 2)
//                || (width * height < 400)
                ){
                contours.erase(contours.begin() + i);
                flags.erase(flags.begin() + i);
                angles.erase(angles.begin() + i);
                //std::cout << "erase:"<< i << " width: " << width << ", height: " << height << std::endl;
            }
        }

        // 没有灯条直接返回
        if(contours.size() < 1){
            return 0;
        }

        for (int i = 0; i < contours.size(); i++){
            cv::RotatedRect minRect = cv::minAreaRect(contours[i]);
            //std::cout <<"after_erase:"<<minRect.size.width << ","<<minRect.size.height << std::endl;
            //std::cout << "angle: " << angles[i] << std::endl;
        }   


        //绘制矩形框
        for (int i = 0; i < contours.size(); i++) {
            cv::RotatedRect minRect = cv::minAreaRect(contours[i]);
            cv::Point2f rectPoints[4];
            minRect.points(rectPoints);
            for (int j = 0; j < 4; j++) {
                cv::line(image, rectPoints[j], rectPoints[(j+1)%4], cv::Scalar(0,255,0), 2);
            }
        }

        /*
     
        */

        // 使用两个外接矩形的短边中点作为边界点
        std::vector<cv::Point2f> rectPointsVec;
        
        for (int i = 0; i < contours.size(); i++) {
            cv::RotatedRect minRect = cv::minAreaRect(contours[i]);
            cv::Point2f rectPoints[4];
            minRect.points(rectPoints);

            for (int  i = 0; i < 4; i++) {
                rectPointsVec.push_back(rectPoints[i]);
            }
            
        }
        //std::cout << "rectPointsVec" <<rectPointsVec << std::endl;
        //std::cout << "rectPointsVec.size" <<rectPointsVec.size() << std::endl;
        //尝试灯条组合

        // 将灯条从左到右排序并组合，基于每四个点的x坐标平均值
        for (int i = 0; i < rectPointsVec.size() / 4 - 1; i++) {
            for (int j = i + 1; j < rectPointsVec.size() / 4; j++) {
                // 计算第i组的x坐标平均值
                float avgX_i = (rectPointsVec[i * 4].x + rectPointsVec[i * 4 + 1].x + rectPointsVec[i * 4 + 2].x + rectPointsVec[i * 4 + 3].x) / 4;
                // 计算第j组的x坐标平均值
                float avgX_j = (rectPointsVec[j * 4].x + rectPointsVec[j * 4 + 1].x + rectPointsVec[j * 4 + 2].x + rectPointsVec[j * 4 + 3].x) / 4;

                // 如果第i组的平均x坐标大于第j组的，交换这两组，同时交换角度
                if (avgX_i > avgX_j) {
                    for (int k = 0; k < 4; k++) {
                        cv::Point2f temp = rectPointsVec[i * 4 + k];
                        rectPointsVec[i * 4 + k] = rectPointsVec[j * 4 + k];
                        rectPointsVec[j * 4 + k] = temp;
                    }
                    double temp_angle = angles[i];
                    angles[i] = angles[j];
                    angles[j] = temp_angle;
                }
            }
        }
        //std::cout << "softed_rectPointsVec" <<rectPointsVec << std::endl;
        //std::cout << "rectPointsVec.size" <<rectPointsVec.size() << std::endl;

        //识别装甲板
        std::vector<std::vector<cv::Point2f>> points_comb;
        for (int i = 0; i < contours.size() - 1; i++){
            cv::RotatedRect minRect0 = cv::minAreaRect(contours[i]);
            cv::RotatedRect minRect1 = cv::minAreaRect(contours[i + 1]);



            //判断条件：角度差小于5度，或者角度和与90度的差小于5度，即接近平行, 且宽高比在1到2之间
            if ((abs(angles[i] - angles[i + 1]) < 5) || 
                (abs(angles[i] + angles[i + 1] - 90)) < 5
                ){
            //std::cout << "angle0: " << angles[i] << std::endl;
            //std::cout << "angle1: " << angles[i + 1] << "\n" << std::endl;
                std::vector<cv::Point2f> points;
                std::vector<cv::Point> temp_points;

                cv::Point2f p1;
                cv::Point2f p2;
                cv::Point2f p3;
                cv::Point2f p4;

                temp_points.push_back(rectPointsVec[i * 4]);
                temp_points.push_back(rectPointsVec[i * 4 + 1]);
                temp_points.push_back(rectPointsVec[i * 4 + 2]);
                temp_points.push_back(rectPointsVec[i * 4 + 3]);

                temp_points.push_back(rectPointsVec[i * 4 + 4]);
                temp_points.push_back(rectPointsVec[i * 4 + 5]);
                temp_points.push_back(rectPointsVec[i * 4 + 6]);
                temp_points.push_back(rectPointsVec[i * 4 + 7]);

                //std::cout << "temp_points" << temp_points << std::endl;

                int flag_i = abs(temp_points[0].y - temp_points[1].y) < abs(temp_points[1].y - temp_points[2].y) ? 1 : 0;
                int flag_j = abs(temp_points[4].y - temp_points[5].y) < abs(temp_points[5].y - temp_points[6].y) ? 1 : 0;

                //std::cout << "flag_i" << flag_i << std::endl;
                //std::cout << "flag_j" << flag_j << std::endl;

                if(flag_i){
                    p1 = (temp_points[0] + temp_points[1]) / 2;
                    p2 = (temp_points[2] + temp_points[3]) / 2;
                }
                else{
                    p1 = (temp_points[1] + temp_points[2]) / 2;
                    p2 = (temp_points[0] + temp_points[3]) / 2;
                }
                if(flag_j){
                    p3 = (temp_points[4] + temp_points[5]) / 2;
                    p4 = (temp_points[6] + temp_points[7]) / 2;
                }
                else{
                    p3 = (temp_points[5] + temp_points[6]) / 2;
                    p4 = (temp_points[4] + temp_points[7]) / 2;
                }


                //判断条件：装甲板宽度高比1.5到3，
                if (abs(p3.x - p1.x) / abs(p2.y - p1.y) > 1 && abs(p3.x - p1.x) / abs(p2.y - p1.y) < 3){
                    points.push_back(p1);
                    points.push_back(p2);
                    points.push_back(p3);
                    points.push_back(p4);
                //std::cout<< "points_comb_v" << p1 << p2 << std::endl;
                //std::cout<< "points_comb" << points << std::endl;
                points_comb.push_back(points);
                }
            }
        }

        std::vector<Armor> armors;
        for (int i = 0; i < points_comb.size(); i++){
            std::vector<cv::Point2f> points;
            points = points_comb[i];
            //std::cout<< "points_comb" << points << std::endl;

            Armor armor(image_copy, false);

            armor.left_light.bottom = points[1];
            armor.left_light.top = points[0];
            armor.right_light.top = points[2];
            armor.right_light.bottom = points[3];

            armor.self_correction();

            //std::cout<< "armorlb" << armor.left_light.bottom << std::endl;
            //std::cout<< "armorlt" << armor.left_light.top << std::endl;
            //std::cout<< "armorrt" << armor.right_light.top << std::endl;
            //std::cout<< "armorrb" << armor.right_light.bottom << std::endl;

            armors.push_back(armor);
        }

        std::vector<std::string> ignore = {"negative"};
        armor::NumberClassifier number_classifier("/media/chen/Data/programme/Visual/SummerStudy/pnp/model/mlp.onnx", 
            "/media/chen/Data/programme/Visual/SummerStudy/pnp/model/label.txt", 
            0.2);
 
        //std::cout << "armors.size: " << armors.size() << std::endl;
        //std::cout << "points_comb.size: " << points_comb.size() << std::endl;
        for (int i = 0; i < points_comb.size(); i++){
            //std::cout << "i: " << i << std::endl;


            std::vector<Armor> armors_copy;
            armors_copy.push_back(armors[i]);

            number_classifier.ExtractNumbers(image_copy, armors_copy);

            number_classifier.Classify(armors_copy);

            points = points_comb[i];
            //std::cout<< "points_comb" << points << std::endl;
            //std::cout<< "armorlb" << armors[i].left_light.bottom << std::endl;
            //std::cout<< "armorlt" << armors[i].left_light.top << std::endl;
            //std::cout<< "armorrt" << armors[i].right_light.top << std::endl;
            //std::cout<< "armorrb" << armors[i].right_light.bottom << std::endl;
            //pnp(image, points);

            if (armors_copy[0].classification_result.find("negative")){
                find = 1;


                cv::Point2f number_position = (points[0] + points[1] + points[2] + points[3]) / 4;
                //std::cout << "image_points: " << points.size() << std::endl; 
                //pnp(image, points);

                // 绘制边界点
                for (int i = 0; i < points.size(); i++) {
                    cv::circle(image, points[i], 5, cv::Scalar(0,0,255), -1);
                }

                //连接对角线
                cv::line(image, points[1], points[2], cv::Scalar(255,0,0), 2);
                cv::line(image, points[0], points[3], cv::Scalar(255,0,0), 2);

                //在image上显示数字
                cv::putText(image, armors_copy[0].classification_result, number_position, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);

                //cv::imshow("image", image);
                
                std::cout << "armors"<< i << " classification_result: " << armors_copy[0].classification_result << std::endl;
                
                    
                std::cout << "armors"<< i << " classification_confidence: " << armors_copy[0].classification_confidence << std::endl;
            }
            else {
                find = 0;
                //cv::imshow("image", image);
            }
        }
        if(wait){
            cv::waitKey(0);
        }
        return 0;
    }

    std::vector<cv::Point2f> get_points(){
        return points;
    }

    bool get_find(){
        return find;
    }

private:
    cv::Mat image;
    std::vector<cv::Point2f> points;
    bool find;
};

