#include <opencv2/opencv.hpp>
#include <vector>
#include "Armor.hpp"
#include "number_classifier.hpp"
#include "process.hpp"

int main() {
    //cv::Mat image = cv::imread("/media/chen/Data/programme/Visual/SummerStudy/pnp/detect/two_armor.png");
    cv::Mat image = cv::imread("/media/chen/Data/programme/Visual/SummerStudy/pnp/detect/0.jpg");
    if (image.empty()) {
        std::cerr << "Failed to read image." << std::endl;
        return -1;
    }
    cv::Mat image_copy = image.clone();

    cv::Mat eroded_image = image_preprocess(image);

    // 查找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(eroded_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    int flag = 0;
    std::cout << contours.size() << std::endl;
    for (size_t i = 0; i < contours.size(); i++) {
        // 对每个轮廓计算最小外接旋转矩形
        cv::RotatedRect minRect = cv::minAreaRect(contours[i]);

        // 获取矩形的宽度和高度
        float width = minRect.size.width;
        float height = minRect.size.height;

        //std::cout << "width: " << width << ", height: " << height << std::endl;

        // 判断是长条方向
        if (width > height) {
            flag += 1;
            // 绘制旋转矩形
            cv::Point2f rectPoints[4];
            minRect.points(rectPoints);
            for (int j = 0; j < 4; j++) {
                cv::line(image, rectPoints[j], rectPoints[(j+1)%4], cv::Scalar(0,255,0), 2);
            }
        }
        else{
            flag -= 1;
        }
    }
    //std::cout<<flag<<std::endl;
    //排除横向的灯条
    if(flag == 0){
        std::cout << "No enough light bar detected" << std::endl;
        return 0;
    }

    for (int i = 0; i < contours.size(); i++){
        cv::RotatedRect minRect = cv::minAreaRect(contours[i]);
        float width = minRect.size.width;
        float height = minRect.size.height;
        if(flag < 0 && width > height || flag > 0 && width < height){
            contours.erase(contours.begin() + i);
        }
    }
    std::vector<cv::Point2f> rectPointsVec;
    // 使用两个外接矩形的短边中点作为边界点
    
    
    for (size_t i = 0; i < contours.size(); i++) {
        cv::RotatedRect minRect = cv::minAreaRect(contours[i]);
        cv::Point2f rectPoints[4];
        minRect.points(rectPoints);

        for (int  i = 0; i < 4; i++) {
            rectPointsVec.push_back(rectPoints[i]);
        }
        std::cout << "rectPointsVec" <<rectPointsVec << std::endl;


    }
    
    //std::cout << points << std::endl;
    //尝试灯条组合
    std::vector<std::vector<cv::Point2f>> points_comb;
    for (size_t i = 0; i < contours.size() - 1; i++) {

        for (int j = i + 1; j < contours.size(); j++){
        std::vector<cv::Point2f> points;
        std::vector<cv::Point> temp_points;
        cv::Point2f p1;
        cv::Point2f p2;
        cv::Point2f p3;
        cv::Point2f p4;
        //std::cout << "i" <<i << std::endl;
        //std::cout << "j" <<j << std::endl;
        //std::cout << "rectPointsVeccdcdcd" <<rectPointsVec[i] << std::endl;

        temp_points.push_back(rectPointsVec[i * 4]);
        temp_points.push_back(rectPointsVec[i * 4 + 1]);
        temp_points.push_back(rectPointsVec[i * 4 + 2]);
        temp_points.push_back(rectPointsVec[i * 4 + 3]);

        temp_points.push_back(rectPointsVec[j * 4]);
        temp_points.push_back(rectPointsVec[j * 4 + 1]);
        temp_points.push_back(rectPointsVec[j * 4 + 2]);
        temp_points.push_back(rectPointsVec[j * 4 + 3]);

        if(flag > 0){
            p1 = (temp_points[0] + temp_points[1]) / 2;
            p2 = (temp_points[2] + temp_points[3]) / 2;
            p3 = (temp_points[4] + temp_points[5]) / 2;
            p4 = (temp_points[6] + temp_points[7]) / 2;

        }
        if(flag < 0){
            p1 = (temp_points[1] + temp_points[2]) / 2;
            p2 = (temp_points[0] + temp_points[3]) / 2;
            p3 = (temp_points[5] + temp_points[6]) / 2;
            p4 = (temp_points[4] + temp_points[7]) / 2;
            
        }

        points.push_back(p1);
        points.push_back(p2);
        points.push_back(p3);
        points.push_back(p4);
        //std::cout<< "points_comb_v" << p1 << p2 << std::endl;
        //std::cout<< "points_comb" << points << std::endl;
        points_comb.push_back(points);
        }
    }

    std::vector<Armor> armors;
    for (int i = 0; i < points_comb.size(); i++){
        std::vector<cv::Point2f> points;
        points = points_comb[i];
        //std::cout<< "points_comb" << points << std::endl;

        Armor armor;
        armor.left_light.bottom = points[1];
        armor.left_light.top = points[0];
        armor.right_light.top = points[2];
        armor.right_light.bottom = points[3];

        //std::cout<< "armorlb" << armor.left_light.bottom << std::endl;
        //std::cout<< "armorlt" << armor.left_light.top << std::endl;
        //std::cout<< "armorrt" << armor.right_light.top << std::endl;
        //std::cout<< "armorrb" << armor.right_light.bottom << std::endl;


        armors.push_back(armor);
    }


    std::vector<std::string> ignore = {"negative"};
    armor::NumberClassifier number_classifier("/media/chen/Data/programme/Visual/SummerStudy/pnp/model/mlp.onnx", 
        "/media/chen/Data/programme/Visual/SummerStudy/pnp/model/label.txt", 
        0.5);
    number_classifier.ExtractNumbers(image_copy, armors);

    number_classifier.Classify(armors);
    for (int i = 0; i < points_comb.size(); i++){
        std::vector<cv::Point2f> points;

        points = points_comb[i];
        //std::cout<< "points_comb" << points << std::endl;
        //pnp(image, points);

        if(armors[i].classification_result .find("negative")){
            //std::cout << "armors[i].classification_result" << armors[i].classification_result << std::endl;
        //计算数字的位置，位于四个顶点中心

            cv::Point2f number_position = (points[0] + points[1] + points[2] + points[3]) / 4;
            pnp(image, points);

            //在image上显示数字
            cv::putText(image, 
            armors[i].classification_result, 
            number_position, 
            cv::FONT_HERSHEY_SIMPLEX, 1, 
            cv::Scalar(0, 0, 255), 
            2);

            cv::imshow("image", image);
            cv::waitKey(0);

            std::cout << armors[i].classification_result << std::endl;

            std::cout << armors[i].classification_confidence << std::endl;
        }
    }
 /*

    //计算数字的位置，位于四个顶点中心
    cv::Point2f number_position = (points[0] + points[1] + points[2] + points[3]) / 4;

    //在image上显示数字
    cv::putText(image, armors[0].classification_result, number_position, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
    cv::imshow("image", image);
    cv::waitKey(0);
*/


    return 0;
}