#include <opencv2/opencv.hpp>
#include <vector>
#include "Armor.hpp"
#include "number_classifier.hpp"

int main() {
    cv::Mat image = cv::imread("/media/chen/Data/programme/Visual/SummerStudy/pnp/detect/20.jpg");
    if (image.empty()) {
        std::cerr << "Failed to read image." << std::endl;
        return -1;
    }
    cv::Mat image_copy = image.clone();

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

    // 绘制边界点
    for (size_t i = 0; i < points.size(); i++) {
        cv::circle(image, points[i], 5, cv::Scalar(0,0,255), -1);
    }

    //连接对角线
    cv::line(image, points[1], points[2], cv::Scalar(255,0,0), 2);
    cv::line(image, points[0], points[3], cv::Scalar(255,0,0), 2);

	// 2D 特征点像素坐标
	std::vector<cv::Point2d> image_points;
	image_points.push_back(points[0]);
	image_points.push_back(points[2]);
	image_points.push_back(points[3]);
	image_points.push_back(points[1]);

	// 画出四个特征点
	for (int i = 0; i < image_points.size(); i++)
	{
		circle(image, image_points[i], 3, cv::Scalar(0, 0, 255), -1);
	}

	// 3D 特征点世界坐标，与像素坐标对应，单位是mm
	std::vector<cv::Point3d> model_points;
	model_points.push_back(cv::Point3d(-67.5f, -27.5f, 0)); // 左上
	model_points.push_back(cv::Point3d(+67.5f, -27.5f, 0)); // 右上
	model_points.push_back(cv::Point3d(+67.5f, +27.5f, 0)); // 右下
	model_points.push_back(cv::Point3d(-67.5f, +27.5f, 0)); // 左下

/*
	model_points.push_back(cv::Point3d(+0.0f, -55.0f, 0)); // 左上
	model_points.push_back(cv::Point3d(+135.0f, +0.0f, 0)); // 右上
	model_points.push_back(cv::Point3d(+135.0f, +0.0f, 0)); // 右下
	model_points.push_back(cv::Point3d(+0.0f, -55.0f, 0)); // 左下
*/

	// 相机内参矩阵
	cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
    2102.080562187802,0,689.2057889332623,
    0,2094.0179120166754,496.6622802275393,
    0,0,1);

	// 相机畸变系数
	cv::Mat dist_coeffs = (cv::Mat_<double>(5, 1) <<         
        -0.06478109387525666,
        0.39036067923005396,
        -0.0042514793151166306,
        0.008306749648029776,
        -1.6613800909405605);

	std::cout << "Camera Matrix " << std::endl << camera_matrix << std::endl << std::endl;
	// 旋转向量
	cv::Mat rotation_vector;
	// 平移向量
	cv::Mat translation_vector;

	// pnp求解
	cv::solvePnP(model_points, image_points, camera_matrix, dist_coeffs,
		rotation_vector, translation_vector, false, cv::SOLVEPNP_ITERATIVE);
	// 默认ITERATIVE方法

	std::cout << "Rotation Vector " << std::endl << rotation_vector << std::endl << std::endl;
	std::cout << "Translation Vector" << std::endl << translation_vector << std::endl << std::endl;
    
	cv::Mat Rvec;
	cv::Mat_<float> Tvec;
	rotation_vector.convertTo(Rvec, CV_32F);  // 旋转向量转换格式
	translation_vector.convertTo(Tvec, CV_32F); // 平移向量转换格式 

	cv::Mat_<float> rotMat(3, 3);
	Rodrigues(Rvec, rotMat);
	// 旋转向量转成旋转矩阵
	std::cout << "rotMat" << std::endl << rotMat << std::endl << std::endl;

	cv::Mat P_oc;
	P_oc = -rotMat.inv() * Tvec;
	// 求解相机的世界坐标，得出p_oc的第三个元素即相机到物体的距离即深度信息，单位是mm
	std::cout << "P_oc" << std::endl << P_oc << std::endl;
    cv::imshow("image", image);



    //将图片写入vector
    std::vector<cv::Mat> images;

    images.push_back(image_copy);

    Armor armor(image_copy);
    std::vector<Armor> armors;
    armors.push_back(armor);
    std::vector<std::string> ignore = {"negative"};

    armor::NumberClassifier number_classifier("/media/chen/Data/programme/Visual/SummerStudy/pnp/model/mlp.onnx", "/media/chen/Data/programme/Visual/SummerStudy/pnp/model/label.txt", 0.5);

    number_classifier.ExtractNumbers(image_copy, armors);

    number_classifier.Classify(armors);

    std::cout << armors[0].classification_result << std::endl;

    std::cout << armors[0].classification_confidence << std::endl;

    //计算数字的位置，位于四个顶点中心
    cv::Point2f number_position = (points[0] + points[1] + points[2] + points[3]) / 4;

    //在image上显示数字
    cv::putText(image, armors[0].classification_result, number_position, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
    cv::imshow("image", image);
    cv::waitKey(0);

    return 0;
}