/*
文件内包含image_preprocess和pnp
*/

#pragma once
#include "../eigen-3.4.0/Eigen/Dense"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <vector>


cv::Mat image_preprocess(cv::Mat image) {
    cv::Mat gray, binary;
    //灰度化
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    //二值化
    cv::threshold(gray, binary, 128, 255, cv::THRESH_BINARY); 
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
	//cv::imshow("dilated_image", dilated_image);
	//cv::waitKey(0);

    return dilated_image;
}

std::pair<Eigen::MatrixXd, Eigen::MatrixXd>  pnp(cv::Mat image, std::vector<cv::Point2f> points) {
	//std::cout << "points: "<< points.size()<<std::endl;
	// 2D 特征点像素坐标
	std::vector<cv::Point2d> image_points;
	image_points.push_back(points[0]);
	image_points.push_back(points[2]);
	image_points.push_back(points[3]);
	image_points.push_back(points[1]);
	//std::cout << "test "<< std::endl;

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
	//std::cout << "rotMat" << std::endl << rotMat << std::endl << std::endl;

	cv::Mat P_oc;
	P_oc = -rotMat.inv() * Tvec;
	// 求解相机的世界坐标，得出p_oc的第三个元素即相机到物体的距离即深度信息，单位是mm
	std::cout << "P_oc" << std::endl << P_oc << std::endl;
    //cv::imshow("image", image);
	//cv::waitKey(0);

	Eigen::MatrixXd rotation_matrix = Eigen::MatrixXd::Zero(3, 3);
	Eigen::MatrixXd translation_matrix = Eigen::MatrixXd::Zero(3, 1);

	cv::cv2eigen(rotMat, rotation_matrix);
	cv::cv2eigen(translation_vector, translation_matrix);

	std::pair<Eigen::MatrixXd, Eigen::MatrixXd> result = std::make_pair(rotation_matrix, translation_matrix);

	return result;
}

class Pnp{
public:
	std::vector<cv::Point2d> image_points;
	std::vector<cv::Point3d> model_points;
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
	
	cv::Mat Rvec;
	cv::Mat_<float> Tvec;

	// 旋转向量
	cv::Mat rotation_vector;
	// 平移向量
	cv::Mat translation_vector;

	cv::Mat_<float> rotMat;
	cv::Mat P_oc;
	Eigen::MatrixXd rotation_matrix = Eigen::MatrixXd::Zero(3, 3);
	Eigen::MatrixXd translation_matrix = Eigen::MatrixXd::Zero(3, 1);

	Pnp(cv::Mat image, std::vector<cv::Point2f> points) {
		//std::cout << "points: "<< points.size()<<std::endl;
		// 2D 特征点像素坐标
		image_points.push_back(points[0]);
		image_points.push_back(points[2]);
		image_points.push_back(points[3]);
		image_points.push_back(points[1]);
		//std::cout << "test "<< std::endl;

		// 画出四个特征点
		for (int i = 0; i < image_points.size(); i++)
		{
			circle(image, image_points[i], 3, cv::Scalar(0, 0, 255), -1);
		}

		// 3D 特征点世界坐标，与像素坐标对应，单位是mm
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



		// pnp求解
		cv::solvePnP(model_points, image_points, camera_matrix, dist_coeffs,
			rotation_vector, translation_vector, false, cv::SOLVEPNP_ITERATIVE);
		// 默认ITERATIVE方法

		std::cout << "Rotation Vector " << std::endl << rotation_vector << std::endl << std::endl;
		std::cout << "Translation Vector" << std::endl << translation_vector << std::endl << std::endl;
		
		rotation_vector.convertTo(Rvec, CV_32F);  // 旋转向量转换格式
		translation_vector.convertTo(Tvec, CV_32F); // 平移向量转换格式 

		cv::Rodrigues(Rvec, rotMat);
		// 旋转向量转成旋转矩阵
		//std::cout << "rotMat" << std::endl << rotMat << std::endl << std::endl;

		P_oc = -rotMat.inv() * Tvec;
		// 求解相机的世界坐标，得出p_oc的第三个元素即相机到物体的距离即深度信息，单位是mm
		std::cout << "P_oc" << std::endl << P_oc << std::endl;
		//cv::imshow("image", image);
		//cv::waitKey(0);



	}

	std::pair<Eigen::MatrixXd, Eigen::MatrixXd> get_rotm_trav() {
		cv::cv2eigen(rotMat, rotation_matrix);
		cv::cv2eigen(translation_vector, translation_matrix);
		return std::make_pair(rotation_matrix, translation_matrix);
	}

	std::pair<Eigen::MatrixXd, Eigen::MatrixXd> get_rotv_trav() {
		Eigen::MatrixXd rotv, trav;
		cv::cv2eigen(rotation_vector, rotv);
		cv::cv2eigen(translation_vector, trav);
		return std::make_pair(rotv, trav);
	}

	Eigen::MatrixXd get_rotm() {
		return rotation_matrix;
	}
};