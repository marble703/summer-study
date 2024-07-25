#pragma once

#include "../eigen-3.4.0/Eigen/Dense"
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <math.h>

#include "KalmanFilter.hpp"

class KalmanFilter_predict{
private:
	int data_size; int stateSize; int measSize; int controlSize; 
	Eigen::MatrixXd data; Eigen::MatrixXd A; Eigen::MatrixXd B; Eigen::MatrixXd H; 
	Eigen::MatrixXd P; Eigen::MatrixXd R; Eigen::MatrixXd Q;
	Eigen::MatrixXd X0;
	std::string output_file; 
	bool outout_to_file; bool predict;
	KalmanFilter kf;




public:
	KalmanFilter_predict(int data_size, int stateSize, int measSize, int controlSize, 
	                     Eigen::MatrixXd data, Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd H, 
						 Eigen::MatrixXd P, Eigen::MatrixXd R, Eigen::MatrixXd Q, Eigen::MatrixXd X0,
						 std::string output_file = "just a default string", 
						 bool outout_to_file = true) :
						 data_size(data_size), stateSize(stateSize), measSize(measSize), controlSize(controlSize),
						 data(data), A(A), B(B), H(H), P(P), R(R), Q(Q), X0(X0), output_file(output_file),
						 outout_to_file(outout_to_file), 
						 kf(stateSize, measSize, controlSize){
						 };

	~KalmanFilter_predict() = default;

	void filte(){
		KalmanFilter kf(stateSize, measSize, controlSize); //创建卡尔曼滤波器

		// 创建状态向量x，控制向量u，测量向量z，预测结果res
		Eigen::VectorXd x(stateSize);
		x.setZero();
		Eigen::VectorXd u(0);
		Eigen::VectorXd z(measSize);
		z.setZero();
		Eigen::VectorXd res(stateSize);


		if(outout_to_file){
			std::ofstream fout;
			fout.open(output_file);
			//std::cout << "data_size: " << data_size << std::endl;
			for (int i = 0; i < data_size; i++){
				//std::cout << "state_" << i << ":\n";
				// 初始化状态向量x
				if (i == 0){
					for (int j = 0; j < stateSize; j++){

						x(j) = X0(j);
						std::cout << "x0: " << x(j) << std::endl;

						//std::cout << "xi: " << x[i] << std::endl;
					}
					kf.init(x, P, R, Q);
				}
				//std::cout << "test" <<std::endl;
				// 预测， 结果存储在res中
				res << kf.predict(A);
				//std::cout << "\nres: " << res << std::endl;
				// 观测
				for (int j = 1; j < measSize + 1; j++){
					//std::cout << "j: " << j << std::endl;
					//std::cout << "data(i,j): " << data(i,j) << std::endl;
					z << data(i,j);
					//std::cout << "zj: " << z << std::endl;
				}
				//std::cout << "z: " << z << std::endl;
				// 更新滤波器
				kf.update(H,z);


				// 存储结果
				for (int j = 1; j < measSize; j++) {
					fout << data(i,j)  << " ";
					//std::cout << "data: " << data(i,j) << " ";
				}
				for (int j = 0; j < stateSize; j++) {
					fout << res[j] << " ";
					
				}
				//std::cout << "res: " << res << " ";
				fout << std::endl;
				//std::cout << "\n"<< std::endl;
			}
			fout.close();
		}
		else{
			for (int i = 0; i < data_size; i++){
				//std::cout << "state_" << i << ":\n";
				// 初始化状态向量x
				if (i == 0){
					for (int j = 0; j < stateSize; j++){

						x(j) = X0(j);
						//std::cout << "x0: " << x(i) << std::endl;

						//std::cout << "xi: " << x[i] << std::endl;
					}
					kf.init(x, P, R, Q);
				}
				//std::cout << "test" <<std::endl;
				// 预测， 结果存储在res中
				res << kf.predict(A);
				//std::cout << "\nres: " << res << std::endl;
				// 观测
				for (int j = 1; j < measSize + 1; j++){
					//std::cout << "j: " << j << std::endl;
					//std::cout << "data(i,j): " << data(i,j) << std::endl;
					z << data(i,j);
					//std::cout << "zj: " << z << std::endl;
				}
				//std::cout << "z: " << z << std::endl;
				// 更新滤波器
				kf.update(H,z);

				//std::cout << "res: " << res << " ";
				//std::cout << "\n"<< std::endl;
			}
			//res << kf.predict(A);
			//std::cout << "res_pred: " << res << " ";
		}
	}

	Eigen::MatrixXd update_step(Eigen::MatrixXd data){

		// 创建状态向量x，控制向量u，测量向量z，预测结果res
		Eigen::VectorXd x(stateSize);
		x.setZero();
		Eigen::VectorXd u(0);
		Eigen::VectorXd z(measSize);
		z.setZero();
		Eigen::VectorXd res(stateSize);

		// 观测
		for (int j = 1; j < measSize + 1; j++){
			//std::cout << "j: " << j << std::endl;
			//std::cout << "data(i,j): " << data(i,j) << std::endl;
			z << data(0, j);
			//std::cout << "zj: " << z << std::endl;
		}
		// 更新滤波器
		kf.update(H,z);

		//预测
		res << kf.predict(A);

		return res;
	}

	Eigen::MatrixXd predict_step(){
		Eigen::VectorXd res(stateSize);
		res << kf.predict(A);
		return res;
	}
};

class Prediction {
private:
    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> pair;
    std::vector<std::pair<Eigen::MatrixXd, Eigen::MatrixXd>> data;

    Eigen::MatrixXd r1, r2; //旋转矩阵
    Eigen::MatrixXd d1, d2; //法向量方向向量
    Eigen::MatrixXd p1, p2; //位置向量

    Eigen::MatrixXd center;
    double radius;

    std::pair<Eigen::Vector3d, Eigen::Vector3d> closestPointsOnLines(const Eigen::Vector3d& p1, const Eigen::Vector3d& d1, const Eigen::Vector3d& p2, const Eigen::Vector3d& d2) {

        Eigen::Vector3d p1p2 = p2 - p1;
        Eigen::Vector3d d1xd2 = d1.cross(d2);

        double d1xd2LengthSquared = d1xd2.squaredNorm();

        double t1 = ((d2.dot(p1p2) * d1.dot(d2)) - (d1.dot(d2) * d2.squaredNorm())) / d1xd2LengthSquared;
        double t2 = ((d1.dot(p1p2) * d1.dot(d2)) - (d1.squaredNorm() * d2.dot(p1p2))) / d1xd2LengthSquared;

        Eigen::Vector3d closestPoint1 = p1 + t1 * d1;
        Eigen::Vector3d closestPoint2 = p2 + t2 * d2;

        return {closestPoint1, closestPoint2};

}

public:
    Prediction() = default;
    ~Prediction() = default;

    Prediction(std::vector<std::pair<Eigen::MatrixXd, Eigen::MatrixXd>> data) : data(data) {
        r1 = data[0].first;
        r2 = data[1].first;
        p1 = data[0].second;
        p2 = data[1].second;

        Eigen::VectorXd n1(3);
        Eigen::VectorXd n2(3);
        n1 << 0, 0, 1;
        n2 << 0, 0, 1;

        d1 = r1 * n1;
        d2 = r2 * n2;
    }

    Eigen::MatrixXd find_center(){
        auto [cp1, cp2] = closestPointsOnLines(p1, d1, p2, d2);
        center = (cp1 + cp2) / 2;
        return center;
    }

    double find_radius(){
		// 圆心到两点的距离的平均值
		radius = (center - p1).norm() + (center - p2).norm() / 2;

        return radius;
    }
};

class Prediction_1{
private:
    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> pair;
    std::vector<std::pair<Eigen::MatrixXd, Eigen::MatrixXd>> data;

    Eigen::MatrixXd r1, r2; //旋转向量
    Eigen::MatrixXd p1, p2; //位置向量

	Eigen::MatrixXd rm1, rm2; //旋转矩阵
	Eigen::MatrixXd d1, d2; //法向量方向向量

	Eigen::VectorXd n1;
	Eigen::VectorXd n2;

    Eigen::MatrixXd center;
    double radius;

double av(){
	//r1减去r2的模长
	double av = (r1 - r2).norm();
	return av;
}

double lv(){
	//p1减去p2的模长
	double lv = (p1 - p2).norm();
	return lv;
}

public:
	double rad(){
		radius = lv() / av();
		return radius;
	}

	void set_rm(Eigen::MatrixXd rm1, Eigen::MatrixXd rm2){
		this->rm1 = rm1;
		this->rm2 = rm2;
	}

	Eigen::MatrixXd find_center(){
		d1 = rm1 * n1;
		d2 = rm2 * n2;

		Eigen::MatrixXd c1 = p1 + d1;
		Eigen::MatrixXd c2 = p2 + d2;
		center = (c1 + c2) / 2;
		return center;
	}



	Prediction_1() = default;
	~Prediction_1() = default;

	Prediction_1(std::vector<std::pair<Eigen::MatrixXd, Eigen::MatrixXd>> data) : data(data){
        r1 = data[0].first;
        r2 = data[1].first;
        p1 = data[0].second;
        p2 = data[1].second;

		n1.resize(3);
		n2.resize(3);

        n1 << 0, 0, 1;
        n2 << 0, 0, 1;
	}

};

class Prediction_2{

};