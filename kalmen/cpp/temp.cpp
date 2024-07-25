#include <iostream>
#include "eigen-3.4.0/Eigen/Dense"
#include "KalmanFilter.hpp"
#include <fstream>

using namespace std;
#define N 1000
#define T 0.01


double data_x[N],data_y[N];


float sample(float x0, float v0, float acc, float t)
{
	return x0 + v0*t + 1 / 2 * acc*t*t;
}

float GetRand()
{
	return 0.5 * rand() / RAND_MAX - 0.25;
}

int main()
{ 
    // 设置输出文件
	ofstream fout;
	fout.open("./source/0.txt");

	float t; // 时间
	for (int i = 0; i < N; i++)
	{
		t = i * T;
		data_x[i] = sample(0, -4, 0, t) + GetRand();
		data_y[i] = sample(0, 6.5, 0, t) + GetRand();
	}

	int stateSize = 6; //状态变量的维度
	int measSize = 2; //测量变量的维度
	int controlSize = 0;  //控制变量的维度

	KalmanFilter kf(stateSize, measSize, controlSize); //创建卡尔曼滤波器

    // 创建状态转移矩阵A，控制矩阵B，观测矩阵H，估计误差协方差矩阵P，测量噪声协方差矩阵R，过程噪声协方差矩阵Q
	Eigen::MatrixXd A(stateSize, stateSize);
	A << 1, 0, T, 0, 1 / 2 * T*T, 0,
		0, 1, 0, T, 0, 1 / 2 * T*T,
		0, 0, 1, 0, T, 0,
		0, 0, 0, 1, 0, T,
		0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 1;
	//cout << A;
	Eigen::MatrixXd B(0,0);
	Eigen::MatrixXd H(measSize, stateSize);
	H << 1, 0, 0, 0, 0, 0,
		0, 1, 0, 0, 0, 0;
	//cout << H;
	Eigen::MatrixXd P(stateSize, stateSize);
	P.setIdentity();
	Eigen::MatrixXd R(measSize, measSize);
	R.setIdentity()*0.01;
	Eigen::MatrixXd Q(stateSize, stateSize);
	Q.setIdentity()*0.001;

	// 创建状态向量x，控制向量u，测量向量z，预测结果res
	Eigen::VectorXd x(stateSize);
	Eigen::VectorXd u(0);
	Eigen::VectorXd z(measSize);
	z.setZero();
	Eigen::VectorXd res(stateSize);

	
	for (int i = 0; i < N; i++)
	{
		//cout << "state_" << i << ":\n";
		// 初始化状态向量x
		if (i == 0){
			x << data_x[i], data_y[i], 0, 0, 0, 0;
			kf.init(x, P, R, Q);
		}

		// 预测， 结果存储在res中
		res << kf.predict(A);
		// 观测
		z << data_x[i], data_y[i];
		// 更新滤波器
		kf.update(H,z);
		// 存储结果
		fout << data_x[i] << " " << res[0] << " " << data_y[i] << " " << res[1] << " " << res[2] << " " << res[3] << " " << res[4] << " " << res[5] << endl;
	}
	fout.close();
	// cout << "Done, use python script to draw the figure....\n";
	std::cin.get();
	return 0;
}


void KalmanFilter_predict(int data_size, int stateSize, int measSize, int controlSize, 
	                      Eigen::MatrixXd data, Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd H, 
						  std::string output_file, 
						  double P_value = 1.0, double R_value = 0.1, double Q_value = 0.01,
						  bool predict = false){


	std::ofstream fout;
	fout.open(output_file);

	// 初始化估计误差协方差矩阵P，测量噪声协方差矩阵R，过程噪声协方差矩阵Q
	Eigen::MatrixXd P(stateSize, stateSize);
	P.setIdentity() * P_value;
	Eigen::MatrixXd R(measSize, measSize);
	R.setIdentity() * R_value;
	Eigen::MatrixXd Q(stateSize, stateSize);
	Q.setIdentity() * Q_value;
	
	KalmanFilter kf(stateSize, measSize, controlSize); //创建卡尔曼滤波器

	// 创建状态向量x，控制向量u，测量向量z，预测结果res
	Eigen::VectorXd x(stateSize);
	x.setZero();
	Eigen::VectorXd u(0);
	Eigen::VectorXd z(measSize);
	z.setZero();
	Eigen::VectorXd res(stateSize);

	for (int i = 0; i < data_size; i++)
	{
		
		//cout << "state_" << i << ":\n";
		// 初始化状态向量x
		if (i == 0){
			for (int j = 0; j < stateSize; j++){
				x[i] = data(i,j);
			}
			kf.init(x, P, R, Q);
		}

		// 预测， 结果存储在res中
		res << kf.predict(A);
		// 观测
		for (int j = 0; j < stateSize; j++){
			z[i] = data(i,j);
		}
		// 更新滤波器
		kf.update(H,z);
		// 存储结果
		fout << data(i,0) << " " << res[0] << " " << data(i,1) << " " << res[1] << " " << res[2] << " " << res[3] << " " << res[4] << " " << res[5] << std::endl;
	}
	fout.close();
}