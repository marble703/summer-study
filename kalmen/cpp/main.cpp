#include <iostream>
#include "eigen-3.4.0/Eigen/Dense"
#include "KalmanFilter.hpp"
#include <fstream>


void KalmanFilter_predict(int data_size, int stateSize, int measSize, int controlSize, 
	                      Eigen::MatrixXd data, Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd H, 
						  Eigen::MatrixXd P, Eigen::MatrixXd R, Eigen::MatrixXd Q, Eigen::MatrixXd X0,
						  std::string output_file = "just a default string", 
						  bool outout_to_file = true, bool predict = false){

	// 初始化估计误差协方差矩阵P，测量噪声协方差矩阵R，过程噪声协方差矩阵Q

	
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

			std::cout << "res: " << res << " ";
			std::cout << "\n"<< std::endl;
		}
		res << kf.predict(A);
		std::cout << "res_pred: " << res << " ";
	}
}

class KalmanFilter_predict{
private:
	int data_size; int stateSize; int measSize; int controlSize; 
	Eigen::MatrixXd data; Eigen::MatrixXd A; Eigen::MatrixXd B; Eigen::MatrixXd H; 
	Eigen::MatrixXd P; Eigen::MatrixXd R; Eigen::MatrixXd Q;
	Eigen::MatrixXd X0;
	std::string output_file; 
	bool outout_to_file; bool predict;

public:
	KalmanFilter kf;
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

int main(){
	int data_size; int stateSize; int measSize; int controlSize; 
	Eigen::MatrixXd data; Eigen::MatrixXd A; Eigen::MatrixXd B; Eigen::MatrixXd H; 
	Eigen::MatrixXd P; Eigen::MatrixXd R; Eigen::MatrixXd Q;
	Eigen::MatrixXd X0;
	std::string input_file; std::string output_file; 

	input_file = "/media/chen/Data/programme/Visual/SummerStudy/kalmen/source/homework_data_4_.txt";
	output_file = "/media/chen/Data/programme/Visual/SummerStudy/kalmen/cpp/output/homework_data_1_output.txt";

	std::ifstream file(input_file); // 打开文件
	std::string line;
	std::vector<std::vector<double>> tempData; // 临时存储数据

	if (file.is_open()) {
		while (getline(file, line)) { // 逐行读取文件
			std::stringstream ss(line);
			std::vector<double> rowData;
			double value;
			while (ss >> value) { // 读取每行的每个值
				rowData.push_back(value);
			}
			tempData.push_back(rowData); // 将这一行的数据添加到临时存储中
		}
		file.close(); // 关闭文件
	}

	// 获取数据的行数和列数
	data_size = tempData.size();
	measSize = data_size > 0 ? tempData[0].size() : 0;
	data.resize(data_size, measSize);
	//std::cout << "tempData[0][0]: " << tempData[0][0] << "tempData[0][1]: " << tempData[0][1] << std::endl;
	//std::cout << "data_size: " << data_size << std::endl;
	//std::cout << "measSize: " << measSize << std::endl;
	// 将数据从std::vector转移到Eigen::MatrixXd
	for (int i = 0; i < data_size; ++i) {
		for (int j = 0; j < measSize; ++j) {
			//std::cout << tempData[i][j] << std::endl;
			data(i, j) = tempData[i][j];
		}
	}

	//data1
	stateSize = 3; measSize = 1; controlSize = 0;

	A.resize(stateSize, stateSize);
	B.resize(stateSize, 1);
	H.resize(measSize, stateSize);

	P.resize(stateSize, stateSize);
	R.resize(measSize, measSize);
	Q.resize(stateSize, stateSize);

	X0.resize(stateSize, 1);

	A << 1, 0.01, 0,
		 0, 1, 0.01,
		 0, 0, 1;

	B << 0, 0, 0;

	H << 1, 0, 0;

	P << 10, 0, 0,
		 0, 10, 0,
		 0, 0, 1;

	R << 1000;

	Q << 1, 0, 0,
		 0, 0.01, 0,
		 0, 0, 0.01;

	X0 << 3, 0, 0;


	KalmanFilter_predict(data_size, stateSize, measSize, controlSize, 
	                     data, A, B, H, P, R, Q, X0,
						 output_file);


	return 0;
}
