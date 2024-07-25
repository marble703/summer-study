#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <numeric>

using namespace Eigen;
using namespace std;

// 一步预测
pair<VectorXd, MatrixXd> kf_predict(const VectorXd& X0, const MatrixXd& P0, const MatrixXd& A, const MatrixXd& Q, const MatrixXd& B, const VectorXd& U1) {
    VectorXd X10 = A * X0 + B * U1;
    MatrixXd P10 = A * P0 * A.transpose() + Q;
    return make_pair(X10, P10);
}

// 测量更新
tuple<VectorXd, MatrixXd, MatrixXd> kf_update(const VectorXd& X10, const MatrixXd& P10, const VectorXd& Z, const MatrixXd& H, const MatrixXd& R) {
    MatrixXd K = P10 * H.transpose() * (H * P10 * H.transpose() + R).inverse();
    VectorXd X1 = X10 + K * (Z - H * X10);
    MatrixXd P1 = (MatrixXd::Identity(K.rows(), H.cols()) - K * H) * P10;
    return make_tuple(X1, P1, K);
}

int main() {
    ifstream dataFile("/media/chen/Data/programme/Visual/SummerStudy/kalmen/source/homework_data_1_.txt");
    string line;
    vector<double> y;
    vector<double> t;

    // 读取数据
    while (getline(dataFile, line)) {
        double x, y_val;
        istringstream iss(line);
        iss >> x >> y_val;
        t.push_back(x);
        y.push_back(y_val);
    }

    int n = t.size(); // 数据量
    int nx = 3; // 变量数量
    double dt = t[1] - t[0]; // t的一阶导

    // 观测噪声协方差
    MatrixXd R = MatrixXd::Identity(1, 1) * 1000;

    // 状态转移矩阵
    MatrixXd A = MatrixXd::Identity(nx, nx);

    // 外部输入量
    MatrixXd B = MatrixXd::Zero(nx, 1);
    VectorXd U1 = VectorXd::Zero(1);

    // 状态假设（观测）初始值
    double x0 = accumulate(y.begin(), y.begin() + 5, 0.0) / 5;
    double v0 = (accumulate(y.begin() + 5, y.begin() + 10, 0.0) / 5 - x0) / 5;
    double a0 = 0.0;
    VectorXd X0(nx);
    X0 << x0, v0, a0;

    // 初始状态不确定度
    MatrixXd P0 = MatrixXd::Zero(nx, nx);
    P0.diagonal() << 10, 10, 1;

    // 状态递推噪声协方差
    MatrixXd Q = MatrixXd::Zero(nx, nx);
    Q.diagonal() << 1, 0.01, 0.01;

    // 开始处理
    VectorXd X1 = X0;
    MatrixXd P1 = P0;
    MatrixXd K;

    for (int i = 0; i < n; ++i) {
        VectorXd Zi(1);
        Zi << y[i];
        MatrixXd Hi(1, nx);
        Hi << 1, 0, 0;

        tie(X1, P1, K) = kf_update(X1, P1, Zi, Hi, R);
    }

    cout << "Kalman Gain (K): \n" << K << endl;

	//将结果写入文件
	ofstream fout("/media/chen/Data/programme/Visual/SummerStudy/kalmen/cpp/output/homework_data_1_output.txt");
	for (int i = 0; i < n; ++i) {
		fout << t[i] << " " << y[i] << " " << X1[0] << " " << X1[1] << " " << X1[2] << endl;
	}
	fout.close();
	

    return 0;
}