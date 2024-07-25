#include <iostream>
#include <cmath>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

// 定义状态变量和观测变量的维度
const int nx = 4;  // 状态变量的维度
const int nz = 2;  // 观测变量的维度

// 定义sigma点的个数及其权重
const int n_sigma = 2 * nx + 1;
double lambda = 3 - nx;

// 定义系统的状态变量和协方差矩阵
VectorXd x(nx);
MatrixXd P(nx, nx);

// 定义观测变量和协方差矩阵
VectorXd z(nz);
MatrixXd R(nz, nz);

// 参数设置
double delta_t = 0.1;

// 定义非线性状态转移函数
VectorXd f(VectorXd x, double delta_t) {
    VectorXd x_pred(nx);
    x_pred(0) = x(0) + x(2) * delta_t;
    x_pred(1) = x(1) + x(3) * delta_t;
    x_pred(2) = x(2);
    x_pred(3) = x(3);

    return x_pred;
}

// 定义非线性观测函数
VectorXd h(VectorXd x) {
    VectorXd z_pred(nz);
    z_pred(0) = sqrt(x(0) * x(0) + x(1) * x(1));
    z_pred(1) = atan2(x(1), x(0));

    return z_pred;
}

// 定义非线性函数Jacobian矩阵
MatrixXd Jacobian(VectorXd x) {
    double px = x(0);
    double py = x(1);
    double vx = x(2);
    double vy = x(3);

    double c1 = px * px + py * py;
    double c2 = sqrt(c1);
    double c3 = (c1 * c2);

    MatrixXd Hj = MatrixXd::Zero(nz, nx);

    // check division by zero
    if (fabs(c1) < 0.0001) {
        cout << "CalculateJacobian () - Error - Division by Zero" << endl;
        return Hj;
    }

    // compute the Jacobian matrix
    Hj(0, 0) = px / c2;
    Hj(0, 1) = py / c2;
    Hj(1, 0) = -py / c1;
    Hj(1, 1) = px / c1;
    Hj(1, 2) = py * (vx * py - vy * px) / c3;
    Hj(1, 3) = px * (vy * px - vx * py) / c3;

    return Hj;
}

// 定义计算sigma点的函数
MatrixXd compute_sigma_points(VectorXd x, MatrixXd P, double lambda) {
    // compute square root of P
    MatrixXd A = P.llt().matrixL();

    // compute sigma points ...
    MatrixXd Xsig = MatrixXd::Zero(nx, n_sigma);
    Xsig.col(0) = x;

    for (int i = 0; i < nx; i++) {
        Xsig.col(i + 1) = x + sqrt(lambda + nx) * A.col(i);
        Xsig.col(i + 1 + nx) = x - sqrt(lambda + nx) * A.col(i);
    }

    return Xsig;
}

// 定义计算扩展sigma点的函数
MatrixXd compute_augmented_sigma_points(VectorXd x, MatrixXd P, double lambda, double std_a, double std_yawdd) {
    // create augmented mean vector
    VectorXd x_aug = VectorXd::Zero(7);
    x_aug.head(5) = x;
    x_aug(5) = 0;
    x_aug(6) = 0;

    // create augmented state covariance
    MatrixXd P_aug = MatrixXd::Zero(7, 7);
    P_aug.topLeftCorner(5, 5) = P;
    P_aug(5, 5) = std_a * std_a;
    P_aug(6, 6) = std_yawdd * std_yawdd;

    // create sigma point matrix
    MatrixXd Xsig_aug = MatrixXd::Zero(7, n_sigma);
    Xsig_aug.col(0) = x_aug;

    // create square root matrix
    MatrixXd L = P_aug.llt().matrixL();

    // create augmented sigma points
    for (int i = 0; i < 7; i++) {
        Xsig_aug.col(i + 1) = x_aug + sqrt(lambda + 7) * L.col(i);
        Xsig_aug.col(i + 1 + 7) = x_aug - sqrt(lambda + 7) * L.col(i);
    }

    return Xsig_aug;
}

// 定义sigma点预测值函数
MatrixXd predict_sigma_points(MatrixXd Xsig_aug, double delta_t) {
    MatrixXd Xsig_pred = MatrixXd::Zero(nx, n_sigma);

    // predict sigma points
    for (int i = 0; i < n_sigma; i++) {
        // extract values for better readability
        double p_x = Xsig_aug(0, i);
        double p_y = Xsig_aug(1, i);
        double v = Xsig_aug(2, i);
        double yaw = Xsig_aug(3, i);
        double yaw_rate = Xsig_aug(4, i);
        double nu_a = Xsig_aug(5, i);
        double nu_yawdd = Xsig_aug(6, i);

        // predicted state values
        double px_p, py_p;

        // avoid division by zero
        if (fabs(yaw_rate) > 0.001) {
            px_p = p_x + v / yaw_rate * (sin(yaw + yaw_rate * delta_t) - sin(yaw));
            py_p = p_y + v / yaw_rate * (cos(yaw) - cos(yaw + yaw_rate * delta_t));
        } else {
            px_p = p_x + v * delta_t * cos(yaw);
            py_p = p_y + v * delta_t * sin(yaw);
        }

        double v_p = v;
        double yaw_p = yaw + yaw_rate * delta_t;
        double yawd_p = yaw_rate;

        // add noise
        px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
        py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
        v_p = v_p + nu_a * delta_t;

        yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
        yawd_p = yawd_p + nu_yawdd * delta_t;

        // write predicted sigma point into right column
        Xsig_pred(0, i) = px_p;
        Xsig_pred(1, i) = py_p;
        Xsig_pred(2, i) = v_p;
        Xsig_pred(3, i) = yaw_p;
        Xsig_pred(4, i) = yawd_p;
    }

    return Xsig_pred;
}

// 定义计算预测状态均值和协方差矩阵函数
void predict_mean_covariance(VectorXd& x_pred, MatrixXd& P_pred, MatrixXd Xsig_pred) {
    // create vector for weights
    VectorXd weights = VectorXd::Zero(n_sigma);
    weights(0) = lambda / (lambda + nx);

    for (int i = 1; i < n_sigma; i++) {
        weights(i) = 0.5 / (lambda + nx);
    }

    // predict state mean
    x_pred = Xsig_pred * weights;

    // predict state covariance matrix
    P_pred.fill(0.0);

    for (int i = 0; i < n_sigma; i++) {
        // state difference
        VectorXd x_diff = Xsig_pred.col(i) - x_pred;

        // angle normalization
        while (x_diff(3) > M_PI) x_diff(3) -= 2. * M_PI;
        while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;

        P_pred = P_pred + weights(i) * x_diff * x_diff.transpose();
    }
}

// 定义计算预测观测均值和协方差矩阵函数
void predict_measure_mean_covariance(VectorXd& z_pred, MatrixXd& S_pred, MatrixXd Xsig_pred) {
    // transform sigma points into measurement space
    MatrixXd Zsig = MatrixXd::Zero(nz, n_sigma);

    for (int i = 0; i < n_sigma; i++) {
        // extract values for better readibility
        double p_x = Xsig_pred(0, i);
        double p_y = Xsig_pred(1, i);
        double v = Xsig_pred(2, i);
        double yaw = Xsig_pred(3, i);

        // measurement model
        Zsig(0, i) = sqrt(p_x * p_x + p_y * p_y);
        Zsig(1, i) = atan2(p_y, p_x);
    }

    // mean predicted measurement
    z_pred = Zsig * weights;

    // innovation covariance matrix S
    S_pred.fill(0.0);

    for (int i = 0; i < n_sigma; i++) {
        // residual
        VectorXd z_diff = Zsig.col(i) - z_pred;

        // angle normalization
        while (z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
        while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;

        S_pred = S_pred + weights(i) * z_diff * z_diff.transpose();
    }

    // add measurement noise covariance matrix
    S_pred = S_pred + R;
}

// 定义计算卡尔曼增益函数
void calculate_Kalman_gain(MatrixXd& K, MatrixXd& Tc, MatrixXd S_pred, MatrixXd Xsig_pred, VectorXd x_pred, VectorXd z_pred) {
    // calculate cross correlation matrix
    Tc.fill(0.0);

    for (int i = 0; i < n_sigma; i++) {
        // residual
        VectorXd z_diff = Xsig_pred.col(i) - x_pred;

        // angle normalization
        while (z_diff(3) > M_PI) z_diff(3) -= 2. * M_PI;
        while (z_diff(3) < -M_PI) z_diff(3) += 2. * M_PI;

        // state difference
        VectorXd x_diff = Xsig_pred.col(i) - x_pred;

        // angle normalization
        while (x_diff(3) > M_PI) x_diff(3) -= 2. * M_PI;
        while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;

        Tc = Tc + weights(i) * x_diff * z_diff.transpose();
    }

    // calculate Kalman gain K;
    K = Tc * S_pred.inverse();
}

// 定义更新状态估计函数
void update_state(VectorXd& x_pred, MatrixXd& P_pred, VectorXd z, MatrixXd K, VectorXd z_pred) {
    // calculate residual
    VectorXd z_diff = z - z_pred;

    // angle normalization
    while (z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;

    // update state mean and covariance matrix
    x_pred = x_pred + K * z_diff;
    P_pred = P_pred - K * S_pred * K.transpose();
}

int main() {
    // 定义初始状态变量和协方差矩阵
    x << 0, 0, 0, 0;
    P << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 10, 0,
        0, 0, 0, 10;

    // 定义观测变量和噪声协方差矩阵
    z << 0, 0;
    R << 0.1, 0,
        0, 0.1;

    // 定义噪声参数
    double std_a = 0.2;
    double std_yawdd = 0.2;

    // 计算sigma点的权重
    VectorXd weights = VectorXd::Zero(n_sigma);
    weights(0) = lambda / (lambda + nx);

    for (int i = 1; i < n_sigma; i++) {
        weights(i) = 0.5 / (lambda + nx);
    }

    // 仿真时间
    double time = 0;

    // 执行UKF跟踪算法
    while (time < 30) {
        // 定义噪声向量
        VectorXd nu = VectorXd::Zero(2);
        nu(0) = std_a * sqrt(delta_t);
        nu(1) = std_yawdd * sqrt(delta_t);

        // 计算扩展sigma点
        MatrixXd Xsig_aug = compute_augmented_sigma_points(x, P, lambda, std_a, std_yawdd);

        // 预测sigma点的状态均值和协方差矩阵
        MatrixXd Xsig_pred = predict_sigma_points(Xsig_aug, delta_t);

        VectorXd x_pred(nx);
        MatrixXd P_pred(nx, nx);

        predict_mean_covariance(x_pred, P_pred, Xsig_pred);

        // 计算预测观测的均值和协方差矩阵
        VectorXd z_pred(nz);
        MatrixXd S_pred(nz, nz);

        predict_measure_mean_covariance(z_pred, S_pred, Xsig_pred);

        // 计算卡尔曼增益
        MatrixXd K(nx, nz);
        MatrixXd Tc(nx, nz);

        calculate_Kalman_gain(K, Tc, S_pred, Xsig_pred, x_pred, z_pred);

        // 更新状态变量
        VectorXd z_true(nz);
        z_true << sqrt(x(0) * x(0) + x(1) * x(1)),
                  atan2(x(1), x(0));

        update_state(x_pred, P_pred, z_true + nu, K, z_pred);

        // 更新时间
        time += delta_t;
    }

    return 0;
}
