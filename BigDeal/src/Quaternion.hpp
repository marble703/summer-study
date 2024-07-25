#include<iostream>
#include<cmath>

class Quaternion{
private:
    double w;
    double x;
    double y;
    double z;

public:
    /**
     * @brief 直接使用四元数构造
     */
    Quaternion(double w, double x, double y, double z) : w(w), x(x), y(y),z(z){}

    /**
     * @brief 使用欧拉角构造，转换成四元数
     */
    Quaternion(double phi, double theta, double psi){
        w = cos(phi / 2) * cos(theta / 2) * cos(psi / 2) + sin(phi / 2) * sin(theta / 2) * sin(psi / 2);
        x = sin(phi / 2) * cos(theta / 2) * cos(psi / 2) - cos(phi / 2) * sin(theta / 2) * sin(psi / 2);
        y = cos(phi / 2) * sin(theta / 2) * cos(psi / 2) + sin(phi / 2) * cos(theta / 2) * sin(psi / 2);
        z = cos(phi / 2) * cos(theta / 2) * sin(psi / 2) - sin(phi / 2) * sin(theta / 2) * cos(psi / 2);
    }

    //构造函数(默认)
    Quaternion() = default;

    //析构函数(默认)
    ~Quaternion() = default;

    //重载+
    Quaternion operator+(const Quaternion& other){
        double wt = w + other.w;
        double xt = x + other.x;
        double yt = y + other.y;
        double zt = z + other.z;
        Quaternion quaternion(wt, xt, yt, zt);
        return quaternion;
    }

    //重载*，四元数乘四元数
    Quaternion operator*(const Quaternion& other){
        double wt = w * other.w - x * other.x - y * other.y - z * other.z;
        double xt = w * other.x + x * other.w + y * other.z - z * other.y;
        double yt = w * other.y - x * other.z + y * other.w + z * other.x;
        double zt = w * other.z + x * other.y - y * other.x + z * other.w;
        Quaternion quaternion(wt, xt, yt, zt);
        return quaternion;
    }

    //重载*，四元数乘数
    Quaternion operator*(double other){
        double wt = w * other;
        double xt = x * other;
        double yt = y * other;
        double zt = z * other;
        Quaternion quaternion(wt, xt, yt, zt);
        return quaternion;
    }

    //重载/，四元数除以数
    Quaternion operator/(double other){
        double wt = w / other;
        double xt = x / other;
        double yt = y / other;
        double zt = z / other;

        Quaternion quaternion(wt, xt, yt, zt);
        return quaternion;
    }

    /**
     * @brief 归一化
     * 
     * @return 归一化后的四元数
     */
    Quaternion normalization(){
        Quaternion quaternion(w, x, y, z);
        double mod = quaternion.mod();

        quaternion = quaternion / mod;

        return quaternion;
    }

    /**
     * @brief 取模
     * 
     * @return 模长
     */
    double mod(){
        return sqrt(pow(w, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2));
    }

    //共轭
    Quaternion conjugate(){
        Quaternion quaternion(w, -x, -y, -z);
        return quaternion;
    }

    //求逆
    Quaternion reverse(){
        Quaternion quaternion(w, x, y, z);
        double mod = quaternion.mod();
        Quaternion quaternion_c = quaternion.conjugate();
        Quaternion quaternion_v = quaternion_c / (mod * mod);
        return quaternion_v;
    }

    //旋转
    Quaternion revolve(double theta){
        Quaternion quaternion(w, x, y, z);
        w = cos(theta / 2);
        x = sin(theta / 2);
        y = sin(theta / 2);
        z = sin(theta / 2);

        return quaternion;
    }

    //输入
    friend std::istream& operator>>(std::istream& in, Quaternion& quaternion) { //重载提取运算符 >>(输入)
        in >> quaternion.w >> quaternion.x >> quaternion.y >> quaternion.z;
        return in;
    }

    //输出
    friend std::ostream& operator<<(std::ostream& os, const Quaternion& quaternion) { //重载提取运算符 >>(输出)
        os << quaternion.w << " " << quaternion.x << " i " << quaternion.y << " j " << quaternion.z << " k";
        os << std::endl;
        return os;
    }

    /**
     * @brief 输出欧拉角
     * 
     * @return double*[3]的一个数组，存储phi,theta,psi
     */
    double* Euler_angles(){
        double* matrix = new double[3];
        double phi;
        double theta;
        double psi;

        phi = atan2(2 * (w * x + y * z), (1 - 2 * (pow(x, 2) + pow(y, 2))));
        theta = asin(2 * (w * y - x * z));
        psi = atan2(2 * (w * z + x * y), (1 - 2 * (pow(y, 2) + pow(z, 2))));
        
        matrix[0] = phi;
        matrix[1] = theta;
        matrix[2] = psi;

        return matrix;
    }

    /**
     * @brief 计算旋转矩阵
     * 
     * @return double[3][3]数组
     */
    double** revolve_matrix(){
        double** matrix = new double*[3];  

        for (int i = 0; i < 3; ++i) {  
            matrix[i] = new double[3];
            }
        matrix[0][0] = 1 - 2 * pow(y, 2) - 2 * pow(z, 2);
        matrix[0][1] = 2 * x * y - 2 * w * z;
        matrix[0][2] = 2 * x * z + 2 * w * y;
        matrix[1][0] = 2 * x * y + 2 * w * z;
        matrix[1][1] = 1 - 2 * pow(x, 2) - 2 * pow(z, 2);
        matrix[1][2] = 2 * y * z - 2 * w * x;
        matrix[2][0] = 2 * x * z - 2 * w * y;
        matrix[2][1] = 2 * y * z + 2 * w * z;
        matrix[2][2] = 1 - 2 * pow(x, 2) - 2 * pow(y, 2);

/* double revolve_matrix[3][3] = {
            {1 - 2 * pow(y, 2) - 2 * pow(z, 2), 2 * x * y - 2 * w * z, 2 * x * z + 2 * w * y}, 
            {2 * x * y + 2 * w * z, 1 - 2 * pow(x, 2) - 2 * pow(z, 2), 2 * y * z - 2 * w * x}, 
            {2 * x * z - 2 * w * y, 2 * y * z + 2 * w * z, 1 - 2 * pow(x, 2) - 2 * pow(y, 2)}}; */
        
        return matrix;
    }

    /**
     * @brief 输出w, x, y, z
     * 
     * @return double[4]数组
     */
    double* values(){
        double* arr = new double[4];

        arr[0] = w;
        arr[1] = x;
        arr[2] = y;
        arr[3] = z;

        return arr;
    }
};