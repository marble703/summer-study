#include<iostream>
#include<cmath>
#include"Quaternion.hpp"

class Pose{
private:
    //姿态 位置
    double x;
    double y;
    double z;

    //姿态 角度
    double phi;
    double theta;
    double psi;

    //旋转 欧拉角
    double d_phi;
    double d_theta;
    double d_psi;

    //旋转 四元数
    double q_w;
    double q_x;
    double q_y;
    double q_z;

    //平移 距离
    double d_x;
    double d_y;
    double d_z;

    /**
     * @brief 坐标系平移
     * 
     * @param xt x方向位移
     * @param yt y方向位移
     * @param zt z方向位移
     * 
     * @return 平移后的Pose
     */
    Pose translate(){
        double n_x = x + d_x;
        double n_y = y + d_y;
        double n_z = z + d_z;

        Pose revolved_pose(n_x, n_y, n_z, phi, theta, psi, q_w, q_x, q_y, q_z, d_x, d_y, d_z);
        return revolved_pose;
    }

    /**
     * @brief 输入四元数，旋转一个向量的位置
     * 
     * @param q_w w方向变换
     * @param q_x x方向变换
     * @param q_y y方向变换
     * @param q_z z方向变换
     * 
     * @return 旋转后的Pose
     */
    Pose revolve_pos(){
        Quaternion p(0, x, y, z); /*三维空间点对应的四元数*/
        Quaternion q(q_w, q_x, q_y, q_z); /*旋转*/
        Quaternion q_nor = q.normalization();/*归一化为单位向量*/
        Quaternion q_nor_v = q_nor.reverse(); /*旋转四元数取逆*/

//        std::cout<<q_nor_v<<" "<<q_nor<<std::endl;

        Quaternion p_n = q_nor * p * q_nor_v; 

        double x_n = p_n.values()[1];
        double y_n = p_n.values()[2];
        double z_n = p_n.values()[3];

        Pose revolved_pose(x_n, y_n, z_n, phi, theta, psi, q_w, q_x, q_y, q_z, d_x, d_y, d_z);

        return revolved_pose;
    }
    /**
     * @brief 输入四元数，旋转一个向量的角度
     */
    Pose revolve_ang(){
        Quaternion quaternion_1(phi, theta, psi);

        Quaternion quaternion_2(d_phi, d_theta, d_psi);

        Quaternion quaternion_ans = quaternion_1 * quaternion_2;

        double* arr = quaternion_ans.Euler_angles();

        double n_phi = arr[0];
        double n_theta = arr[1];
        double n_psi = arr[2];

        Pose revolved_pose(x, y, z, n_phi, n_theta, n_psi, d_phi, d_theta, d_psi, d_x, d_y, d_z);

        return revolved_pose;
    }

public:
    /**
     * @brief 构造函数，输入坐标和角度
     * 
     * @param x, y, z 坐标
     */
    Pose(double x, double y, double z,
        double phi, double theta, double psi) : 
        x(x), y(y), z(z),
        phi(phi), theta(theta), psi(psi){}

    /**
     * @brief 构造函数，使用欧拉角
     * 
     * @param x, y, z 坐标
     * @param d_phi, d_theta, d_psi 旋转欧拉角
     * @param d_x, d_y, d_z 平移
     */
    Pose(double x, double y, double z,
        double phi, double theta, double psi, 
        double d_phi, double d_theta, double d_psi, 
        double d_x, double d_y, double d_z) : 
        x(x), y(y), z(z), 
        phi(phi), theta(theta), psi(psi), 
        d_phi(d_phi), d_theta(d_theta), d_psi(d_psi), 
        d_x(d_x), d_y(d_y), d_z(d_z){

        Quaternion quaternion(d_phi, d_theta, d_psi);
        double* arr = quaternion.values();

        q_w = arr[0];
        q_x = arr[1];
        q_y = arr[2];
        q_z = arr[3];
        }

    /**
     * @brief 构造函数，使用四元数
     * 
     * @param x, y, z 坐标
     * @param q_w, q_x, q_y, q_z 四元数
     * @param d_x, d_y, d_z 平移
     */
    Pose(double x, double y, double z, 
        double phi, double theta, double psi, 
        double q_w, double q_x, double q_y, double q_z, 
        double d_x, double d_y, double d_z) : 
        x(x), y(y), z(z), 
        phi(phi), theta(theta), psi(psi), 
        q_w(q_w), q_x(q_x), q_y(q_y), q_z(q_z), 
        d_x(d_x), d_y(d_y), d_z(d_z){
        
        Quaternion quaternion(q_w, q_x, q_y, q_z);
        double* arr = quaternion.Euler_angles();

        d_phi = arr[0];
        d_theta = arr[1];
        d_psi = arr[2];
        }

    //默认构造函数
    Pose() = default;

    //默认析构函数
    ~Pose() = default;

    /**
     * @brief 修改四元数，同时修改旋转欧拉角
     * 
     * @param n_q_w, n_q_x, n_q_y, n_q_z 直接赋值
     */
    void set_quaternion(double n_q_w, double n_q_x, double n_q_y, double n_q_z){
        q_w = n_q_w;
        q_x = n_q_x;
        q_y = n_q_y;
        q_z = n_q_z;

        Quaternion quaternion(q_w, q_x, q_y, q_z);
        double* arr = quaternion.Euler_angles();

        d_phi = arr[0];
        d_theta = arr[1];
        d_psi = arr[2];
    }

    /**
     * @brief 修改旋转欧拉角，同时修改四元数
     * 
     * @param n_d_phi, n_d_theta, n_d_psi 直接赋值
     */
    void set_Euler_angles(double n_d_phi, double n_d_theta, double n_d_psi){
        d_phi = n_d_phi;
        d_theta = n_d_theta;
        d_psi = n_d_psi;

        Quaternion quaternion(d_phi, d_theta, d_psi);
        double* arr = quaternion.values();

        q_w = arr[0];
        q_x = arr[1];
        q_y = arr[2];
        q_z = arr[3];
    }

    /**
     * @brief 修改位置
     * 
     * @param n_x, n_y, n_z 直接赋值
     */
    void set_position(double n_x, double n_y, double n_z){
        x = n_x;
        y = n_y;
        z = n_z;
    }

    /**
     * @brief 修改位移
     * 
     * @param n_d_x, n_d_y, n_d_z 直接赋值
     */
    void set_displacement(double n_d_x, double n_d_y, double n_d_z){
        d_x = n_d_x;
        d_y = n_d_y;
        d_z = n_d_z;
    }

    /**
     * @brief 修改角度
     * 
     * @param n_x, n_y, n_z 直接赋值
     */
    void set_angle(double n_phi, double n_theta, double n_psi){
        phi = n_phi;
        theta = n_theta;
        psi = n_psi;
    }
    /**
     * @brief 修改位姿,取另一个四元数的值赋给这个四元数的旋转和位移
     */
    void set_transform(Pose pose){
        double*  d_ang_arr = pose.Euler_angles();

        d_phi = d_ang_arr[0];
        d_theta = d_ang_arr[1];
        d_psi = d_ang_arr[2];

        pose.set_Euler_angles(d_phi, d_theta, d_psi);

        double*  d_pos_arr = pose.pos();

        d_x = d_pos_arr[0];
        d_y = d_pos_arr[1];
        d_z = d_pos_arr[2];

        pose.set_displacement(d_x ,d_y, d_z);
    }

    /**
     * @brief 位姿变换，先平移后旋转
     * @brief 通过调用translate()和revolve()实现
     * 
     * @return 平移旋转后的Pose
     */
    Pose t_r(){
        Pose pose(x, y, z, phi, theta, psi, d_phi, d_theta, d_psi, d_x, d_y, d_z);
        
        pose = pose.revolve_pos();
        pose = pose.revolve_ang();
        pose = pose.translate();

        return pose;
    }

    /**
     * @brief 输出位置
     * 
     * @return double[3]数组，包含x, y, z
     */
    double* pos(){
        
        double* arr = new double[3];

        arr[0] = x;
        arr[1] = y;
        arr[2] = z;

        return arr;
    }

    /**
     * @brief 输出角度
     * 
     * @return double[3]数组，包含phi, theta, psi
     */    
    double* Euler_angles(){
        double* arr = new double[3];

        arr[0] = phi;
        arr[1] = theta;
        arr[2] = psi;

        return arr;
    }

    /**
     * @brief 输出旋转角度
     * 
     * @return double[3]数组，包含 d_phi, d_theta, d_psi
     */    
    double* d_Euler_angles(){
        double* arr = new double[3];

        arr[0] = d_phi;
        arr[1] = d_theta;
        arr[2] = d_psi;

        return arr;
    }

    /**
     * @brief 输出位置
     * 
     * @return double[3]数组，包含x, y, z
     */
    double* d_pos(){
        
        double* arr = new double[3];

        arr[0] = d_x;
        arr[1] = d_y;
        arr[2] = d_z;

        return arr;
    }

    friend std::ostream& operator<<(std::ostream& os, const Pose& pose) { //重载提取运算符 >>(输出)
        os << "X: " <<pose.x<< " Y: " << pose.y << " Z: " << pose.z;
        os << "\nraw: " <<pose.phi<< " pitch: " << pose.theta << " roll: " << pose.psi;
//        os << "\nd_phi: " <<pose.d_phi<< " d_theta: " << pose.d_theta << " d_psi: " << pose.d_psi;
//        os << "\nq_w: " <<pose.q_w<< " q_x: " << pose.q_x << " q_y: " << pose.q_y<< " q_z: " << pose.q_z;
//        os << "\nD_X: " <<pose.d_x<< " D_Y: " << pose.d_y << " D_Z: " << pose.d_z;
        os << std::endl;
        return os;
    }

    friend std::istream& operator>>(std::istream& in, Pose& pose) { //重载提取运算符 >>(输出)

        in >> pose.x >> pose.y >> pose.z >> 
              pose.phi >> pose.theta >> pose.psi>>
              pose.d_phi >> pose.d_theta >> pose.d_psi>>
              pose.d_x >> pose.d_y >> pose.d_z;

        return in;
    }

    double* details(){
        double* detail = new double[16];
        detail[0] = x;
        detail[1] = y;
        detail[2] =  z;
        detail[3] =  phi;
        detail[4] =  theta;
        detail[5] =  psi;
        detail[6] =  d_phi;
        detail[7] =  d_theta;
        detail[8] =  d_psi;
        detail[9] =  q_w;
        detail[10] =  q_x;
        detail[11] =  q_y;
        detail[12] =  q_z;
        detail[13] =  d_x;
        detail[14] =  d_y;
        detail[15] =  d_z;

        return detail;
    }
};