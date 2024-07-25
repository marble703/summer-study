#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <time.h>

#include "src/Armor.hpp"
#include "src/number_classifier.hpp"
#include "src/process.hpp"
#include "src/Detect.hpp"
#include "eigen-3.4.0/Eigen/Dense"
#include "src/Predict.hpp"

#include <fstream>


int main() {
    //cv::Mat image = cv::imread("/media/chen/Data/programme/Visual/SummerStudy/BigDeal/11.png");
    //cv::Mat image = cv::imread("/media/chen/Data/programme/Visual/SummerStudy/pnp/detect/two_armor.png");

    //Detect detector(image);
    //detector.detect(true);

        cv::VideoCapture capture;
        cv::Mat frame;
        std::vector<int64_t> frame_time;

        std::vector<cv::Point2f> points;
        std::pair<Eigen::MatrixXd, Eigen::MatrixXd> rot_tra_pair;
        std::vector<std::pair<Eigen::MatrixXd, Eigen::MatrixXd>> rot_tra_data;

        //p1
        std::pair<Eigen::MatrixXd, Eigen::MatrixXd> rotv_trav_pair;
        std::vector<std::pair<Eigen::MatrixXd, Eigen::MatrixXd>> rotv_trav_data;
        Eigen::MatrixXd rot_m;
        std::vector<Eigen::MatrixXd> rot_m_data;
        

        std::vector<Eigen::MatrixXd> centers;
        std::vector<double> radiuses; //半径
        std::vector<int> fs; //帧记数

        int data_size; int stateSize; int measSize; int controlSize; 
        stateSize = 6; measSize = 3; controlSize = 0;

        Eigen::MatrixXd data; Eigen::MatrixXd A; Eigen::MatrixXd B; Eigen::MatrixXd H; 
        Eigen::MatrixXd P; Eigen::MatrixXd R; Eigen::MatrixXd Q;
        Eigen::VectorXd X0(stateSize);
        Eigen::VectorXd res(stateSize);
		Eigen::VectorXd z(measSize);
		z.setZero();

        Eigen::MatrixXd center;

        //p1
        double radius1;
        std::vector<double> radiuses1;
        Eigen::MatrixXd center1;
        std::vector<Eigen::MatrixXd> centers1;

        double radius;
        double t;

        A.resize(stateSize, stateSize);
        B.resize(stateSize, 1);
        H.resize(measSize, stateSize);

        P.resize(stateSize, stateSize);
        R.resize(measSize, measSize);
        Q.resize(stateSize, stateSize);
        

        // 暂存的A，用不到
        A << 1, 0, 0, 0.01, 0, 0,
             0, 1, 0, 0, 0.01, 0,
             0, 0, 1, 0, 0, 0.01,
             0, 0, 0, 1, 0, 0,
             0, 0, 0, 0, 1, 0,
             0, 0, 0, 0, 0, 1;

        B << 0, 0, 0, 0, 0, 0;

        H << 1, 0, 0, 0, 0, 0,
             0, 1, 0, 0, 0, 0,
             0, 0, 1, 0, 0, 0;

        P << 100, 0, 0, 0, 0, 0,
             0, 100, 0, 0, 0, 0,
             0, 0, 100, 0, 0, 0,
             0, 0, 0, 100, 0, 0,
             0, 0, 0, 0, 100, 0,
             0, 0, 0, 0, 0, 100;

        R << 1000, 0, 0,
             0, 1000, 0,
             0, 0, 1000;

        Q << 0.1, 0, 0, 0, 0, 0,
             0, 0.1, 0, 0, 0, 0,
             0, 0, 0.1, 0, 0, 0,
             0, 0, 0, 0.1, 0, 0,
             0, 0, 0, 0, 0.1, 0,
             0, 0, 0, 0, 0, 0.1;

        X0 << 0, 0, 0, 0, 0, 0;

        KalmanFilter kf(stateSize, measSize, controlSize);
        

        bool init = true;
        
        int test0 = 0;

        capture.open("/media/chen/Data/programme/Visual/SummerStudy/BigDeal/source/7月14日.mp4");
        while (capture.read(frame)) {
            std::cout<< "frame start" <<frame_time.size()<< std::endl;
            auto start = std::chrono::system_clock::now();
            //cv::imshow("origin", frame);
            cv::Mat image = frame;
            Detect detector(frame);
            // 输入true单帧播放
            detector.detect(true);
            cv::imshow("frame", frame);
            //std::cout << "points: " << detector.get_points() << std::endl;




            bool find = detector.get_find();

            if(detector.get_points().size() > 0 && find) {
                points = detector.get_points();
                Pnp pnp(image, points);

                rot_tra_pair = pnp.get_rotm_trav();
                rot_tra_data.push_back(rot_tra_pair);

                //p1
                rotv_trav_pair = pnp.get_rotv_trav();
                rotv_trav_data.push_back(rotv_trav_pair);
                rot_m = pnp.get_rotm();
                rot_m_data.push_back(rot_m);


                fs.push_back(frame_time.size());
                std::cout << "rot_tra_pair: " << Eigen::MatrixXd(rot_tra_pair.first) << std::endl;
                std::cout << "rot_tra_pair: " << Eigen::MatrixXd(rot_tra_pair.second) << std::endl;
                std::cout << "rot_tra_pair0: " << Eigen::MatrixXd(rot_tra_data[0].first) << std::endl;
                std::cout << "rot_tra_pair0: " << Eigen::MatrixXd(rot_tra_data[0].second) << std::endl;
            }
            std::cout << "rot_tra_data.size(): " << rot_tra_data.size() << std::endl;
            if(rot_tra_data.size() == 2) {

                if(rot_tra_data[0] == rot_tra_data[1]){
                    test0++;
                    std::cout << "test0: " << test0 << std::endl;
                    rot_tra_data.erase(rot_tra_data.begin());

                    // p1
                    rotv_trav_data.erase(rotv_trav_data.begin());

                    continue;
                }

                /*
                std::cout << "rot_tra_data0: " << Eigen::MatrixXd(rot_tra_data[0].first) << std::endl;
                std::cout << "rot_tra_data0: " << Eigen::MatrixXd(rot_tra_data[0].second) << std::endl;
                std::cout << "rot_tra_data1: " << Eigen::MatrixXd(rot_tra_data[1].first) << std::endl;
                std::cout << "rot_tra_data1: " << Eigen::MatrixXd(rot_tra_data[1].second) << std::endl;
                */

                Prediction prediction(rot_tra_data);
                center = prediction.find_center();
                std::cout << "center: " << center << std::endl;
                centers.push_back(center);

                //p1
                Prediction_1 prediction1(rotv_trav_data);
                radius1 = prediction1.rad();
                prediction1.set_rm(rot_m_data[0], rot_m_data[1]);
                center1 = prediction1.find_center();
                centers1.push_back(center1);
                radiuses1.push_back(radius1);
                std::cout << "radiuses1: " << radius1 << std::endl;


                double dt = fs[1] - fs[0]; 

                radius = prediction.find_radius();
                radiuses.push_back(radius);
                std::cout << "radiuses: " << radius << std::endl;

                std::cout<< "init: " << init << std::endl;


                // 初始化滤波器
                if(init){
                    std::cout << "dt: " << dt << std::endl;

                    A << 1, 0, 0, dt, 0, 0,
                        0, 1, 0, 0, dt, 0,
                        0, 0, 1, 0, 0, dt,
                        0, 0, 0, 1, 0, 0,
                        0, 0, 0, 0, 1, 0,
                        0, 0, 0, 0, 0, 1;

                    kf.init(X0, P, R, Q);
                    std::cout << "X0: " << X0 << std::endl;
                    init = false;
                    // 擦除第一个
                    rot_tra_data.erase(rot_tra_data.begin());
                    fs.erase(fs.begin());
                    centers.erase(centers.begin());
                    centers1.erase(centers1.begin());

                    //p1
                    rotv_trav_data.erase(rotv_trav_data.begin());
                    }
                // 预测和更新
                else{
                    //std::cout << "kf.predict(A): " << kf.predict(A) << std::endl;
                    std::cout << "dt: " << dt << std::endl;

                    res << kf.predict(A);
                    std::cout<< "res:" << res << std::endl;
                    

                    /*
                    for (int j = 0; j < measSize; j++){
                        //std::cout << "j: " << j << std::endl;
                        //std::cout << "data(i,j): " << data(i,j) << std::endl;
                        std::cout << "zj: " << centers[0](j) << std::endl;
                        z << centers[0](j);
                        std::cout << "zj: " << z << std::endl;
                    }
                    */
                    z << centers1[0];
                    std::cout << "z: " << z << std::endl;
                    kf.update(H,z);
                    std::cout<< "update:" << res << std::endl;
                    // 擦除第一个，半径不擦除
                    rot_tra_data.erase(rot_tra_data.begin());
                    fs.erase(fs.begin());
                    centers.erase(centers.begin());

                    // p1
                    rotv_trav_data.erase(rotv_trav_data.begin());
                    }
                }




            std::cout << "\nframe finish\n\n\n" << std::endl;
            auto end = std::chrono::system_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count(); 

            std::cout <<"frame time: "<< duration << std::endl;
            frame_time.push_back(duration);
            cv::waitKey(1);
        }

        double frame_time_t = 0;
        for (int i = 0; i < frame_time.size(); i++) {
            frame_time_t += frame_time[i];
        }

        std::cout << "average time: " << frame_time_t / frame_time.size() << std::endl;
        std::cout << "DFPS: " << 1000 / (frame_time_t / frame_time.size()) << std::endl;
        double radiuses_t = 0;

        //p1
        double radiuses1_t = 0;
/*

*/
        for (int i = 0; i < radiuses.size(); i++) {
            std::cout << "radius: " << radiuses[i] << std::endl;
            std::cout << "radius1: " << radiuses1[i] << std::endl;

            radiuses_t += radiuses[i];
            radiuses1_t += radiuses1[i];


            // ...

            std::ofstream outfile("/media/chen/Data/programme/Visual/SummerStudy/BigDeal/output.txt");
            if (outfile.is_open()) {
                for (int i = 0; i < radiuses.size(); i++) {
                    outfile << radiuses[i] << " " << radiuses1[i] << "\n";
                }
                outfile.close();
            } else {
                std::cout << "Unable to open file";
            }
        }
        std::cout << "radiuses_t: " << radiuses_t / radiuses.size() << std::endl;
        std::cout << "radiuses1_t: " << radiuses1_t / radiuses1.size() << std::endl;

    return 0;
}