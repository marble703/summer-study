#include<iostream>
#include <vector>
#include <string>

#include"Pose_dist.hpp"

int main(int argc, char** argv){
    Quaternion a(1,2,3,4);
    Quaternion b(0,1,2,3);
    Quaternion c(0.1, 0.1, 0.1);
//    std::cout<<a<<std::endl;

    double* arr = a.Euler_angles();
//    std::cout<<arr[0]<<" "<<arr[1]<<" "<<arr[2]<<std::endl;
    
//    std::cout<<c<<std::endl;

    std::cout<<"Camera_to_Gimbal"<<std::endl;

    Pose pose_o_g(0.7,1.3,0,
            0,0,0,
            0,0,0,
            0,0,0);

    Pose pose_g_c(0.2,0,0,
            -1.57,0,-1.57,
            0,0,0,
            0,0,0);

    Pose pose_g_s(0.18,0,0,
            0,0,0,
            0,0,0,
            0,0,0);

    Pose pose_c_a(0.7,-0.13,1.9,
            1.1,-0.4,0.0,
            0,0,0,
            0,0,0);
    
    std::cout<< "from camera to armor"<<std::endl;
    std::cout<< pose_c_a.t_r()<<std::endl;

    std::cout<< "from gimbal to armor"<<std::endl;
    pose_g_c.set_transform(pose_c_a);

/*
    Pose pose_g_a(0.2,0,0,
            -1.57,0,-1.57,
            1.1,-0.4,0,
            0.7,-0.13,1.9);

    std::cout<< pose_g_a.t_r()<<std::endl;
 */



    std::cout<< pose_g_c.t_r()<<std::endl;


/*

    Pose_dist pose_dist;
    int len;

    std::vector<std::string> names;
    std::vector<Pose> poses;

    auto it_name = names.begin();
    auto it_pose = poses.begin();

    std::string name;
    Pose pose;

//    pose_dist.set_pose();



    std::cin>>len>>std::endl;

    for (int i = 0; i++; i< len){

        std::cout<<"name"<<i<<":"<<std::endl;
        std::cin>>name;

        std::cout<<"pose"<<i<<":"<<std::endl;
        std::cin>>pose;

        pose_dist.set_pose(name, pose);
    }

    std::vector<std::string> tran_names;
    Pose ans_pose = pose_dist.tran_pose(tran_names);

    std::cout<<"pose:"<<ans_pose<<std::endl;
*/
    return 0;
}