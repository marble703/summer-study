#include<iostream>
#include<cmath>
#include <vector>
#include <string>
#include"Pose.hpp"

class Pose_dist
{
private:
    std::vector<Pose> pose_data;
    std::vector<std::string> pose_name;

    std::vector<Pose>::iterator it_pose_data = pose_data.begin();
    std::vector<std::string>::iterator it_pose_name = pose_name.begin();

public:
    Pose_dist() = default;
    ~Pose_dist() = default;

    void set_pose(std::string name, Pose pose){
        auto it_pose_name2 = pose_name.insert(it_pose_name + 1, name);
        auto it_pose_data2 = pose_data.insert(it_pose_data + 1, pose);

        it_pose_name += 1;
        it_pose_data += 1;
    }
    
    /**
     * @brief 读取名称为name的位姿
     * 
     * @param name 需要读取的位姿的名称
     */
    Pose read_pose(std::string name){
        auto it_n = pose_name.begin();
        auto it_d = pose_data.begin();

        while (it_n != pose_name.end() && *it_n != name){
            it_pose_name += 1;
            it_pose_data += 1;
        }
        return *it_d;
    }

    Pose tran_pose(std::vector<std::string> name){
        auto it_name = name.begin();
        auto it_pose_name = pose_name.begin();
        auto it_pose_data = pose_data.begin();

        Pose first_pose = *(it_pose_data);

        Pose tran_pose;
        Pose ans_pose;

         while (it_pose_name != pose_name.end() ){
            if(*it_name == *it_pose_name){
                tran_pose = read_pose(*it_pose_name);

                first_pose.set_transform(tran_pose);

                ans_pose = tran_pose.t_r();

                it_name +=1;
            }
            it_pose_name += 1;
            it_pose_data += 1;
        }
        return ans_pose;
    }
};

