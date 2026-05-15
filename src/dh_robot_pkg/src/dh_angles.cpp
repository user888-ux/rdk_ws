#include <kdl/frames.hpp>
#include <vector>
#include <array>
#include <stdexcept>
#include <cmath>

std::pair<double,double> count_angles(double pos_wid,double pos_height)//无人机在图像中的实时位置
{
    //----------------- 参数设置区
    // 图像分辨率
    int width=640;
    int height=320;

    // base_link -> yaw 变换矩阵
    KDL::Frame base_link_to_yaw(
        KDL::Rotation::RPY(0.0, 0.0, -2.3561999999999999),
        KDL::Vector(0.0053420000000000004, 0.0, 0.069900000000000004)
    );

    // yaw -> pitch 
    KDL::Frame yaw_to_pitch(
        KDL::Rotation::RPY(0.0, -0.67154000000000003, 0.26890000000000003),
        KDL::Vector(0.082627999999999993, -0.036354999999999998, 0.057200000000000001)
    );

    // pitch -> camera
    KDL::Frame pitch_to_camera(
        KDL::Rotation::RPY(-1.5708, 0.0, -1.5708),
        KDL::Vector(0.03007, 0.059999999999999998, 0.00034649000000000002)
    );

    //相机内参K


    //----------------- 开始写你的算法









    //----------------- 

    //返回计算结果
    double angle1=0;
    double angle2=0;

    std::pair<double,double>p_res;
    p_res.first=angle1;
    p_res.second=angle2;
    return p_res;
}