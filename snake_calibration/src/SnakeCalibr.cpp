//
// Created by apengse on 23-3-11.
//
#include "SnakeCalibr/SnakeCalibr.h"

int main() {
    Eigen::Matrix3d rotx(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(1, 0, 0)));
    Eigen::Matrix3d roty(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(0, 1, 0)));
    Eigen::Matrix3d rotz(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(0, 0, 1)));

    std::cout << "rotx: " << rotx.matrix() << std::endl;
    std::cout << "roty: " << roty.matrix() << std::endl;
    std::cout << "rotz: " << rotz.matrix() << std::endl;

    std::cout << "id 3: " << Eigen::Matrix3d::Identity().matrix() << std::endl;
    std::cout << "zev 3: " << Eigen::Vector3d::Zero().matrix() << std::endl;
    return 0;
}