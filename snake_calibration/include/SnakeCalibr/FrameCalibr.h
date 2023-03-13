//
// Created by apengse on 23-3-11.
//

#ifndef FRAMECALIBR_H
#define FRAMECALIBR_H

#include <iostream>
#include <opencv2/calib3d.hpp>

#include "FrameCal.h"

class FrameCalibr {
public:
    FrameCalibr();

    virtual ~FrameCalibr();

private:
    double link_l_; // The length of each link.
    int link_n_; // The number of link.
    Sophus::SE3<double> base_frame_;

    std::vector<double> alpha_;
    std::vector<double> a_;
    std::vector<double> theta_;
    std::vector<double> d_;

    FrameCal frame_cal_;

    std::vector<Sophus::SE3<double>> frame_rel_;
    std::vector<Sophus::SE3<double>> frame_abs_;

    std::vector<Eigen::Vector4d> joint_local_; //The joint position is at the origin of the coordinate system.
    std::vector<Eigen::Vector4d> joint_global_;
    std::vector<Eigen::Vector4d> centr_local_; //The centroid of each link is in the middle.
    std::vector<Eigen::Vector4d> centr_global_;
private:
    void calPosition();

public:
    void updateTheta(const std::vector<double> &);
};

#endif //FRAMECALIBR_H