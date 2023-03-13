//
// Created by apengse on 23-3-11.
//

#include "SnakeCalibr/FrameCalibr.h"
#include <cmath>

FrameCalibr::FrameCalibr() {
    this->link_l_ = 0.16; //Unit: m.
    this->link_n_ = 5;
    this->base_frame_ = Sophus::SE3<double>(Eigen::Matrix3d::Identity(),
                                            Eigen::Vector3d::Zero());

    this->joint_local_.clear();
    this->centr_local_.clear();
    for (int i = 0; i < this->link_n_; i++) {
        this->joint_local_.emplace_back(0, 0, 0, 1);
        this->centr_local_.emplace_back(this->link_l_ / 2, 0, 0, 1);
    }

    this->alpha_.clear();
    this->a_.clear();
    this->theta_.clear();
    this->d_.clear();
    for (int i = 0; i < this->link_n_; i++) {
        if (i == 0) {
            this->alpha_.push_back(0);
            this->a_.push_back(0);
            this->theta_.push_back(0);
            this->d_.push_back(0);
        } else {
            this->alpha_.push_back(pow(-1, i) * M_PI_2);
            this->a_.push_back(this->link_l_);
            this->theta_.push_back(pow(-1, i) * 0);
            this->d_.push_back(0);
        }
    }

    calPosition();
}

FrameCalibr::~FrameCalibr() {

}

void FrameCalibr::calPosition() {
    this->frame_cal_.updateFrame(this->base_frame_,
                                 this->alpha_,
                                 this->a_,
                                 this->theta_,
                                 this->d_);

    this->frame_rel_.clear();
    this->frame_abs_.clear();
    this->frame_rel_ = this->frame_cal_.getFrameRel();
    this->frame_abs_ = this->frame_cal_.getFrameAbs();

    // Compute each joint position and centroid position in global coordinate system.
    this->joint_global_.clear();
    this->centr_global_.clear();
    for (int i = 0; i < this->link_n_; i++) {
        this->joint_global_.push_back(this->frame_abs_.at(i) * this->joint_local_.at(i));
        this->centr_global_.push_back(this->frame_abs_.at(i) * this->centr_local_.at(i));
    }
}

void FrameCalibr::updateTheta(const std::vector<double> &joint_angle) {
    this->theta_.clear();
    for (int i = 0; i < joint_angle.size(); i++) {
        this->theta_.push_back(pow(-1, i) * joint_angle.at(joint_angle.size() - i - 1));
    }

    calPosition();
}