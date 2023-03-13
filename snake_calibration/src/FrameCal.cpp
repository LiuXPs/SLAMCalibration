//
// Created by apengse on 23-3-11.
//

#include "SnakeCalibr/FrameCal.h"

FrameCal::FrameCal() {
}

FrameCal::~FrameCal() {
}

void FrameCal::calRelFrame(const std::vector<double> &alpha,
                           const std::vector<double> &a,
                           const std::vector<double> &theta,
                           const std::vector<double> &d) {
    this->alpha_.clear();
    this->a_.clear();
    this->theta_.clear();
    this->d_.clear();

    this->frame_rel_.clear();
    this->frame_rel_abs_.clear();

    this->alpha_.insert(this->alpha_.begin(), alpha.begin(), alpha.end());
    this->a_.insert(this->a_.begin(), a.begin(), a.end());
    this->theta_.insert(this->theta_.begin(), theta.begin(), theta.end());
    this->d_.insert(this->d_.begin(), d.begin(), d.end());

    for (int i = 0; i < this->alpha_.size(); i++) {
        Eigen::Matrix3d rotx(Eigen::AngleAxisd(this->alpha_.at(i), Eigen::Vector3d(1, 0, 0)));
        Eigen::Vector3d transx(this->a_.at(i), 0, 0);
        Eigen::Matrix3d rotz(Eigen::AngleAxisd(this->theta_.at(i), Eigen::Vector3d(0, 0, 1)));
        Eigen::Vector3d transz(0, 0, this->d_.at(i));

        Sophus::SE3<double> Rt_1(rotx, Eigen::Vector3d::Zero());
        Sophus::SE3<double> Rt_2(Eigen::Matrix3d::Identity(), transx);
        Sophus::SE3<double> Rt_3(rotz, Eigen::Vector3d::Zero());
        Sophus::SE3<double> Rt_4(Eigen::Matrix3d::Identity(), transz);

        Sophus::SE3<double> Rt = Rt_1 * Rt_2 * Rt_3 * Rt_4;
        this->frame_rel_.push_back(Rt);
    }

    for (int i = 0; i < this->frame_rel_.size(); i++) {
        if (i == 1) {
            this->frame_rel_abs_.push_back(this->frame_rel_.at(i));
        } else {
            this->frame_rel_abs_.push_back(this->frame_rel_abs_.at(i - 1) * this->frame_rel_.at(i));
        }
    }
}

void FrameCal::calAbsFrame(const Sophus::SE3<double> &base_frame) {
    this->frame_abs_.clear();
    this->base_frame_ = base_frame;

    for (auto &frame: this->frame_rel_abs_) {
        this->frame_abs_.push_back(this->base_frame_ * frame);
    }
}

std::vector<Sophus::SE3<double>> FrameCal::getFrameRel() {
    return this->frame_rel_;
}

std::vector<Sophus::SE3<double>> FrameCal::getFrameAbs() {
    return this->frame_abs_;
}

void FrameCal::updateFrame(const Sophus::SE3<double> &base_frame,
                           const std::vector<double> &alpha,
                           const std::vector<double> &a,
                           const std::vector<double> &theta,
                           const std::vector<double> &d) {
    calRelFrame(alpha, a, theta, d);
    calAbsFrame(base_frame);
}