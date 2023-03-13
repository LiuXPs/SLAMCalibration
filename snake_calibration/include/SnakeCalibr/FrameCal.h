//
// Created by apengse on 23-3-11.
//

#ifndef FRAMECAL_H
#define FRAMECAL_H

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/so3.hpp>
#include <sophus/se3.hpp>

class FrameCal {
public:
    FrameCal();

    virtual ~FrameCal();

private:
    std::vector<double> alpha_;
    std::vector<double> a_;
    std::vector<double> theta_;
    std::vector<double> d_;

    Sophus::SE3<double> base_frame_;

    std::vector<Sophus::SE3<double>> frame_rel_;
    std::vector<Sophus::SE3<double>> frame_rel_abs_;
    std::vector<Sophus::SE3<double>> frame_abs_;
private:
    void calRelFrame(const std::vector<double> &,
                     const std::vector<double> &,
                     const std::vector<double> &,
                     const std::vector<double> &);

    void calAbsFrame(const Sophus::SE3<double> &);

public:
    std::vector<Sophus::SE3<double>> getFrameRel();

    std::vector<Sophus::SE3<double>> getFrameAbs();

    void updateFrame(const Sophus::SE3<double> &base_frame,
                     const std::vector<double> &alpha,
                     const std::vector<double> &a,
                     const std::vector<double> &theta,
                     const std::vector<double> &d);
};

#endif //FRAMECAL_H
