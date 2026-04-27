#pragma once

#include <Eigen/Dense>
#include <iostream>

class IMUIntegrator
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    IMUIntegrator() {
        Reset();
    }

    ~IMUIntegrator() = default;

    void Reset(){
        initialized_ = false;
        init_count_ = 0;
        mean_acc_ = Eigen::Vector3d::Zero();
        mean_gyr_ = Eigen::Vector3d::Zero();

        rot_ = Eigen::Matrix3d::Identity();
        pos_ = Eigen::Vector3d::Zero();
        vel_ = Eigen::Vector3d::Zero();

        bg_ = Eigen::Vector3d::Zero();
        ba_ = Eigen::Vector3d::Zero();
        gravity_ = Eigen::Vector3d(0, 0, -9.81);
        last_time_ = -1.0;
    }

    void Predict(double current_time, const Eigen::Vector3d &acc_raw, const Eigen::Vector3d &gyr_raw) {
        // 1. initialization first 20 frames
        if (!initialized_) {
            mean_acc_ += acc_raw;
            mean_gyr_ += gyr_raw;
            init_count_++;

            if (init_count_ >= 20) {
                mean_acc_ /= float(init_count_);
                mean_gyr_ /= float(init_count_);

                double gravity_norm = 9.81;
                gravity_ = -mean_acc_ / mean_acc_.norm() * gravity_norm;

                bg_ = mean_gyr_;

                initialized_ = true;
                last_time_ = current_time;
                std::cout << "IMU initialized. Gravity: " << gravity_.transpose() << ", Gyro bias: " << bg_.transpose() << std::endl;
            }
            return;
        }

        // 2. predict
        double dt = current_time - last_time_;
        last_time_ = current_time;
        if(dt <= 0) {
            std::cerr << "Non-positive time difference: " << dt << std::endl;
            return;
        }

        Eigen::Vector3d unbias_acc = acc_raw - ba_;
        Eigen::Vector3d unbias_gyr = gyr_raw - bg_;

        // Update rotation
        Eigen::Matrix3d dR = Exp(unbias_gyr * dt);
        Eigen::Matrix3d new_rot = rot_ * dR;
        // Update velocity and position
        Eigen::Vector3d global_acc = rot_ * unbias_acc + gravity_;
        Eigen::Vector3d new_pos = pos_ + vel_ * dt + 0.5 * global_acc * dt * dt;
        Eigen::Vector3d new_vel = vel_ + global_acc * dt;

        new_vel *= 0.98;

        // Update state
        rot_ = new_rot;
        pos_ = new_pos;
        vel_ = new_vel;
    }

    Eigen::Affine3f GetCurrentPose() const {
        Eigen::Affine3f pose = Eigen::Affine3f::Identity();
        pose.linear() = rot_.cast<float>();
        pose.translation() = pos_.cast<float>();
        return pose;
    }

private:
    Eigen::Matrix3d Exp(const Eigen::Vector3d &ang_vel) {
        double d = ang_vel.norm();
        if (d < 1e-5) return Eigen::Matrix3d::Identity();
        Eigen::Vector3d axis = ang_vel / d;
        return Eigen::AngleAxisd(d, axis).toRotationMatrix();
    }

    bool initialized_ = false;
    int init_count_ = 0;
    double last_time_ = -1.0;
    Eigen::Vector3d mean_acc_;
    Eigen::Vector3d mean_gyr_;
    Eigen::Matrix3d rot_;
    Eigen::Vector3d pos_;
    Eigen::Vector3d vel_;
    Eigen::Vector3d bg_;
    Eigen::Vector3d ba_;
    Eigen::Vector3d gravity_;
};
