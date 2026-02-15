#ifndef VIMU_FILTER_HPP
#define VIMU_FILTER_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <vector>
#include <array>
#include "structs.hpp"

struct IMU_geometry {
    Eigen::Vector3d position;        //Position meters
    Eigen::Matrix3d rotational_matrix;        //Rotational matrix
    Eigen::Matrix3d skewed_position_matrix;   //Skew symetric matrix
};

//An adaptive kalman filter for multi IMU fusion giving one more accurate virtual IMU
//Based on information from https://www.mdpi.com/1424-8220/11/7/6771#Processing_Speed_of_Architectures_and_Number_of_IMUs, and ChatGPT is used iterativly for 
//understanding the contents of the paper, and how to implement it.
//Have added lots of references to the paper to make the code understandable for myself, and easier understandable for others ;-b
//Ideally the IMUs should be time synchronized. 
class VIMU_filter{
public:
    VIMU_filter(const unsigned int IMU_count): x_(Eigen::VectorXd::Zero(9)), p_(Eigen::MatrixXd::Identity(9, 9)*10), q_(Eigen::MatrixXd::Identity(9, 9)*0.1), 
                    r_((Eigen::MatrixXd(6, 6) << 
                    acc_var_x, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, acc_var_y, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, acc_var_z, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, gyro_var_x, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, gyro_var_y, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, gyro_var_z
                    ).finished())
    {
        IMUs.resize(IMU_count);
    }

    void set_imu_geometry(const Eigen::Vector3d& position, const Eigen::Quaterniond& rotation, const unsigned int id)
    {
        IMUs[id].rotational_matrix = rotation.toRotationMatrix();
        IMUs[id].position = position;
        IMUs[id].skewed_position_matrix = skew(position);
    }

    void update(const std::vector<Eigen::Vector3d>& acc, const std::vector<Eigen::Vector3d>& gyro, const std::vector<int>& imu_ids)
    {
        //Needs more than 2 IMUs to compute as explained in the paper https://www.mdpi.com/1424-8220/11/7/6771#Processing_Speed_of_Architectures_and_Number_of_IMUs section 2.1
        if (acc.size() <= 2)
        {
            return;
        }

        //Stacking matricess as explained in section 3 of the paper
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6*acc.size(), 9);
        Eigen::VectorXd Z(6*acc.size());
        Eigen::MatrixXd R_total = Eigen::MatrixXd::Zero(6*acc.size(), 6*acc.size());

        //Goes through all the IMUs
        for(int i = 0; i < acc.size(); i++) {
            int IMU_id = imu_ids[i];

            Eigen::MatrixXd H_i = Eigen::MatrixXd::Zero(6,9);
            H_i.block<3,3>(3,3) = IMUs[IMU_id].rotational_matrix;

            //Section 2.1 of the paper. Linearization for the H matrix
            Eigen::Matrix3d omega_skew = skew(x_.segment<3>(3));
            Eigen::Matrix3d omega_cross_position_skew = skew(x_.segment<3>(3).cross(IMUs[IMU_id].position));
            Eigen::Matrix3d r_skew = IMUs[IMU_id].skewed_position_matrix;
            Eigen::Matrix3d linearization = omega_skew * r_skew + omega_cross_position_skew;

            //Second line of equation 3 in the paper
            H_i.block<3,3>(0,0) = IMUs[IMU_id].rotational_matrix;           //linear acc effect
            H_i.block<3,3>(0,3) = -IMUs[IMU_id].rotational_matrix * linearization;      //Rotational speed effect 
            H_i.block<3,3>(0,6) = -IMUs[IMU_id].rotational_matrix * r_skew; //Rotational acceleration effect

            //Predictions (implementation of equation 2 in the paper)
            Eigen::Vector3d gyro_pred = IMUs[IMU_id].rotational_matrix * x_.segment<3>(3);
            Z.segment<3>(i*6+3) = gyro[i] - gyro_pred;
            Eigen::Vector3d acc_pred = IMUs[IMU_id].rotational_matrix * 
                (x_.segment<3>(0) +                         //Predicted linear acc
                x_.segment<3>(6).cross(IMUs[IMU_id].position) +               //Predicted linear acc from tangential acc
                x_.segment<3>(3).cross(x_.segment<3>(3).cross(IMUs[IMU_id].position)));       //Predicted linear acc from sentripital acc
            Z.segment<3>(i*6) = acc[i] - acc_pred;

            //Stacks H_i to the H matrix 
            H.block(i*6, 0, 6, 9) = H_i;
            R_total.block(i*6, i*6, 6, 6) = r_;
        }

        update_adaptive_noise();
        p_ = p_ + q_;

        //Updates kalman gain and state
        Eigen::MatrixXd S = H * p_ * H.transpose() + R_total;
        Eigen::MatrixXd K = p_ * H.transpose() * S.inverse();
        x_ = x_ + K * Z;
        
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(9, 9);
        p_ = (I - K * H) * p_ * (I - K * H).transpose() + K * R_total * K.transpose();
    }

    Eigen::Vector3d get_acceleration()
    {
        return x_.segment<3>(0);
    }

    Eigen::Vector3d get_gyro()
    {
        return x_.segment<3>(3);
    }

private:
    void update_adaptive_noise()
    {
        Eigen::Vector3d acc = x_.segment<3>(0);
        Eigen::Vector3d omega = x_.segment<3>(3);
        //Based on section 2.2 of the paper
        std::rotate(omega_buffer_.begin(), omega_buffer_.begin() + 1, omega_buffer_.end());
        std::rotate(acc_buffer_.begin(), acc_buffer_.begin() + 1, acc_buffer_.end());

        omega_buffer_.back() = omega;
        acc_buffer_.back() = acc;

        //Calculates the average
        Eigen::Vector3d average_omega = Eigen::Vector3d::Zero();
        Eigen::Vector3d average_acc = Eigen::Vector3d::Zero();
        for (int i = 0; i < 10; i++) {
            average_omega += omega_buffer_[i];
            average_acc += acc_buffer_[i];
        }
        average_omega /= 10.0f;
        average_acc /= 10.0f;

        //Calculates the variance
        Eigen::Vector3d omega_variance = Eigen::Vector3d::Zero();
        Eigen::Vector3d acc_variance = Eigen::Vector3d::Zero();
        for (int i = 0; i < 10; i++) {
            Eigen::Vector3d omega_difference = omega_buffer_[i] - average_omega;
            omega_variance.x() += omega_difference.x() * omega_difference.x();
            omega_variance.y() += omega_difference.y() * omega_difference.y();
            omega_variance.z() += omega_difference.z() * omega_difference.z();

            Eigen::Vector3d acc_difference = acc_buffer_[i] - average_acc;
            acc_variance.x() += acc_difference.x() * acc_difference.x();
            acc_variance.y() += acc_difference.y() * acc_difference.y();
            acc_variance.z() += acc_difference.z() * acc_difference.z();
        }
        omega_variance /= 9.0f;
        acc_variance /= 9.0f;

        //Uses the max variance to prevent under estimation
        double max_omega_variance = std::max({omega_variance.x(), omega_variance.y(), omega_variance.z()});
        double max_acc_variance = std::max({acc_variance.x(), acc_variance.y(), acc_variance.z()});

        q_.block<3,3>(0,0) = Eigen::Matrix3d::Identity() * max_acc_variance;
        q_.block<3,3>(3,3) = Eigen::Matrix3d::Identity() * max_omega_variance;
        q_.block<3,3>(6,6) = Eigen::Matrix3d::Identity() * max_omega_variance;
    }

    Eigen::Matrix3d skew(const Eigen::Vector3d& v)
    {
        Eigen::Matrix3d skewed_matrix;
        skewed_matrix << 0, -v.z(), v.y(),
             v.z(), 0, -v.x(),
             -v.y(), v.x(), 0;
        return skewed_matrix;
    }

private:
    //Estimated IMU noise
    const double acc_var_x = 0.001788;
    const double acc_var_y = 0.001610;
    const double acc_var_z = 0.001994;
    const double gyro_var_x = 0.000002603;
    const double gyro_var_y = 0.000002754;
    const double gyro_var_z = 0.000002742;

    //Defines klamann stuff
    Eigen::VectorXd x_;
    Eigen::MatrixXd p_;
    Eigen::MatrixXd q_;
    Eigen::MatrixXd r_;

    //Defines buffers
    std::array<Eigen::Vector3d, 10> omega_buffer_ = {Eigen::Vector3d::Zero()};      //If at 100Hz, 50 samples corresponds to 0.5 seconds of data.
    std::array<Eigen::Vector3d, 10> acc_buffer_ = {Eigen::Vector3d::Zero()};
    std::vector<IMU_geometry> IMUs;
};

#endif // VIMU_FILTER_HPP