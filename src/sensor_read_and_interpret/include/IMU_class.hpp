#ifndef IMU_CLASS_HPP
#define IMU_CLASS_HPP

#include <structs.hpp>
#include <chrono>
#include <array>
#include <algorithm>
#include <Eigen/Dense>
#include "filter_coeffs_lowpass.hpp"
#include "klamann_shit.hpp"
#include <iostream>

class IMU
{
public:
    IMU(const Eigen::Vector3d position_on_robot, const Eigen::Quaterniond initial_orientation)
        : position_(-position_on_robot.x(), position_on_robot.y(), position_on_robot.z()), orientation_(1.0, 0.0, 0.0, 0.0)
    {
        //Sets up the IMU
        orientation_.normalize();
        imu_to_robot_frame_ = initial_orientation.conjugate();
        imu_to_robot_frame_.normalize();
        last_update_time_ = std::chrono::steady_clock::now();
    }

    void calibrate_gravity(const float acc_x, const float acc_y, const float acc_z)
    {
        //Calibrates the gravitational vector by averaging over several samples
        gravitational_vector_.x() += acc_x/calibration_needed_;
        gravitational_vector_.y() += acc_y/calibration_needed_;
        gravitational_vector_.z() += acc_z/calibration_needed_;
        calibration_count_++;
    }

    void calibrate_gyro(const float gyro_x, const float gyro_y, const float gyro_z)
    {
        //Calibrates the gyroscope bias by averaging over several samples
        gyro_bias_.x() += gyro_x/calibration_needed_;
        gyro_bias_.y() += gyro_y/calibration_needed_;
        gyro_bias_.z() += gyro_z/calibration_needed_;
    }

    void update2_electric_boogalo(const float acc_x, const float acc_y, const float acc_z,
                             const float gyro_x, const float gyro_y, const float gyro_z,
                             const bool fuse_sensors)
    {
        float dt = std::chrono::duration<float>(std::chrono::steady_clock::now() - last_update_time_).count();
        last_update_time_ = std::chrono::steady_clock::now();

        if (calibration_count_ < calibration_needed_) {
            calibrate_gravity(acc_x, acc_y, acc_z);
            calibrate_gyro(gyro_x, gyro_y, gyro_z);
            return;
        }

        //Method based on https://lavalle.pl/vr/node279.html

        Eigen::Vector3d gyro_sensor(gyro_x - gyro_bias_.x(),
                                    gyro_y - gyro_bias_.y(),
                                    gyro_z - gyro_bias_.z());

        //Finds rotational angle
        double angle = gyro_sensor.norm() * dt;

        //Finds rotational axis, creates quaternion and updates
        Eigen::Vector3d axis = gyro_sensor.normalized();
        Eigen::Quaterniond delta_quaternion;
        delta_quaternion = Eigen::AngleAxisd(angle, axis);
        orientation_ = orientation_ * delta_quaternion;

        acc_ << acc_x, acc_y, acc_z;

        if (fuse_sensors)
        {
            orientation_ = fuse_acceleration_gyroscope_for_orientation(acc_, dt);
        }

        acc_ = orientation_ * acc_;

        // Fjern gravitasjon og bias i sensorramme
        Eigen::Vector3d acc_sensor = acc_ - gravitational_vector_;
                        
        acc_sensor = orientation_.conjugate() * acc_sensor;

        //Rotate acc to ROV frame
        acc_ = orientation_ * acc_sensor;
        rotated_gyro_ = orientation_ * gyro_sensor;
    }

    //Theese four functions are self documenting. Think instead on if you have remembered to eat today
    Eigen::Vector3d get_acceleration() const
    {
        return acc_;
    }

    Eigen::Quaterniond get_orientation() const
    {
        return orientation_;
    }

    Eigen::Quaterniond get_delta_orientation()
    {
        return delta_orientation_;
    }

    Eigen::Vector3d get_gyro()
    {
        return rotated_gyro_;
    }

private:
    Eigen::Quaterniond fuse_acceleration_gyroscope_for_orientation(const Eigen::Vector3d& acc, const float dt)
    {
        //Using a modified version of fusing method proposed in 
        //https://ciis.lcsr.jhu.edu/lib/exe/fetch.php?media=courses:456:2021:projects:456-2021-01:estimatingorientationcontreras.pdf
        float length_acc = acc.norm();
        float length_gravity = std::sqrt(
            gravitational_vector_.x() * gravitational_vector_.x() +
            gravitational_vector_.y() * gravitational_vector_.y() +
            gravitational_vector_.z() * gravitational_vector_.z()
        );

        //Return if acceleration is out of expected range
        if (length_acc < length_gravity*0.95 || length_acc > length_gravity*1.05)
        {
            return orientation_;
        }
        //Normalize vectors
        Eigen::Vector3d norm_acc(
            acc.x() / length_acc,
            acc.y() / length_acc,
            acc.z() / length_acc
        );

        Eigen::Vector3d norm_gravity(
            gravitational_vector_.x() / length_gravity,
            gravitational_vector_.y() / length_gravity,
            gravitational_vector_.z() / length_gravity
        );
        //Calculate error between expected gravity vector and measured gravity vector
        Eigen::Vector3d expected_gravity = orientation_.conjugate() *norm_gravity;
        Eigen::Vector3d error = norm_acc.cross(expected_gravity);

        //Get correction from PID controller
        Eigen::Vector3d correction = PID_corrector(error, length_acc, dt);
        //Sign on correction might be wrong...
        orientation_ = orientation_ * Eigen::Quaterniond(
            1.0f,
            correction.x() * dt * 0.5f,
            correction.y() * dt * 0.5f,
            0.0f
        );
        orientation_.normalize();

        return orientation_;
    }

    Eigen::Vector3d PID_corrector(Eigen::Vector3d error, float acc_length, float dt)
    {
        //PID controller constants
        constexpr float Kp = 0.7f;
        constexpr float Ki = 0.003f;
        constexpr float Kd = 0.0f;
        constexpr float weight = 0.2f;
        //Static variables to hold integral and previous error
        static Eigen::Vector3d integral_error = Eigen::Vector3d::Zero();
        static Eigen::Vector3d prev_error = Eigen::Vector3d::Zero();
        //Update integral error
        integral_error.x() += error.x() * dt * weight;
        integral_error.y() += error.y() * dt * weight;
        integral_error.z() += error.z() * dt * weight;
        
        //Can't correct z axis (yaw) with accelerometer
        Eigen::Vector3d gain(
            Kp * error.x() + Ki * integral_error.x() + Kd * (error.x() - prev_error.x()) / dt,
            Kp * error.y() + Ki * integral_error.y() + Kd * (error.y() - prev_error.y()) / dt,
            0.0f
        );
        prev_error = error;
        return gain;
    }

    Eigen::Vector3d compansate_acc_for_angular_velocity(Eigen::Vector3d acc, Eigen::Vector3d gyro, const float dt)
    {
        //Equation from https://robotics.stackexchange.com/questions/24276/calculation-of-imu-offset-for-placement-of-inertial-measurement-unit-away-from-c? with some changed signs to fit ROV coordinate system
        //Quick maths
        Eigen::Vector3d gyro_acc(
            -(gyro.x() - prev_gyro_.x()) / dt,
            -(gyro.y() - prev_gyro_.y()) / dt,
            (gyro.z() - prev_gyro_.z()) / dt
        );

        Eigen::Vector3d acc_comp = acc;
        //Cross product calculations
        acc_comp.x() += -(gyro.y()*gyro.y() + gyro.z()*gyro.z())*position_.x() + (gyro.x()*gyro.y() - gyro_acc.z())*position_.y() + (gyro.x()*gyro.z() + gyro_acc.y())*position_.z();
        acc_comp.y() += (gyro.x()*gyro.y() + gyro_acc.z())*position_.x() + -(gyro.x()*gyro.x() + gyro.z()*gyro.z())*position_.y() + (gyro.y()*gyro.z() - gyro_acc.x())*position_.z();
        acc_comp.z() += (gyro.x()*gyro.z() - gyro_acc.y())*position_.x() + (gyro.y()*gyro.z() + gyro_acc.x())*position_.y() + -(gyro.x()*gyro.x() + gyro.y()*gyro.y())*position_.z();

        prev_gyro_ = gyro;

        return acc_comp;
    }

private:
    //Variables and shit
    Eigen::Vector3d position_;
    Eigen::Vector3d acc_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation_ = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond delta_orientation_ = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond imu_to_robot_frame_;
    Eigen::Vector3d gravitational_vector_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_bias_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d prev_gyro_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d rotated_gyro_ = Eigen::Vector3d::Zero();
    unsigned int calibration_count_ = 0;
    unsigned int calibration_needed_ = 500;
    std::chrono::steady_clock::time_point last_update_time_;
    //Initializes filter window. Filter not currently in use, probably wont be either
    std::array<float, FILTER_LENGTH> acc_x_buffer_ = {0.0f};
    std::array<float, FILTER_LENGTH> acc_y_buffer_ = {0.0f};
    std::array<float, FILTER_LENGTH> acc_z_buffer_ = {0.0f};
};

#endif // IMU_CLASS_HPP