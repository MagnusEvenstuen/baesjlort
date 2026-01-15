#include <structs.hpp>
#include <cmath>
#include <chrono>
#include <array>
#include <algorithm>
#include "filter_coeffs_lowpass.hpp"

class IMU
{
public:
    IMU(const Vector3 position_on_robot, const Quaternion initial_orientation)
        : position_(position_on_robot), orientation_(initial_orientation)
    {
        orientation_.normalize();
        last_update_time_ = std::chrono::steady_clock::now();
    }

    void calibrate_gravity(const float acc_x, const float acc_y, const float acc_z)
    {
        gravitational_vector_.x += acc_x/calibration_needed_;
        gravitational_vector_.y += acc_y/calibration_needed_;
        gravitational_vector_.z += acc_z/calibration_needed_;
        calibration_count_++;
    }

    void calibrate_gyro(const float gyro_x, const float gyro_y, const float gyro_z)
    {
        gyro_bias_.x += gyro_x/calibration_needed_;
        gyro_bias_.y += gyro_y/calibration_needed_;
        gyro_bias_.z += gyro_z/calibration_needed_;
    }

    void update(const float acc_x, const float acc_y, const float acc_z,
                const float gyro_x, const float gyro_y, const float gyro_z)
    {
        float dt = std::chrono::duration<float>(std::chrono::steady_clock::now() - last_update_time_).count();
        last_update_time_ = std::chrono::steady_clock::now();
        if (calibration_count_ < calibration_needed_)
        {
            calibrate_gravity(acc_x, acc_y, acc_z);
            calibrate_gyro(gyro_x, gyro_y, gyro_z);
            return;
        }


        Quaternion delta_orientation(
            1.0f,
            (gyro_x - gyro_bias_.x) * dt * 0.5f,
            (gyro_y - gyro_bias_.y) * dt * 0.5f,
            (gyro_z - gyro_bias_.z) * dt * 0.5f
        );
        //delta_orientation_ = delta_orientation;

        orientation_ = orientation_ * delta_orientation;
        orientation_.normalize();

        std::rotate(acc_x_buffer_.begin(), acc_x_buffer_.begin() + 1, acc_x_buffer_.end());
        std::rotate(acc_y_buffer_.begin(), acc_y_buffer_.begin() + 1, acc_y_buffer_.end());
        std::rotate(acc_z_buffer_.begin(), acc_z_buffer_.begin() + 1, acc_z_buffer_.end());
        acc_x_buffer_.back() = acc_x;
        acc_y_buffer_.back() = acc_y;
        acc_z_buffer_.back() = acc_z;
        Vector3 filtered_acc = {0.0f, 0.0f, 0.0f};


        for (size_t i = 0; i < FILTER_LENGTH; i++)
        {
            filtered_acc.x += filter_coeffs[i] * acc_x_buffer_[i];
            filtered_acc.y += filter_coeffs[i] * acc_y_buffer_[i];
            filtered_acc.z += filter_coeffs[i] * acc_z_buffer_[i];
        }

        acc_ = filtered_acc;

        // Rotate acceleration to world frame and low pass filter
        orientation_ = fuse_acceleration_gyroscope_for_orientation({acc_x, acc_y, acc_z}, delta_orientation, dt);
        acc_ = compansate_acc_for_angular_velocity({acc_x, acc_y, acc_z}, {gyro_x, gyro_y, gyro_z}, dt);
        acc_ = orientation_.rotate_vector({acc_.x, acc_.y, acc_.z});

        // Subtract gravity in world frame
        acc_.x -= gravitational_vector_.x;
        acc_.y -= gravitational_vector_.y;
        acc_.z -= gravitational_vector_.z;

        prev_gyro_ = {gyro_x, gyro_y, gyro_z};
    }

    Vector3 get_acceleration() const
    {
        return acc_;
    }

    Quaternion get_orientation() const
    {
        return orientation_;
    }

    Quaternion get_delta_orientation()
    {
        return delta_orientation_;
    }

private:
    Quaternion fuse_acceleration_gyroscope_for_orientation(const Vector3& acc, const Quaternion& delta_orientation, float dt)
    {
        //Using a modified version of fusing method proposed in 
        //https://ciis.lcsr.jhu.edu/lib/exe/fetch.php?media=courses:456:2021:projects:456-2021-01:estimatingorientationcontreras.pdf
        float length_acc = std::sqrt(acc.x*acc.x + acc.y*acc.y + acc.z*acc.z);
        float length_gravity = std::sqrt(
            gravitational_vector_.x * gravitational_vector_.x +
            gravitational_vector_.y * gravitational_vector_.y +
            gravitational_vector_.z * gravitational_vector_.z
        );
        float length_delta_orientation = std::sqrt(
            delta_orientation.w * delta_orientation.w +
            delta_orientation.x * delta_orientation.x +
            delta_orientation.y * delta_orientation.y +
            delta_orientation.z * delta_orientation.z
        );

        //Return if acceleration is out of expected range
        if (length_acc < 9.6f || length_acc > 10.0f)
        {
            return orientation_;
        }
        
        Vector3 norm_acc = {
            acc.x / length_acc,
            acc.y / length_acc,
            acc.z / length_acc
        };

        Vector3 norm_gravity = {
            gravitational_vector_.x / length_gravity,
            gravitational_vector_.y / length_gravity,
            gravitational_vector_.z / length_gravity
        };
        Vector3 expected_gravity = orientation_.rotate_vector_inverse(norm_gravity);
        Vector3 error = {
            norm_acc.y * expected_gravity.z - norm_acc.z * expected_gravity.y,
            norm_acc.z * expected_gravity.x - norm_acc.x * expected_gravity.z,
            norm_acc.x * expected_gravity.y - norm_acc.y * expected_gravity.x
        };

        Vector3 correction = PID_corrector(error, length_acc, dt);
        orientation_ = orientation_ * Quaternion(
            1.0f,
            correction.x * dt * 0.5f,
            correction.y * dt * 0.5f,
            0.0f
        );
        orientation_.normalize();

        return orientation_;
    }

    Vector3 PID_corrector(Vector3 error, float acc_length, float dt)
    {
        constexpr float Kp = 1.2f;
        constexpr float Ki = 0.003f;
        constexpr float Kd = 0.001f;

        static Vector3 integral_error = {0.0f, 0.0f, 0.0f};
        static Vector3 prev_error = {0.0f, 0.0f, 0.0f};
        constexpr float weight = 0.3f;

        integral_error.x += error.x * dt * weight;
        integral_error.y += error.y * dt * weight;
        integral_error.z += error.z * dt * weight;
        
        //Can't correct z axis (yaw) with accelerometer
        Vector3 gain = {
            Kp * error.x + Ki * integral_error.x + Kd * (error.x - prev_error.x) / dt,
            Kp * error.y + Ki * integral_error.y + Kd * (error.y - prev_error.y) / dt,
            0.0f
        };
        prev_error = error;
        return gain;
    }

    Vector3 compansate_acc_for_angular_velocity(Vector3 acc, Vector3 gyro, const float dt)
    {
        //Equation from https://robotics.stackexchange.com/questions/24276/calculation-of-imu-offset-for-placement-of-inertial-measurement-unit-away-from-c? with some changed signs to fit ROV coordinate system
        Vector3 gyro_acc = {
            (gyro.x - prev_gyro_.x) / dt,
            (gyro.y - prev_gyro_.y) / dt,
            (gyro.z - prev_gyro_.z) / dt
        };

        Vector3 acc_comp = acc;
        acc_comp.x -= -(gyro.y*gyro.y + gyro.z*gyro.z)*position_.x + (gyro.x*gyro.y - gyro_acc.z)*position_.y + (gyro.x*gyro.z + gyro_acc.y)*position_.z;
        acc_comp.y -= (gyro.x*gyro.y + gyro_acc.z)*position_.x + -(gyro.x*gyro.x + gyro.z*gyro.z)*position_.y + (gyro.y*gyro.z - gyro_acc.x)*position_.z;
        acc_comp.z -= (gyro.x*gyro.z - gyro_acc.y)*position_.x + (gyro.y*gyro.z + gyro_acc.x)*position_.y + -(gyro.x*gyro.x + gyro.y*gyro.y)*position_.z;

        return acc_comp;
    }

private:
    Vector3 position_;
    Vector3 acc_;
    Quaternion orientation_;
    Quaternion delta_orientation_;
    Vector3 gravitational_vector_ = {0.0f, 0.0f, 0.0f};
    Vector3 gyro_bias_ = {0.0f, 0.0f, 0.0f};
    Vector3 prev_gyro_ = {0.0f, 0.0f, 0.0f};
    unsigned int calibration_count_ = 0;
    unsigned int calibration_needed_ = 500;
    std::chrono::steady_clock::time_point last_update_time_;
    std::array<float, FILTER_LENGTH> acc_x_buffer_ = {0.0f};
    std::array<float, FILTER_LENGTH> acc_y_buffer_ = {0.0f};
    std::array<float, FILTER_LENGTH> acc_z_buffer_ = {0.0f};
};