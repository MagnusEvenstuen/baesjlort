#ifndef SENSOR_HANDLER_HPP
#define SENSOR_HANDLER_HPP

#include "structs.hpp"
#include <chrono>
#include <fstream>
#include <iomanip>
#include <cmath>

class sensor_handler
{
public:
    sensor_handler()
    {
        last_update_time_ = std::chrono::steady_clock::now();
        start_time_ = last_update_time_;
        csv_file_.open("/home/gud/Skole/baesjlort/plotting_program/data_files/sensor_data.csv", std::ios::app);
        if (csv_file_.is_open())
        {
            csv_file_ << "timestamp,acc_x,acc_y,acc_z,vel_x,vel_y,vel_z,pos_x,pos_y,pos_z,"
                        "correct_acc_x,correct_acc_y,correct_acc_z, correct_vel_x,correct_vel_y,"
                        "correct_vel_z, correct_pos_x,correct_pos_y,correct_pos_z, depth_by_pressure,"
                        "orientation_x, orientation_y, orientation_z, truth_orientation_x, truth_orientation_y, truth_orientation_z\n";
        }
    }
    ~sensor_handler()
    {
        if (csv_file_.is_open())
        {
            csv_file_.close();
        }
    }
    
    //Assume gravity calibration is done while stationary
    Vector3 calibrate_gravity(const float acc_x, const float acc_y, const float acc_z, 
                            const unsigned int count = 100)
    {
        gravitational_vector_.x += acc_x/count;
        gravitational_vector_.y += acc_y/count;
        gravitational_vector_.z += acc_z/count;
        last_update_time_ = std::chrono::steady_clock::now();
        last_truth_update_time_ = std::chrono::steady_clock::now();
        orientation_ = perfect_orientation_;
        return gravitational_vector_;
    }

    void update_truth_odometry(float pos_x, float pos_y, float pos_z,
                           float vel_x, float vel_y, float vel_z,
                           float quat_x, float quat_y, float quat_z, float quat_w)
    {
        Vector3 prev_speed = perfect_speed_;
        auto current_time = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(current_time - last_truth_update_time_).count();
        last_truth_update_time_ = current_time;
        perfect_position_.x = pos_x;
        perfect_position_.y = pos_y;
        perfect_position_.z = -pos_z;
        perfect_speed_.x = vel_x;
        perfect_speed_.y = vel_y;
        perfect_speed_.z = -vel_z;
        perfect_acceleration_.x = (perfect_speed_.x - prev_speed.x) / dt;
        perfect_acceleration_.y = (perfect_speed_.y - prev_speed.y) / dt;
        perfect_acceleration_.z = (perfect_speed_.z - prev_speed.z) / dt;
        
        perfect_orientation_.w = quat_w;
        perfect_orientation_.x = quat_x;
        perfect_orientation_.y = quat_y;
        perfect_orientation_.z = quat_z;
    }


    void update_depth(float current_pressure)
    {
        //Presumes starting at surface level
        if (!set_surface_pressure_)
        {
            surface_pressure_ = current_pressure;
            set_surface_pressure_ = true;
        }
        depth_ = -(current_pressure - surface_pressure_) / (water_density_ * gravity_);
    }

    void update(float acc_x, float acc_y, float acc_z, 
                const float gyro_x, const float gyro_y, const float gyro_z)
    {
        auto current_time = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(current_time - last_update_time_).count();
        last_update_time_ = current_time;

        // Update orientation from gyro
        float angle = std::sqrt(gyro_x*gyro_x + gyro_y*gyro_y + gyro_z*gyro_z) * dt;
        if (angle > 0.0001f) {
            float scale = std::sin(angle / 2.0f) / (angle / dt);
            orientation_ = orientation_ * Quaternion(std::cos(angle / 2.0f), gyro_x*scale, gyro_y*scale, gyro_z*scale);
            orientation_.normalize();
        }

        // Rotate acceleration to world frame
        Vector3 acc = orientation_.rotate_vector({acc_x, acc_y, acc_z});
        
        // Subtract gravity in world frame
        acc.x -= gravitational_vector_.x;
        acc.y -= gravitational_vector_.y;
        acc.z -= gravitational_vector_.z;

        //Update position and speed
        current_position_.x += current_speed_.x * dt + 0.5f * acc.x * dt * dt;
        current_position_.y += current_speed_.y * dt + 0.5f * acc.y * dt * dt;
        current_position_.z += current_speed_.z * dt + 0.5f * acc.z * dt * dt;
        current_speed_.x += acc.x * dt;
        current_speed_.y += acc.y * dt;
        current_speed_.z += acc.z * dt;

        log_to_csv(acc.x, acc.y, acc.z);
    }

    Vector3 get_position() const
    {
        return current_position_;
    }

    void log_to_csv(float acc_x, float acc_y, float acc_z)
    {
        if (csv_file_.is_open())
        {
            auto current_time = std::chrono::steady_clock::now();
            float elapsed = std::chrono::duration<float>(current_time - start_time_).count();
            
            csv_file_ << std::fixed << std::setprecision(6)
                     << elapsed << ","
                     << acc_x << "," << acc_y << "," << acc_z << ","
                     << current_speed_.x << "," << current_speed_.y << "," << current_speed_.z << ","
                     << current_position_.x << "," << current_position_.y << "," << current_position_.z << ","
                     << perfect_acceleration_.x << "," << perfect_acceleration_.y << "," << perfect_acceleration_.z << ","
                     << perfect_speed_.x << "," << perfect_speed_.y << "," << perfect_speed_.z << ","
                     << perfect_position_.x << ", " << perfect_position_.y << ", " << perfect_position_.z << ", " << depth_ << ","
                     << orientation_.x << ", " << orientation_.y << ", " << orientation_.z << ", "
                     << perfect_orientation_.x << ", " << perfect_orientation_.y << ", " << perfect_orientation_.z
                     << "\n";
            csv_file_.flush();
        }
    }

private:
    Vector3 current_position_ = {0.0f, 0.0f, 0.0f};
    Vector3 current_speed_ = {0.0f, 0.0f, 0.0f};
    Vector3 gravitational_vector_ = {0.0f, 0.0f, 0.0f};
    Vector3 perfect_acceleration_ = {0.0f, 0.0f, 0.0f};
    Vector3 perfect_speed_ = {0.0f, 0.0f, 0.0f};
    Vector3 perfect_position_ = {0.0f, 0.0f, 0.0f};
    Quaternion orientation_;
    Quaternion perfect_orientation_;
    std::chrono::steady_clock::time_point last_update_time_;
    std::chrono::steady_clock::time_point last_truth_update_time_;
    std::chrono::steady_clock::time_point start_time_;
    std::ofstream csv_file_;
    float depth_ = 0.0f;
    float surface_pressure_ = 101325.0f;   //Pa
    const float water_density_ = 997.0f;       //kg/m^3
    const float gravity_ = 9.81f;              //m/s^2
    bool set_surface_pressure_ = false;
};

#endif // SENSOR_HANDLER_HPP