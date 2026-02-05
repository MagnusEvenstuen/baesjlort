#ifndef SENSOR_HANDLER_HPP
#define SENSOR_HANDLER_HPP

#include "structs.hpp"
#include "klamann_shit.hpp"
#include <chrono>
#include <fstream>
#include <iomanip>
#include <cmath>

class sensor_handler
{
public:
    sensor_handler(): orientation_({1.0f, 0.0f, 0.0f, 0.0f}), kalmann(
                        //A, B, and C matrices found using sysID in python
                      // A-matrix (9x9)
                      (Eigen::MatrixXd(9, 9) << 
                        1.01474855e+00, -3.64004525e-03,  6.28674112e-03, -4.47525821e-03,  2.05615813e-03, -6.08400975e-03, -5.11822564e-02, -5.66985020e-02,  3.06323981e-02,
                       -8.23657083e-03,  1.01450252e+00,  2.17234618e-03,  7.74917768e-03, -4.44659960e-04, -9.30973716e-04,  7.47567143e-02, -2.23738564e-02,  1.59409683e-02,
                       -7.08858487e-04, -3.46455935e-03,  1.00273585e+00,  3.63134556e-03, -3.34290771e-03,  4.82507888e-05, -8.84471126e-03, -1.20440629e-02, -3.50992498e-02,
                       -6.71560612e-03, -5.67478798e-03, -7.15709703e-03,  9.74354415e-01,  2.11145598e-03,  1.78921063e-03, -5.42824152e-04,  2.28382584e-02, -3.30980702e-05,
                        2.51902544e-03,  3.47544406e-03,  6.12675686e-03,  3.11435782e-03,  9.69184449e-01, -3.30665253e-03,  2.62446794e-03, -1.00846802e-02, -1.00980802e-02,
                       -1.42501047e-03, -1.08763272e-03, -3.10551936e-03, -1.16572494e-03, -1.60035718e-03,  9.68092797e-01, -3.26301049e-03,  3.26529025e-03,  2.60026154e-04,
                        4.01557730e-02, -9.05964337e-02, -5.25690686e-02, -1.24906609e-02,  8.66566588e-03,  6.84152346e-03,  7.66674965e-01,  8.19147263e-02,  1.28386202e-02,
                        9.15217487e-02,  1.18477997e-01,  2.63131963e-01,  5.77194483e-02, -3.46663809e-02, -2.94484941e-02,  4.39050595e-02,  4.68562201e-01, -1.76713498e-01,
                        1.79344254e-02,  3.19068720e-02,  1.76267726e-01,  3.97075066e-02, -3.23145393e-02, -1.35346696e-02,  2.75814912e-02, -2.08191625e-01,  7.34542522e-01
                      ).finished(),
                      
                      // B-matrix (9x8)
                      (Eigen::MatrixXd(9, 8) << 
                        2.98293906e-05, -2.44738972e-04,  2.62080423e-04, -4.43274020e-05,  2.08123026e-04,  1.71083814e-04,  2.39171495e-04,  2.09476457e-04,
                       -3.79616097e-05, -1.34540736e-04,  1.30317965e-04,  4.38752322e-05, -3.03286948e-04, -2.37521598e-04, -2.85424545e-04, -2.28960043e-04,
                       -9.35663557e-05,  6.31172996e-06, -4.12366911e-05,  1.28541472e-04,  3.33876579e-05,  5.70632127e-05,  3.90942398e-05,  6.45101324e-05,
                        6.12110444e-05,  1.55923994e-04, -1.15566422e-04, -1.03301160e-04,  1.00096413e-04, -1.26340963e-04,  9.83673125e-05, -1.28714275e-04,
                        1.26200224e-04, -2.61140656e-04, -1.69107649e-04,  3.02796112e-04, -2.98482851e-05, -1.01636115e-05,  1.66461765e-05,  3.64988769e-05,
                        1.20363850e-05,  1.12426698e-05, -1.20758038e-05, -1.13948587e-05, -1.73372733e-04, -1.83042059e-04,  2.07029190e-04,  1.98107573e-04,
                        5.25743499e-04,  7.47356137e-04, -5.98482796e-04, -6.84355817e-04,  1.08385111e-03,  1.02128086e-03,  9.98008432e-04,  9.71457986e-04,
                       -2.31407942e-03, -3.27695959e-03,  2.70257792e-03,  2.93187799e-03,  7.39796171e-05,  3.79594218e-04, -4.59665470e-05,  2.65707417e-04,
                       -1.88844810e-03, -1.18471959e-03,  8.24168880e-04,  2.26814080e-03,  6.53727575e-05,  2.13181521e-04,  4.77802841e-06,  1.56527984e-04
                      ).finished(),
                      
                      // C-matrix (6x9)
                      (Eigen::MatrixXd(6, 9) << 
                        -1.66602819, -1.73746887,  1.14323272,  0.31891409, -0.43971104, -0.07133168,  0.17240743, -2.10248406,  4.36921106,
                        -0.51901373, -0.46379425, -0.75549216,  0.17637054, -0.14906085,  0.05421895,  0.68677665, -2.7314619,  -1.09893456,
                         1.89365573, -1.40039379,  0.0308432,   0.09731294,  0.01367617, -0.24559277,  4.88776415,  0.8780713,   0.2848233,
                        -0.45886987,  0.05328577, -0.62696199,  0.06452097, -0.41871057, -8.16701337,  0.06690262,  0.10126616, -0.07358426,
                        -0.57616533, -0.42729205,  1.0209955,  -8.79008857, -0.10850449, -0.10594111,  0.09963103,  0.02876716,  0.04196708,
                        -0.62032075, -0.68633797,  0.33857208,  0.09106045,  8.53158774, -0.4015052,   0.0531382,  -0.02041128, -0.12874247
                      ).finished()
                  )
    {
        //Opens CSV file for logging
        last_update_time_ = std::chrono::steady_clock::now();
        start_time_ = last_update_time_;
        csv_file_.open("/home/gud/Skole/baesjlort/scripts/plotting_program/data_files/sensor_data.csv", std::ios::app);
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
        //Closse CSV file if open
        if (csv_file_.is_open())
        {
            csv_file_.close();
        }
    }

    void update_pose_estimate(const float pos_x, const float pos_y, const float pos_z,
                           const float quat_x, const float quat_y, const float quat_z, 
                           const float quat_w, const float dt)
    {
        if (first_update_)
        {
            //On first update, set position directly to SLAM position to avoid large jumps
            current_position_.x = pos_x;
            current_position_.y = pos_y;
            current_position_.z = pos_z;
            first_update_ = false;
            prev_SLAM_pos_ = {pos_x, pos_y, pos_z};
            prev_SLAM_orientation_ = {quat_w, quat_x, quat_y, quat_z};
            return;
        }
        //Update position and orientation from SLAM pose estimate
        Vector3 estimated_SLAM_speed = {
            (pos_x - prev_SLAM_pos_.x) / dt,
            (pos_z - prev_SLAM_pos_.z) / dt,
            (pos_y - prev_SLAM_pos_.y) / dt
        };

        //Rotates the estimated speed to the correct frame. Rotations work, but is a mess
        estimated_SLAM_speed = orientation_.rotate_vector(estimated_SLAM_speed);

        Quaternion estimated_SLAM_delta_orientation = {
            (quat_w - prev_SLAM_orientation_.w) / dt,
            -(quat_x - prev_SLAM_orientation_.x) / dt,
            -(quat_z - prev_SLAM_orientation_.z) / dt,
            (quat_y - prev_SLAM_orientation_.y) / dt
        };

        float alpha = 0.7f;

        if (dt > 0.3f)
        {
            alpha = 1.0f;
        } else if (dt < 0.1f)
        {
            alpha = 0.3f;
        }

        //Complementary filter for speed. Higher weight on current speed due to IMU having higher update rate
        //SLAM speed is noisy due to low frame rate
        current_speed_.x = alpha*current_speed_.x + (1-alpha)*estimated_SLAM_speed.x;
        current_speed_.y = alpha*current_speed_.y + (1-alpha)*estimated_SLAM_speed.y;
        current_speed_.z = alpha*current_speed_.z + (1-alpha)*estimated_SLAM_speed.z;

        //Update prev values for speed calculation
        prev_SLAM_pos_ = {pos_x, pos_y, pos_z};
        prev_SLAM_orientation_ = {quat_w, quat_x, quat_y, quat_z};
    }

    void update_truth_odometry(float pos_x, float pos_y, float pos_z,
                           float vel_x, float vel_y, float vel_z,
                           float quat_x, float quat_y, float quat_z, float quat_w)
    {
        //Update perfect position, speed, acceleration and orientation from truth odometry sensor
        Vector3 prev_speed = perfect_speed_;
        auto current_time = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(current_time - last_truth_update_time_).count();
        last_truth_update_time_ = current_time;
        perfect_position_.x = pos_x;
        perfect_position_.y = pos_y;
        perfect_position_.z = pos_z;
        perfect_speed_.x = vel_x;
        perfect_speed_.y = vel_y;
        perfect_speed_.z = vel_z;
        perfect_acceleration_.x = (perfect_speed_.x - prev_speed.x) / dt;
        perfect_acceleration_.y = (perfect_speed_.y - prev_speed.y) / dt;
        perfect_acceleration_.z = (perfect_speed_.z - prev_speed.z) / dt;
        
        perfect_orientation_.w = quat_w;
        perfect_orientation_.x = quat_x;
        perfect_orientation_.y = quat_y;
        perfect_orientation_.z = quat_z;
        perfect_orientation_ = world_correction_ * perfect_orientation_;
        perfect_orientation_.normalize();
        perfect_speed_ = perfect_orientation_.rotate_vector_inverse(perfect_speed_);
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

    void update(const Vector3& acc, const Vector3& gyro, const std::array<float, 8>& thruster_gain)
    {
        //Updates delta time
        auto current_time = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(current_time - last_update_time_).count();
        last_update_time_ = current_time;
        
        //Transfers vectors to Eigen for Kalman filter
        Eigen::VectorXd measurement(6);
        measurement << acc.x, acc.y, acc.z, gyro.x, gyro.y, gyro.z;
        Eigen::VectorXd gain(8); 
        gain << thruster_gain[0], thruster_gain[1], thruster_gain[2], thruster_gain[3], thruster_gain[4], thruster_gain[5], thruster_gain[6], thruster_gain[7];
        Eigen::VectorXd filtered_values = kalmann.update(gain, measurement);
        //Gets filtered measurements from Kalman filter
        Vector3 acc_filtered = {
            static_cast<float>(filtered_values[0]),
            static_cast<float>(filtered_values[1]),
            static_cast<float>(filtered_values[2])
        };
        
        Vector3 gyro_filtered = {
            static_cast<float>(filtered_values[3]),
            static_cast<float>(filtered_values[4]),
            static_cast<float>(filtered_values[5])
        };

        //Update orientation based on filtered gyroscope data
        delta_orientation = Quaternion{
            1.0f,
            gyro_filtered.x * dt * 0.5f,
            gyro_filtered.y * dt * 0.5f,
            gyro_filtered.z * dt * 0.5f
        };
        orientation_ = orientation_ * delta_orientation;
        orientation_.normalize();

        acc_filtered = orientation_.rotate_vector(acc_filtered);

        //Update position and speed based on filtered accelerometer data
        current_position_.x -= current_speed_.x * dt + 0.5f * acc_filtered.x * dt * dt;
        current_position_.y -= current_speed_.y * dt + 0.5f * acc_filtered.y * dt * dt;
        current_position_.z -= current_speed_.z * dt + 0.5f * acc_filtered.z * dt * dt;
        //Update depth with pressure sensor data using complementary filter
        current_position_.z = 0.1*current_position_.z + 0.9*depth_;
        current_speed_.x += acc_filtered.x * dt;
        current_speed_.y += acc_filtered.y * dt;
        current_speed_.z += acc_filtered.z * dt;

        log_to_csv(acc_filtered.x, acc_filtered.y, acc_filtered.z);
    }

    Vector3 get_position() const
    {
        return current_position_;
    }

    Quaternion get_orientation() const
    {
        return orientation_;
    }

    void log_to_csv(float acc_x, float acc_y, float acc_z)
    {
        if (csv_file_.is_open())
        {
            auto current_time = std::chrono::steady_clock::now();
            float elapsed = std::chrono::duration<float>(current_time - start_time_).count();
            //Logs data to CSV file for plotting
            csv_file_ << std::fixed << std::setprecision(6)
                     << elapsed << ","
                     << acc_y << "," << -acc_x << "," << acc_z << ","
                     << current_speed_.x << "," << -current_speed_.y << "," << current_speed_.z << ","
                     << current_position_.x << "," << -current_position_.y << "," << -current_position_.z << ","
                     << perfect_acceleration_.x << "," << perfect_acceleration_.y << "," << perfect_acceleration_.z << ","
                     << perfect_speed_.x << "," << perfect_speed_.y << "," << perfect_speed_.z << ","
                     << perfect_position_.x << ", " << perfect_position_.y << ", " << -perfect_position_.z << ", " << depth_ << ","
                     << orientation_.x << ", " << orientation_.y << ", " << orientation_.z << ", "
                     << perfect_orientation_.x << ", " << perfect_orientation_.y << ", " << perfect_orientation_.z
                     << "\n";
            csv_file_.flush();
        }
    }

private:
    klamann_shit kalmann;
    Vector3 current_position_ = {0.0f, 0.0f, 0.0f};
    Vector3 current_speed_ = {0.0f, 0.0f, 0.0f};
    Vector3 perfect_acceleration_ = {0.0f, 0.0f, 0.0f};
    Vector3 perfect_speed_ = {0.0f, 0.0f, 0.0f};
    Vector3 perfect_position_ = {0.0f, 0.0f, 0.0f};
    Vector3 prev_SLAM_pos_ = {0.0f, 0.0f, 0.0f};
    Quaternion prev_SLAM_orientation_ = {1.0f, 0.0f, 0.0f, 0.0f};
    Quaternion orientation_ = {1.0f, 0.0f, 0.0f, 0.0f};
    Quaternion perfect_orientation_;
    Quaternion world_correction_ = Quaternion(0.0f, 1.0f, 0.0f, 0.0f);
    Quaternion delta_orientation = {1.0f, 0.0f, 0.0f, 0.0f};
    std::chrono::steady_clock::time_point last_update_time_;
    std::chrono::steady_clock::time_point last_truth_update_time_;
    std::chrono::steady_clock::time_point start_time_;
    std::ofstream csv_file_;
    float depth_ = 0.0f;
    float surface_pressure_ = 101325.0f;   //Pa
    const float water_density_ = 997.0f;       //kg/m^3
    const float gravity_ = 9.81f;              //m/s^2
    bool set_surface_pressure_ = false;
    bool first_update_ = true;
};

#endif // SENSOR_HANDLER_HPP