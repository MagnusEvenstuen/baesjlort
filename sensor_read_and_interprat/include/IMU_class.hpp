#include <structs.hpp>
#include <cmath>
#include <eigen3/Eigen/Dense>

class IMU
{
public:
    IMU(const Vector3 position_on_robot, const Quaternion initial_orientation)
        : position_(position_on_robot), orientation_(initial_orientation)
    {
        orientation_.normalize();
    }

    void calibrate_gravity(const float acc_x, const float acc_y, const float acc_z, 
                           const unsigned int count = 100)
    {
        gravitational_vector_.x += acc_x/count;
        gravitational_vector_.y += acc_y/count;
        gravitational_vector_.z += acc_z/count;
    }

    void update(const float acc_x, const float acc_y, const float acc_z,
                const float gyro_x, const float gyro_y, const float gyro_z,
                const float dt)
    {

        Quaternion delta_orientation(
            1.0f,
            gyro_x * dt * 0.5f,
            gyro_y * dt * 0.5f,
            gyro_z * dt * 0.5f
        );

        delta_orientation.normalize();
        orientation_ = orientation_ * delta_orientation;
        orientation_.normalize();

        // Rotate acceleration to world frame
        acc = orientation_.rotate_vector({acc_x, acc_y, acc_z});
        acc = compansate_acc_for_angular_velocity(acc, {gyro_x, gyro_y, gyro_z}, dt);
        
        // Subtract gravity in world frame
        acc.x -= gravitational_vector_.x;
        acc.y -= gravitational_vector_.y;
        acc.z -= gravitational_vector_.z;
        prev_gyro_ = {gyro_x, gyro_y, gyro_z};
    }

    Vector3 compansate_acc_for_angular_velocity(Vector3 acc, Vector3 gyro, const float dt)
    {
        //Equation from https://robotics.stackexchange.com/questions/24276/calculation-of-imu-offset-for-placement-of-inertial-measurement-unit-away-from-c?
        Vector3 gyro_acc = {
            (gyro.x - prev_gyro_.x) / dt,
            (gyro.y - prev_gyro_.y) / dt,
            (gyro.z - prev_gyro_.z) / dt
        };

        Vector3 acc_comp = acc;
        acc_comp.x += -(gyro.y*gyro.y + gyro.z*gyro.z)*position_.x + (gyro.x*gyro.y - gyro_acc.z)*position_.y + (gyro.x*gyro.z + gyro_acc.y)*position_.z;
        acc_comp.y += (gyro.x*gyro.y + gyro_acc.z)*position_.x + -(gyro.x*gyro.x + gyro.z*gyro.z)*position_.y + (gyro.y*gyro.z - gyro_acc.x)*position_.z;
        acc_comp.z += (gyro.x*gyro.z - gyro_acc.y)*position_.x + (gyro.y*gyro.z + gyro_acc.x)*position_.y + -(gyro.x*gyro.x + gyro.y*gyro.y)*position_.z;

        return acc_comp;
    }

    Vector3 get_acceleration() const
    {
        return acc;
    }

    Quaternion get_orientation() const
    {
        return orientation_;
    }

private:
    Vector3 position_;
    Vector3 acc;
    Quaternion orientation_;
    Vector3 gravitational_vector_ = {0.0f, 0.0f, 0.0f};
    Vector3 prev_gyro_ = {0.0f, 0.0f, 0.0f};
};