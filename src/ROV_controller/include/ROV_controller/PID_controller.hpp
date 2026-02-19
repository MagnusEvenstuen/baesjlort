#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <Eigen/Dense>

//This PID controller class will be used to regulate one axis of the ROV, for instance the x-axis. Multiple of theese PID regulators will be used to control all the axis
class PID_controller
{
public:
    PID_controller(const double kp, const double ki, const double kd): kp_(kp), ki_(ki), kd_(kd)
    {

    }

    void set_target_position(const double target_position)
    {
        target_position_ = target_position;
    }

    void set_target_quaternion(const Eigen::Quaterniond target_quaternion)
    {
        target_quaternion_ = target_quaternion;
        target_quaternion_.normalize();
    }

    double update(const double position, const double dt)
    {
        double error = target_position_ - position;
        integral_ += error * dt;
        double derivative = (error - prev_error_)/dt;
        prev_error_ = error;

        return kp_* error + ki_ * integral_ + kd_ * derivative;
    }

    double get_error_float()
    {
        return prev_error_;
    }

    //Overloads the update function to work with quaternions as well.
    Eigen::Vector3d update(const Eigen::Quaterniond& current, const float dt)
    {
        Eigen::Quaterniond quaternion_error = current.inverse() * target_quaternion_;
        quaternion_error.normalize();

        Eigen::Vector3d error;
        
        //Checks if best to rotate left or right
        if (quaternion_error.w() < 0) {
            error = Eigen::Vector3d(-quaternion_error.x(), -quaternion_error.y(), -quaternion_error.z());
        } else {
            error = Eigen::Vector3d(quaternion_error.x(), quaternion_error.y(), quaternion_error.z());
        }
        
        error *= 2.0f;
        integral_vector_ += error * dt;
        
        Eigen::Vector3d derivative = (error - prev_error_vector_) / dt;
        prev_error_vector_ = error;
        
        return kp_ * error + ki_ * integral_vector_ + kd_ * derivative;
    }

    Eigen::Vector3d get_error_quat()
    {
        return prev_error_vector_;
    }

private:
    double target_position_;
    Eigen::Quaterniond target_quaternion_ = Eigen::Quaterniond::Identity();
    double kp_;
    double ki_;
    double kd_;
    double integral_ = 0.0f;
    Eigen::Vector3d integral_vector_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    Eigen::Vector3d prev_error_vector_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    double prev_error_ = 0.0f;
};

#endif // PID_CONTROLLER_HPP