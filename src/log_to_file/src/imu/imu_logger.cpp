#include "log_to_file/imu/imu_logger.hpp"
#include <ctime>
#include <chrono>
#include <filesystem>

ImuLogger::ImuLogger(std::filesystem::path path)
	: rclcpp::Node("imu_logger")
{
    if (path.empty())
    {
        path = "imu.csv";
    }

    RCLCPP_INFO(get_logger(), "Logging to %s", std::filesystem::absolute(path).c_str());

	if (std::filesystem::exists(path))
	{
		log_file_.open(path, std::ios::app);
	}
	else
	{
		log_file_.open(path);
		// Make csv headers
		log_file_ << "timestamp,frame_id,"
            // Orientation
            << "ori_x,ori_y,ori_z,ori_w,"
            << "ori_cov_xx,ori_cov_xy,ori_cov_xz,"
            << "ori_cov_yx,ori_cov_yy,ori_cov_yz,"
            << "ori_cov_zx,ori_cov_zy,ori_cov_zz,"
            // Angular Velocity
            << "ang_vel_x,ang_vel_y,ang_vel_z,"
            << "ang_vel_cov_xx,ang_vel_cov_xy,ang_vel_cov_xz,"
            << "ang_vel_cov_yx,ang_vel_cov_yy,ang_vel_cov_yz,"
            << "ang_vel_cov_zx,ang_vel_cov_zy,ang_vel_cov_zz,"
            // Linear Acceleration
            << "lin_acc_x,lin_acc_y,lin_acc_z,"
            << "lin_acc_cov_xx,lin_acc_cov_xy,lin_acc_cov_xz,"
            << "lin_acc_cov_yx,lin_acc_cov_yy,lin_acc_cov_yz,"
            << "lin_acc_cov_zx,lin_acc_cov_zy,lin_acc_cov_zz,"
            << std::endl;
	}

	// TODO: Create subscribers
    imu_subscriber_ = create_subscription<Imu>("/gbr/imu_center", 10,
            std::bind(&ImuLogger::imuSubscriptionCallback, this, std::placeholders::_1));
}

void ImuLogger::imuSubscriptionCallback(Imu::UniquePtr msg)
{
    // Concatenate seconds and nanoseconds
    log_file_ << msg->header.stamp.sec << msg->header.stamp.nanosec
        << ',' << msg->header.frame_id
        << ',' << msg->orientation.x
        << ',' << msg->orientation.y
        << ',' << msg->orientation.z
        << ',' << msg->orientation.w;
    for (const auto &ori_cov : msg->orientation_covariance)
    {
        log_file_ << ',' << ori_cov;
    }

    log_file_ << ',' << msg->angular_velocity.x
        << ',' << msg->angular_velocity.y
        << ',' << msg->angular_velocity.z;
    for (const auto &ang_vel_cov : msg->angular_velocity_covariance)
    {
        log_file_ << ',' << ang_vel_cov;
    }

    log_file_ << ',' << msg->linear_acceleration.x
        << ',' << msg->linear_acceleration.y
        << ',' << msg->linear_acceleration.z;
    for (const auto &lin_acc_cov : msg->linear_acceleration_covariance)
    {
        log_file_ << ',' << lin_acc_cov;
    }

    log_file_ << std::endl;
}
