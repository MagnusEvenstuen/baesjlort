#ifndef KLAMANN_SHIT_HPP
#define KLAMANN_SHIT_HPP

//For this class to work eigen needs to be installed by running sudo apt install ros-jazzy-eigen* in the terminal
//This kalman implementation is made specifically for the ROV system. Changes need to be made for other use cases.

#include <Eigen/Dense>

class klamann_shit
{
public:
    klamann_shit(Eigen::MatrixXd& a_matrix, Eigen::MatrixXd& b_matrix, Eigen::MatrixXd& c_matrix):
    a_matrix_(a_matrix), b_matrix_(b_matrix), c_matrix_(c_matrix), x_hat_(Eigen::VectorXd::Zero(9)),
        x_hat_pre_(Eigen::VectorXd::Zero(9)),
        p_(Eigen::MatrixXd::Identity(9, 9) * 10),
        p_pre_(Eigen::MatrixXd::Identity(9, 9)),
        q_(Eigen::MatrixXd::Identity(9, 9) * 0.001),
        r_(Eigen::MatrixXd::Identity(6, 6) * 0.01),
        k_(Eigen::MatrixXd::Zero(9, 6))
    {

    }

    ~klamann_shit() = default;

    Eigen::VectorXd update(const Eigen::VectorXd& u, const Eigen::VectorXd& measurement)
    {
        x_hat_pre_ = a_matrix_ * x_hat_ + b_matrix_ * u;    //Predicts next state from model
        p_pre_ = a_matrix_ * p_ * a_matrix_.transpose() + q_;   //Predicts confidence
        Eigen::MatrixXd measurement_error = c_matrix_ * p_pre_ * c_matrix_.transpose() + r_;
        k_ = p_pre_ * c_matrix_.transpose() * measurement_error.inverse();      //Calculates kalman gain

        Eigen::VectorXd y = measurement - c_matrix_ * x_hat_pre_;
        x_hat_ = x_hat_pre_ + k_ * y;

        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(9, 9);        //Updates covarians using Joseph form for numerical stability
        p_ = (I - k_ * c_matrix_) * p_pre_ * (I - k_ * c_matrix_).transpose() + 
             k_ * r_ * k_.transpose();
        
        return c_matrix_*x_hat_;
    }

private:
    Eigen::MatrixXd a_matrix_;
    Eigen::MatrixXd b_matrix_;
    Eigen::MatrixXd c_matrix_;

    Eigen::VectorXd x_hat_;     //State estimator
    Eigen::VectorXd x_hat_pre_;     //Predicted state
    Eigen::MatrixXd p_;     //Estimation error covarians
    Eigen::MatrixXd p_pre_;     //Predicted covarians
    Eigen::MatrixXd q_;     //Process noise
    Eigen::MatrixXd r_;     //Measurement noise
    Eigen::MatrixXd k_;     //Kalman gain
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(9, 9);
};

#endif //KLAMANN_SHIT_HPP