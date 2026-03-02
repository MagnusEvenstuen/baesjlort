#include "controller_input/controller_input.hpp"
#include "SFML/Window/Joystick.hpp"

ControllerInput::ControllerInput()
    : rclcpp::Node("controller_input")
{
    timer_ = create_wall_timer(std::chrono::milliseconds(50),
        std::bind(&ControllerInput::timerCallback, this));

    controller_state_publisher_ = create_publisher<ControllerState>("/controller_state", 10);
}

void ControllerInput::timerCallback() const
{
    // Must be manually called when not using sf::Window
    sf::Joystick::update();
    if (sf::Joystick::isConnected(0))
    {
        ControllerState msg;
        msg.header.frame_id = "controller";
        msg.header.stamp = get_clock()->now();
        msg.lx = sf::Joystick::getAxisPosition(0, sf::Joystick::Axis::X);
        msg.ly = sf::Joystick::getAxisPosition(0, sf::Joystick::Axis::Y);
        msg.rx = sf::Joystick::getAxisPosition(0, sf::Joystick::Axis::U);
        msg.ry = sf::Joystick::getAxisPosition(0, sf::Joystick::Axis::V);
        msg.cross = sf::Joystick::isButtonPressed(0, Button::Cross);
        msg.circle = sf::Joystick::isButtonPressed(0, Button::Circle);
        msg.triangle = sf::Joystick::isButtonPressed(0, Button::Triangle);
        msg.square = sf::Joystick::isButtonPressed(0, Button::Square);
        msg.l1 = sf::Joystick::isButtonPressed(0, Button::L1);
        msg.r1 = sf::Joystick::isButtonPressed(0, Button::R1);
        msg.l2 = sf::Joystick::isButtonPressed(0, Button::L2);
        msg.r2 = sf::Joystick::isButtonPressed(0, Button::R2);
        msg.select = sf::Joystick::isButtonPressed(0, Button::Select);
        msg.start = sf::Joystick::isButtonPressed(0, Button::Start);
        msg.ps = sf::Joystick::isButtonPressed(0, Button::PS);
        msg.l3 = sf::Joystick::isButtonPressed(0, Button::L3);
        msg.r3 = sf::Joystick::isButtonPressed(0, Button::R3);
        msg.up = sf::Joystick::isButtonPressed(0, Button::Up);
        msg.down = sf::Joystick::isButtonPressed(0, Button::Down);
        msg.left = sf::Joystick::isButtonPressed(0, Button::Left);
        msg.right = sf::Joystick::isButtonPressed(0, Button::Right);

        controller_state_publisher_->publish(msg);
    }
    else
    {
        RCLCPP_INFO(get_logger(), "No controller detected");
    }
}
