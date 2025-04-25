#ifndef MOTOR_CONTROL_NODE_HPP_
#define MOTOR_CONTROL_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32.hpp>
#include <pigpio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <thread>
#include <string>
#include <cmath>

class MotorControlNode : public rclcpp::Node {
public:
    MotorControlNode();
    ~MotorControlNode();

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void timer_callback();
    void run_socket_server();
    void send_steps(int step_pin, int dir_pin, int steps);
    void cleanup();

    // ROS 2 components
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr left_steps_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr right_steps_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // GPIO pins (BCM numbering)
    const int LEFT_STEP_PIN = 17;
    const int LEFT_DIR_PIN = 18;
    const int RIGHT_STEP_PIN = 22;
    const int RIGHT_DIR_PIN = 23;

    // Robot parameters
    const double WHEEL_DIAMETER = 0.1; // meters
    const int STEPS_PER_REVOLUTION = 200;
    const double DISTANCE_PER_STEP = (M_PI * WHEEL_DIAMETER) / STEPS_PER_REVOLUTION;
    const double WHEEL_BASE = 0.3; // meters

    // State variables
    double linear_vel_ = 0.0;
    double angular_vel_ = 0.0;
    int total_left_steps_ = 0;
    int total_right_steps_ = 0;
    bool manual_control_ = false;

    // Socket server thread
    std::thread socket_thread_;
};

#endif // MOTOR_CONTROL_NODE_HPP_