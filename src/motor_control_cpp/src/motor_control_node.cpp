#include "motor_control_cpp/motor_control_node.hpp"
#include <unistd.h>
#include <arpa/inet.h>
#include <iostream>

MotorControlNode::MotorControlNode() : Node("motor_control_node") {
    // Initialize ROS 2 components
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&MotorControlNode::cmd_vel_callback, this, std::placeholders::_1));
    left_steps_pub_ = this->create_publisher<std_msgs::msg::Int32>("/left_steps", 10);
    right_steps_pub_ = this->create_publisher<std_msgs::msg::Int32>("/right_steps", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&MotorControlNode::timer_callback, this));

    // Initialize pigpio
    if (gpioInitialise() < 0) {
        RCLCPP_ERROR(this->get_logger(), "pigpio initialization failed");
        throw std::runtime_error("pigpio initialization failed");
    }

    // Set GPIO modes
    gpioSetMode(LEFT_STEP_PIN, PI_OUTPUT);
    gpioSetMode(LEFT_DIR_PIN, PI_OUTPUT);
    gpioSetMode(RIGHT_STEP_PIN, PI_OUTPUT);
    gpioSetMode(RIGHT_DIR_PIN, PI_OUTPUT);

    // Initialize GPIO pins to low
    gpioWrite(LEFT_STEP_PIN, 0);
    gpioWrite(LEFT_DIR_PIN, 0);
    gpioWrite(RIGHT_STEP_PIN, 0);
    gpioWrite(RIGHT_DIR_PIN, 0);

    // Start socket server thread
    socket_thread_ = std::thread(&MotorControlNode::run_socket_server, this);

    RCLCPP_INFO(this->get_logger(), "Motor Control Node with Socket Server started");
}

MotorControlNode::~MotorControlNode() {
    cleanup();
    if (socket_thread_.joinable()) {
        socket_thread_.join();
    }
    gpioTerminate();
    RCLCPP_INFO(this->get_logger(), "Motor Control Node shutdown");
}

void MotorControlNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (!manual_control_) {
        linear_vel_ = msg->linear.x;
        angular_vel_ = msg->angular.z;
    }
}

void MotorControlNode::run_socket_server() {
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        RCLCPP_ERROR(this->get_logger(), "Socket creation failed");
        return;
    }

    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(12345);

    if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Bind failed");
        close(server_fd);
        return;
    }

    if (listen(server_fd, 5) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Listen failed");
        close(server_fd);
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Socket server listening on port 12345");

    while (rclcpp::ok()) {
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);
        int client_fd = accept(server_fd, (struct sockaddr*)&client_addr, &addr_len);
        if (client_fd < 0) {
            RCLCPP_ERROR(this->get_logger(), "Accept failed");
            continue;
        }

        char client_ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &(client_addr.sin_addr), client_ip, INET_ADDRSTRLEN);
        RCLCPP_INFO(this->get_logger(), "Client connected: %s:%d", client_ip, ntohs(client_addr.sin_port));

        char buffer[1024] = {0};
        while (rclcpp::ok()) {
            int bytes_read = read(client_fd, buffer, 1024);
            if (bytes_read <= 0) {
                RCLCPP_INFO(this->get_logger(), "Client disconnected");
                break;
            }

            std::string command(buffer, bytes_read);
            command.erase(std::remove(command.begin(), command.end(), '\n'), command.end());
            RCLCPP_INFO(this->get_logger(), "Received command: %s", command.c_str());

            if (command == "forward") {
                manual_control_ = true;
                linear_vel_ = 0.1;
                angular_vel_ = 0.0;
            } else if (command == "backward") {
                manual_control_ = true;
                linear_vel_ = -0.1;
                angular_vel_ = 0.0;
            } else if (command == "left") {
                manual_control_ = true;
                linear_vel_ = 0.0;
                angular_vel_ = 0.5;
            } else if (command == "right") {
                manual_control_ = true;
                linear_vel_ = 0.0;
                angular_vel_ = -0.5;
            } else if (command == "stop") {
                manual_control_ = true;
                linear_vel_ = 0.0;
                angular_vel_ = 0.0;
            } else if (command == "auto") {
                manual_control_ = false;
                linear_vel_ = 0.0;
                angular_vel_ = 0.0;
            } else {
                RCLCPP_WARN(this->get_logger(), "Unknown command: %s", command.c_str());
            }

            std::string response = "Command received: " + command + "\n";
            send(client_fd, response.c_str(), response.length(), 0);
        }
        close(client_fd);
    }
    close(server_fd);
}

void MotorControlNode::timer_callback() {
    double left_vel = linear_vel_ - (angular_vel_ * WHEEL_BASE / 2.0);
    double right_vel = linear_vel_ + (angular_vel_ * WHEEL_BASE / 2.0);

    double left_steps_per_sec = left_vel / DISTANCE_PER_STEP;
    double right_steps_per_sec = right_vel / DISTANCE_PER_STEP;

    int left_steps = static_cast<int>(left_steps_per_sec * 0.1);
    int right_steps = static_cast<int>(right_steps_per_sec * 0.1);

    total_left_steps_ += left_steps;
    total_right_steps_ += right_steps;

    auto left_msg = std_msgs::msg::Int32();
    left_msg.data = total_left_steps_;
    auto right_msg = std_msgs::msg::Int32();
    right_msg.data = total_right_steps_;
    left_steps_pub_->publish(left_msg);
    right_steps_pub_->publish(right_msg);

    send_steps(LEFT_STEP_PIN, LEFT_DIR_PIN, left_steps);
    send_steps(RIGHT_STEP_PIN, RIGHT_DIR_PIN, right_steps);

    RCLCPP_INFO(this->get_logger(), "Steps: left=%d, right=%d", left_steps, right_steps);
}

void MotorControlNode::send_steps(int step_pin, int dir_pin, int steps) {
    if (steps == 0) return;

    gpioWrite(dir_pin, steps > 0 ? 1 : 0);
    steps = std::abs(steps);

    for (int i = 0; i < steps; ++i) {
        gpioWrite(step_pin, 1);
        usleep(500); // 0.0005 seconds
        gpioWrite(step_pin, 0);
        usleep(500);
    }
}

void MotorControlNode::cleanup() {
    gpioWrite(LEFT_STEP_PIN, 0);
    gpioWrite(LEFT_DIR_PIN, 0);
    gpioWrite(RIGHT_STEP_PIN, 0);
    gpioWrite(RIGHT_DIR_PIN, 0);
    RCLCPP_INFO(this->get_logger(), "GPIO pins cleaned up");
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<MotorControlNode>();
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }
    rclcpp::shutdown();
    return 0;
}