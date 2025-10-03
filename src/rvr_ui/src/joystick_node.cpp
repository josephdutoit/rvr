#include "rclcpp/rclcpp.hpp"
#include "rvr_msgs/msg/gimbal_msg.hpp"
#include "rvr_msgs/msg/drive_msg.hpp"
#include <linux/joystick.h>
#include <fcntl.h>
#include <unistd.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <cmath>

double PI = std::atan(1.0) * 4;

class JoystickNode : public rclcpp::Node
{
public:
    JoystickNode() : Node("joystick_node"), stop_thread_(false), x_(0.0), y_(0.0)
    {
        publisher_ = this->create_publisher<rvr_msgs::msg::GimbalMsg>("cmd_gimbal", 10);
        drive_publisher_ = this->create_publisher<rvr_msgs::msg::DriveMsg>("cmd_drive", 10);

        // Open the joystick device
        const char *device_path = "/dev/input/js0";
        fd_ = open(device_path, O_RDONLY | O_NONBLOCK);
        if (fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open joystick device: %s. Is the controller connected?", device_path);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Game Controller Node started. Listening to device: %s", device_path);

        // Start the input loop in a separate thread
        input_thread_ = std::thread(&JoystickNode::joystick_loop, this);
    }

    ~JoystickNode()
    {
        stop_thread_ = true;
        if (input_thread_.joinable()) {
            input_thread_.join();
        }
        if (fd_ >= 0) {
            close(fd_);
        }
    }

private:
    void joystick_loop()
    {
        struct js_event event;
        while (rclcpp::ok() && !stop_thread_) {
            int bytes = read(fd_, &event, sizeof(event));
            if (bytes == sizeof(event)) {
                if (event.type == JS_EVENT_AXIS) {
                    bool updated_gimbal = false;
                    bool updated_drive = false;

                    // Right stick controls gimbal velocity
                    if (event.number == 3) { // Right stick horizontal axis
                        gimbal_velocity_x_ = event.value / 32767.0 * 180.0; // Scale to -180 to 180
                        updated_gimbal = true;
                    } else if (event.number == 4) { // Right stick vertical axis
                        gimbal_velocity_y_ = -event.value / 32767.0 * 60.0; // Scale to -60 to 60
                        updated_gimbal = true;
                    }

                    // Left stick controls speed and heading
                    if (event.number == 0) { // Left stick horizontal axis
                        drive_x_ = event.value / 32767.0;
                        updated_drive = true;
                    } else if (event.number == 1) { // Left stick vertical axis
                        drive_y_ = event.value / 32767.0;
                        updated_drive = true;
                    }

                    if (updated_gimbal) {
                        update_gimbal_position();
                        publish_gimbal_position();
                    }

                    if (updated_drive) {
                        publish_drive_command();
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void update_gimbal_position()
    {
        // Update gimbal position based on velocity
        gimbal_x_ += gimbal_velocity_x_ * 0.01; // Scale velocity by time step (10ms)
        gimbal_y_ += gimbal_velocity_y_ * 0.01;

        // Clamp gimbal position to valid range
        gimbal_x_ = std::clamp(gimbal_x_, -180.0, 180.0);
        gimbal_y_ = std::clamp(gimbal_y_, -30.0, 90.0);
    }

    void publish_gimbal_position()
    {
        auto msg = std::make_shared<rvr_msgs::msg::GimbalMsg>();

        // Publish the updated gimbal position
        msg->x = static_cast<int16_t>(gimbal_x_);
        msg->y = static_cast<int16_t>(gimbal_y_);
        msg->speed_x = static_cast<int16_t>(gimbal_velocity_x_);
        msg->speed_y = static_cast<int16_t>(gimbal_velocity_y_);
        msg->acceleration = 0;

        // RCLCPP_INFO(this->get_logger(), "Publishing Gimbal Position: x=%d, y=%d, speed_x=%d, speed_y=%d", 
        //             msg->x, msg->y, msg->speed_x, msg->speed_y);
        publisher_->publish(*msg);
    }

    void publish_drive_command()
    {
        auto msg = std::make_shared<rvr_msgs::msg::DriveMsg>();
        msg->linear_vel = drive_y_; //m/s
        msg->angular_vel = drive_x_;

        // RCLCPP_INFO(this->get_logger(), "Publishing Drive Command: speed=%f, heading=%f", msg->linear_vel, msg->angular_vel);
        drive_publisher_->publish(*msg);
    }

    rclcpp::Publisher<rvr_msgs::msg::GimbalMsg>::SharedPtr publisher_;
    rclcpp::Publisher<rvr_msgs::msg::DriveMsg>::SharedPtr drive_publisher_;
    std::thread input_thread_;
    bool stop_thread_;
    double x_;
    double y_;
    double drive_x_ = 0.0;
    double drive_y_ = 0.0;
    double linear_vel_ = 0.0;
    double angular_vel_ = 0.0;
    int fd_;

    double gimbal_x_ = 0.0;
    double gimbal_y_ = 0.0;
    double gimbal_velocity_x_ = 0.0;
    double gimbal_velocity_y_ = 0.0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoystickNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}