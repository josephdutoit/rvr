#include "rclcpp/rclcpp.hpp"
#include "rvr_msgs/msg/gimbal_msg.hpp"
#include "utils/transforms.hpp"
#include <libevdev/libevdev.h> 
#include <fcntl.h>  // For open()
#include <unistd.h>  // For close()
#include <thread>
#include <chrono>

class KeyboardNode : public rclcpp::Node
{
public:
    KeyboardNode() : Node("keyboard_node"), x_(0.0), y_(0.0)
    {
        publisher_ = this->create_publisher<rvr_msgs::msg::GimbalMsg>("cmd_gimbal", 10);

        // Open the keyboard device (replace with your device path)
        const char* device_path = "/dev/input/event1";  // Update this based on your device
        fd_ = open(device_path, O_RDONLY | O_NONBLOCK);
        if (fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open keyboard device: %s", device_path);
            return;
        }

        if (libevdev_new_from_fd(fd_, &dev_) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize libevdev");
            close(fd_);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Keyboard Node started. Listening to device: %s", device_path);
        RCLCPP_INFO(this->get_logger(), "Use arrow keys on the robot's keyboard to control the gimbal.");

        // Start the input loop in a separate thread
        input_thread_ = std::thread(&KeyboardNode::keyboard_loop, this);
    }

    ~KeyboardNode()
    {
        stop_thread_ = true;
        if (input_thread_.joinable()) {
            input_thread_.join();
        }
        libevdev_free(dev_);
        close(fd_);
    }

private:
    void keyboard_loop()
    {
        struct input_event ev;
        while (rclcpp::ok() && !stop_thread_) {
            int rc = libevdev_next_event(dev_, LIBEVDEV_READ_FLAG_NORMAL, &ev);
            if (rc == 0) {  // Event available
                if (ev.type == EV_KEY && ev.value == 1) {  // Key press (value=1)
                    bool updated = false;
                    switch (ev.code) {
                        case KEY_UP:  // 103
                            if (y_ < 90.0) {
                                y_ += 1.0;
                                updated = true;
                            }
                            break;
                        case KEY_DOWN:  // 108
                            if (y_ > -30.0) {
                                y_ -= 1.0;
                                updated = true;
                            }
                            break;
                        case KEY_LEFT:  // 105
                            if (x_ > -180.0) {
                                x_ -= 1.0;
                                updated = true;
                            }
                            break;
                        case KEY_RIGHT:  // 106
                            if (x_ < 180.0) {
                                x_ += 1.0;
                                updated = true;
                            }
                            break;
                        case KEY_Q:  // 16 (for quit)
                            RCLCPP_INFO(this->get_logger(), "Quitting Keyboard Node.");
                            stop_thread_ = true;
                            return;
                        default:
                            break;
                    }
                    if (updated) {
                        publish_gimbal_position();
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void publish_gimbal_position()
    {
        auto msg = std::make_shared<rvr_msgs::msg::GimbalMsg>();
        msg->x = static_cast<int16_t>(x_);
        msg->y = static_cast<int16_t>(y_);
        msg->speed = 0;
        msg->acceleration = 0;

        RCLCPP_INFO(this->get_logger(), "Publishing Gimbal Position: x=%d, y=%d", msg->x, msg->y);
        publisher_->publish(*msg);
    }

    rclcpp::Publisher<rvr_msgs::msg::GimbalMsg>::SharedPtr publisher_;
    std::thread input_thread_;
    bool stop_thread_ = false;
    double x_;
    double y_;
    int fd_;
    struct libevdev* dev_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
