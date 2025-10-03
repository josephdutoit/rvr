#include "rclcpp/rclcpp.hpp"
#include "rvr_msgs/msg/gimbal_msg.hpp"
#include <ncurses.h>
#include <chrono>
#include <thread>

class KeyboardNode : public rclcpp::Node
{
public:
    KeyboardNode() : Node("keyboard_node"), x_(0.0), y_(0.0)
    {
        publisher_ = this->create_publisher<rvr_msgs::msg::GimbalMsg>("cmd_gimbal", 10);

        // Initialize ncurses for keyboard input
        initscr();
        cbreak();
        noecho();
        keypad(stdscr, TRUE);
        nodelay(stdscr, TRUE);

        RCLCPP_INFO(this->get_logger(), "Keyboard Node started. Use arrow keys to control the gimbal.");
        RCLCPP_INFO(this->get_logger(), "Press 'q' to quit.");

        // Start the input loop in a separate thread
        input_thread_ = std::thread(&KeyboardNode::keyboard_loop, this);
    }

    ~KeyboardNode()
    {
        stop_thread_ = true;
        if (input_thread_.joinable()) {
            input_thread_.join();
        }

        // Restore terminal settings
        endwin();
    }

private:
    void keyboard_loop()
    {
        while (rclcpp::ok() && !stop_thread_) {
            int ch = getch();
            bool updated = false;

            switch (ch) {
                case KEY_UP:
                    if (y_ < 90.0) {
                        y_ += 1.0; // Increment Y position
                        updated = true;
                    }
                    break;
                case KEY_DOWN:
                    if (y_ > -30.0) {
                        y_ -= 1.0; // Decrement Y position
                        updated = true;
                    }
                    break;
                case KEY_LEFT:
                    if (x_ > -180.0) {
                        x_ -= 1.0; // Decrement X position
                        updated = true;
                    }
                    break;
                case KEY_RIGHT:
                    if (x_ < 180.0) {
                        x_ += 1.0; // Increment X position
                        updated = true;
                    }
                    break;
                case 'q': // Quit the node
                    RCLCPP_INFO(this->get_logger(), "Quitting Keyboard Node.");
                    stop_thread_ = true;
                    return;
                default:
                    break;
            }

            if (updated) {
                publish_gimbal_position();
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    void publish_gimbal_position()
    {
        auto msg = std::make_shared<rvr_msgs::msg::GimbalMsg>();
        msg->x = x_;
        msg->y = y_;
        msg->speed = 0; // Default speed
        msg->acceleration = 0; // Default acceleration

        RCLCPP_INFO(this->get_logger(), "Publishing Gimbal Position: x=%d, y=%d", msg->x, msg->y);
        publisher_->publish(*msg);
    }

    rclcpp::Publisher<rvr_msgs::msg::GimbalMsg>::SharedPtr publisher_;
    std::thread input_thread_;
    bool stop_thread_ = false;
    double x_; // Gimbal X position
    double y_; // Gimbal Y position
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
