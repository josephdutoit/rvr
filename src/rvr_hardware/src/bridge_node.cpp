#include "rclcpp/rclcpp.hpp"
#include "rvr_msgs/msg/gimbal_msg.hpp"
#include "rvr_msgs/msg/drive_msg.hpp"
#include "rvr_msgs/msg/telemetry_msg.hpp"
#include "rvr_msgs/msg/imu_msg.hpp"
#include "rvr_msgs/msg/odometry_msg.hpp"
#include "json/json.h"
#include <libserialport.h>
#include <thread>
#include <chrono>
#include <mutex>

double MAX_LIN_VEL = 1.0; // m/s
double MAX_ANG_VEL = 4.0; // rad/s

class JSONBridgeNode : public rclcpp::Node
{
public:
    JSONBridgeNode() : Node("json_bridge_node"), stop_thread_(false)
    {
        cmd_gimbal_subscription_ = this->create_subscription<rvr_msgs::msg::GimbalMsg>(
            "cmd_gimbal", 10, std::bind(&JSONBridgeNode::cmd_gimbal_callback, this, std::placeholders::_1)
        );
        cmd_drive_subscription_ = this->create_subscription<rvr_msgs::msg::DriveMsg>(
            "cmd_drive", 10, std::bind(&JSONBridgeNode::cmd_drive_callback, this, std::placeholders::_1)
        );

        telemetry_publisher_ = this->create_publisher<rvr_msgs::msg::TelemetryMsg>("telemetry", 10);
        imu_publisher_ = this->create_publisher<rvr_msgs::msg::ImuMsg>("imu", 10);
        odometry_publisher_ = this->create_publisher<rvr_msgs::msg::OdometryMsg>("odometry", 10);

        sp_return error = sp_get_port_by_name("/dev/ttyTHS1", &serial_port_);
        if (error != SP_OK) {
            RCLCPP_ERROR(this->get_logger(), "Failed to find serial port!");
            return;
        }

        error = sp_open(serial_port_, SP_MODE_READ_WRITE);
        if (error != SP_OK) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port!");
            return;
        }

        sp_set_baudrate(serial_port_, 115200);

        read_thread_ = std::thread(&JSONBridgeNode::read_serial_data, this);
    }

    ~JSONBridgeNode()
    {
        stop_thread_ = true;
        if (read_thread_.joinable()) {
            read_thread_.join();
        }
        sp_close(serial_port_);
        sp_free_port(serial_port_);
    }

private:
    void json_cmd(const Json::Value &json_msg)
    {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        Json::StreamWriterBuilder writer;
        writer["indentation"] = "";
        std::string msg = Json::writeString(writer, json_msg) + "\n";

        sp_nonblocking_write(serial_port_, msg.c_str(), msg.size());
    }

    void cmd_drive_callback(const rvr_msgs::msg::DriveMsg::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(message_mutex_);
            latest_drive_msg_ = *msg;  // Store the latest message
        }
        process_drive_message();
    }

    void cmd_gimbal_callback(const rvr_msgs::msg::GimbalMsg::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(message_mutex_);
            latest_gimbal_msg_ = *msg;  // Store the latest message
        }
        process_gimbal_message();
    }

    void process_drive_message()
    {
        std::lock_guard<std::mutex> lock(message_mutex_);
        Json::Value command;
        command["T"] = 13;
        command["X"] = -latest_drive_msg_.linear_vel * MAX_LIN_VEL; // The chassis is inverted
        command["Z"] = -latest_drive_msg_.angular_vel * MAX_ANG_VEL; // The chassis is inverted

        json_cmd(command);
    }

    void process_gimbal_message()
    {
        std::lock_guard<std::mutex> lock(message_mutex_);
        Json::Value command;
        command["T"] = 133;
        command["X"] = latest_gimbal_msg_.x;
        command["Y"] = latest_gimbal_msg_.y;
        command["SPD"] = latest_gimbal_msg_.speed;
        command["ACC"] = latest_gimbal_msg_.acceleration;

        json_cmd(command);
    }

    void read_serial_data()
    {
        char buffer[256];
        bool json_started = false;
        std::string json_buffer;
        while (rclcpp::ok() && !stop_thread_) {
            int bytes_read = sp_nonblocking_read(serial_port_, buffer, sizeof(buffer) - 1);
            if (bytes_read > 0) {
                for (int i = 0; i < bytes_read; ++i) {
                    char c = buffer[i];
                    if (c == '{') {
                        json_started = true;
                        json_buffer.clear();
                    }
                    if (json_started) {
                        json_buffer += c;
                    }
                    if (c == '}') {
                        json_started = false;
                        handle_telemetry_data(json_buffer);
                        json_buffer.clear();
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void handle_telemetry_data(const std::string &json_data)
    {
        Json::CharReaderBuilder reader;
        Json::Value root;
        std::string errs;

        std::istringstream s(json_data);
        if (!Json::parseFromStream(reader, s, &root, &errs)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse JSON: %s", errs.c_str());
            return;
        }

        // Publish telemetry data
        auto telemetry_msg = rvr_msgs::msg::TelemetryMsg();
        telemetry_msg.header.stamp = this->get_clock()->now();  // Add timestamp
        telemetry_msg.header.frame_id = "telemetry_frame";      // Add frame ID
        telemetry_msg.voltage = root["v"].asDouble();
        telemetry_msg.pan = root["pan"].asDouble();
        telemetry_msg.tilt = root["tilt"].asDouble();
        telemetry_publisher_->publish(telemetry_msg);

        // Publish IMU data
        auto imu_msg = rvr_msgs::msg::ImuMsg();
        imu_msg.header.stamp = this->get_clock()->now();        // Add timestamp
        imu_msg.header.frame_id = "imu_frame";                 // Add frame ID
        imu_msg.gx = root["gx"].asDouble();
        imu_msg.gy = root["gy"].asDouble();
        imu_msg.gz = root["gz"].asDouble();
        imu_msg.ax = root["ax"].asDouble();
        imu_msg.ay = root["ay"].asDouble();
        imu_msg.az = root["az"].asDouble();
        imu_msg.mx = root["mx"].asDouble();
        imu_msg.my = root["my"].asDouble();
        imu_msg.mz = root["mz"].asDouble();
        imu_publisher_->publish(imu_msg);

        // Publish odometry data
        auto odometry_msg = rvr_msgs::msg::OdometryMsg();
        odometry_msg.header.stamp = this->get_clock()->now();   // Add timestamp
        odometry_msg.header.frame_id = "odom_frame";           // Add frame ID
        odometry_msg.left_distance = root["odl"].asDouble();
        odometry_msg.right_distance = root["odr"].asDouble();
        odometry_publisher_->publish(odometry_msg);
    }

    rclcpp::Subscription<rvr_msgs::msg::GimbalMsg>::SharedPtr cmd_gimbal_subscription_;
    rclcpp::Subscription<rvr_msgs::msg::DriveMsg>::SharedPtr cmd_drive_subscription_;
    struct sp_port *serial_port_;  // Use the libserialport C library
    std::thread read_thread_;
    std::mutex serial_mutex_;
    bool stop_thread_;

    rvr_msgs::msg::DriveMsg latest_drive_msg_;
    rvr_msgs::msg::GimbalMsg latest_gimbal_msg_;
    std::mutex message_mutex_;

    rclcpp::Publisher<rvr_msgs::msg::TelemetryMsg>::SharedPtr telemetry_publisher_;
    rclcpp::Publisher<rvr_msgs::msg::ImuMsg>::SharedPtr imu_publisher_;
    rclcpp::Publisher<rvr_msgs::msg::OdometryMsg>::SharedPtr odometry_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JSONBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
