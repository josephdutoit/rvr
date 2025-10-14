#include "rclcpp/rclcpp.hpp"
#include <libserialport.h>
#include "rvr_msgs/msg/gps_msg.hpp"

class GPSNode : public rclcpp::Node {
public:
  GPSNode() : Node("gps_node") {
    // Initialize serial port
    sp_port *port;
    sp_get_port_by_name("/dev/ttyACM0", &port);
    sp_open(port, SP_MODE_READ);

    gps_publisher = this->create_publisher<rvr_msgs::msg::GPSMsg>();
    read_thread = std::thread(&GPSNode::read_gps_data, this);
  }

  ~GPSNode() {
    // Close serial port
    sp_close(port);
  }

private:
  void read_gps_data() {
    char buffer[256];
    while (rclcpp::ok()) {
      int bytes_read = sp_nonblocking_read(port, buffer, sizeof(buffer) - 1);
      if (bytes_read > 0) {
        buffer[bytes_read] = '\0';
        std::string data(buffer);
        // Parse GPS data (assuming NMEA format for this example)
        if (data.find("$GPGGA") != std::string::npos) {
          auto gps_msg = rvr_msgs::msg::GPSMsg();
          // Simple parsing logic (not robust, just for illustration)
          std::sscanf(data.c_str(), "$GPGGA,%*f,%f,%*c,%f,%*c,%*d,%*d,%*f,%f,%*f,%*f,%*f,%*f",
                      &gps_msg.latitude, &gps_msg.longitude, &gps_msg.altitude);
          gps_msg.header.stamp = this->get_clock()->now();
          gps_msg.header.frame_id = "gps_frame";
          gps_publisher->publish(gps_msg);
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  sp_port *port;
  rclcpp::Publisher<rvr_msgs::msg::GPSMsg>::SharedPtr gps_publisher;
  std::thread read_thread;


};
