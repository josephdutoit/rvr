#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>


class CameraPublisherNode : public rclcpp::Node 
{
public:
    CameraPublisherNode() : Node("camera_publisher_node")
    {
        image_pub_ = image_transport::create_publisher(
            this, "camera/image"
        );

        cap_ = cv::VideoCapture(0);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Error opening video stream!");
            return;
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 10 Hz
            std::bind(&CameraPublisherNode::publish_image, this)
        );
    }

private:
    void publish_image()
    {
        cv::Mat frame;
        cap_ >> frame;

        if (!frame.empty()) {
            std_msgs::msg::Header header;
            header.stamp = this->now();
            header.frame_id = "camera_frame";

            sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(
                header, "bgr8", frame
            ).toImageMsg();

            image_pub_.publish(msg);
        } else {
            RCLCPP_WARN(this->get_logger(), "Empty frame captured!");
        }
    }

    cv::VideoCapture cap_;
    image_transport::Publisher image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
