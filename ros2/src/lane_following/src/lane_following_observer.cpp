// lane_following_probe.cpp

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

using std::placeholders::_1;

class LaneFollowingProbe : public rclcpp::Node
{
public:
  LaneFollowingProbe()
  : Node("lane_following_probe")
  {
    // Declare and get parameters
    this->declare_parameter<std::string>("camera_topic", "/camera/csi_front_image");
    this->declare_parameter<double>("frame_rate", 30.0);
    camera_topic_ = this->get_parameter("camera_topic").as_string();
    fps_ = this->get_parameter("frame_rate").as_double();

    // Subscribe to CSI front image topic
    image_sub_ = image_transport::create_subscription(
      this, camera_topic_,
      std::bind(&LaneFollowingProbe::image_callback, this, _1),
      "raw");

    last_display_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Lane Following Probe Node Started");
    RCLCPP_INFO(this->get_logger(), "Subscribed to topic: %s", camera_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Target FPS: %.2f", fps_);
  }

private:
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    try {
      // Convert ROS image to OpenCV
      cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

      // Convert to HSV, threshold for yellow
      cv::Mat hsv, binary;
      cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
      cv::inRange(hsv, cv::Scalar(10, 50, 100), cv::Scalar(45, 255, 255), binary);

      // Debug overlay (make yellow area red)
      cv::Mat overlay = frame.clone();
      for (int i = 0; i < overlay.rows; ++i) {
        for (int j = 0; j < overlay.cols; ++j) {
          if (binary.at<uchar>(i, j) > 0) {
            overlay.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 255);
          }
        }
      }

      // Throttle display to configured FPS
      rclcpp::Time now = this->now();
      if ((now - last_display_time_).seconds() > (1.0 / fps_)) {
        cv::imshow("Detection Overlay", overlay);
        cv::waitKey(1);
        last_display_time_ = now;
      }
    }
    catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  image_transport::Subscriber image_sub_;
  rclcpp::Time last_display_time_;
  std::string camera_topic_;
  double fps_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaneFollowingProbe>());
  rclcpp::shutdown();
  return 0;
}