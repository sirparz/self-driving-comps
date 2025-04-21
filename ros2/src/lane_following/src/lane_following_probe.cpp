// lane_following_probe.cpp

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <qcar2_interfaces/msg/motor_commands.hpp>
#include <opencv2/opencv.hpp>
#include <cmath>

using std::placeholders::_1;

class LaneFollowingProbe : public rclcpp::Node
{
public:
  LaneFollowingProbe()
  : Node("lane_following_probe")
  {
    this->declare_parameter<std::string>("camera_topic", "/camera/csi_front_image");
    this->declare_parameter<double>("frame_rate", 30.0);
    this->declare_parameter<int>("frame_width", 820);
    this->declare_parameter<int>("frame_height", 410);

    camera_topic_ = this->get_parameter("camera_topic").as_string();
    fps_ = this->get_parameter("frame_rate").as_double();
    camera_width_ = this->get_parameter("frame_width").as_int();
    camera_height_ = this->get_parameter("frame_height").as_int();

    image_sub_ = image_transport::create_subscription(
      this, camera_topic_,
      std::bind(&LaneFollowingProbe::image_callback, this, _1),
      "raw");

    motor_pub_ = this->create_publisher<qcar2_interfaces::msg::MotorCommands>(
      "/qcar2_motor_speed_cmd", 10);

    last_display_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Lane Following Probe Node Started");
    RCLCPP_INFO(this->get_logger(), "Subscribed to topic: %s", camera_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Target FPS: %.2f", fps_);
  }

private:
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    try {
      cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

      // Scale crop based on original 1640x820 crop [524:674, 0:820]
      double y_start_ratio = 524.0 / 820.0;
      double crop_height_ratio = 150.0 / 820.0;
      double crop_width_ratio = 820.0 / 1640.0;

      int crop_y = static_cast<int>(y_start_ratio * camera_height_);
      int crop_h = static_cast<int>(crop_height_ratio * camera_height_);
      int crop_w = static_cast<int>(crop_width_ratio * camera_width_);

      cv::Rect roi(0, crop_y, crop_w, crop_h);
      if (frame.cols < (roi.x + roi.width) || frame.rows < (roi.y + roi.height)) {
        RCLCPP_WARN(this->get_logger(), "Image too small for scaled crop, skipping frame");
        return;
      }
      cv::Mat cropped = frame(roi);

      // Convert to HSV and threshold for yellow
      cv::Mat hsv, binary;
      cv::cvtColor(cropped, hsv, cv::COLOR_BGR2HSV);
      cv::inRange(hsv, cv::Scalar(10, 50, 100), cv::Scalar(45, 255, 255), binary);

      // Find non-zero points and fit a line (slope, intercept)
      std::vector<cv::Point> points;
      cv::findNonZero(binary, points);

      double slope = 0.0, intercept = 0.0;
      bool valid = false;

      if (points.size() > 10) {
        cv::Vec4f line;
        cv::fitLine(points, line, cv::DIST_L2, 0, 0.01, 0.01);
        float vx = line[0], vy = line[1], x0 = line[2], y0 = line[3];
        slope = vy / vx;
        intercept = y0 - slope * x0;
        valid = true;
      }

      // Compute steering from slope/intercept
      double steering = 0.0;
      if (valid) {
        double rawSteering = 1.5 * (slope - 0.3419) + (1.0 / 150.0) * (intercept + 5.0);
        steering = std::clamp(rawSteering, -0.5, 0.5);
      }

      // Publish command if valid
      auto cmd = qcar2_interfaces::msg::MotorCommands();
      cmd.motor_names = {"motor_throttle", "steering_angle"};
      cmd.values = valid ? std::vector<double>{0.25 * std::cos(steering), steering}
                         : std::vector<double>{0.0, 0.0};
      motor_pub_->publish(cmd);

      // Overlay for debug
      cv::Mat overlay = cropped.clone();
      for (int i = 0; i < overlay.rows; ++i) {
        for (int j = 0; j < overlay.cols; ++j) {
          if (binary.at<uchar>(i, j) > 0) {
            overlay.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 255);
          }
        }
      }

      // Throttle display
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
  rclcpp::Publisher<qcar2_interfaces::msg::MotorCommands>::SharedPtr motor_pub_;
  rclcpp::Time last_display_time_;
  std::string camera_topic_;
  double fps_;
  int camera_width_;
  int camera_height_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaneFollowingProbe>());
  rclcpp::shutdown();
  return 0;
}
