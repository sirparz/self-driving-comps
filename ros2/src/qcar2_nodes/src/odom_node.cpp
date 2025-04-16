#include <chrono>
#include <functional>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

class OdomNode : public rclcpp::Node
{
public:
    OdomNode() : Node("odom_node"),
                 x_(0.0), y_(0.0), theta_(0.0),
                 prev_encoder_(0.0),
                 current_encoder_(0.0),
                 initial_encoder_(0.0),
                 imu_angular_z_(0.0),
                 initialized_(false)
    {
        counts_per_rev_ = 2880.0;
        wheel_radius_ = 0.033;
        // Originally, raw_scale = ((13.0*19.0)/(70.0*30.0)) ≈ 0.1176.
        // Remove the extra 50% reduction and use raw_scale directly.
        double raw_scale = ((13.0 * 19.0) / (70.0 * 30.0));
        distance_per_count_ = raw_scale * (2 * M_PI * wheel_radius_) / counts_per_rev_;

        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                        "qcar2_joint", 10, std::bind(&OdomNode::joint_callback, this, _1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
                        "qcar2_imu", 10, std::bind(&OdomNode::imu_callback, this, _1));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        last_time_ = this->now();
        timer_ = this->create_wall_timer(20ms, std::bind(&OdomNode::timer_callback, this));
    }
    
    ~OdomNode()
    {
        // On shutdown, publish a reset odometry (all zeros) to "clear" the odom value.
        auto reset_time = this->now();
        auto odom = nav_msgs::msg::Odometry();
        odom.header.stamp = reset_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = 0.0;
        odom.pose.pose.position.y = 0.0;
        odom.pose.pose.position.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();
        odom.twist.twist.linear.x = 0.0;
        odom.twist.twist.angular.z = 0.0;
        odom_pub_->publish(odom);
        
        geometry_msgs::msg::TransformStamped odom_tf;
        odom_tf.header.stamp = reset_time;
        odom_tf.header.frame_id = "odom";
        odom_tf.child_frame_id = "base_link";
        odom_tf.transform.translation.x = 0.0;
        odom_tf.transform.translation.y = 0.0;
        odom_tf.transform.translation.z = 0.0;
        odom_tf.transform.rotation.x = q.x();
        odom_tf.transform.rotation.y = q.y();
        odom_tf.transform.rotation.z = q.z();
        odom_tf.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(odom_tf);
    }

private:
    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if(!msg->position.empty())
        {
            // On the first callback, record the initial encoder value.
            if (!initialized_)
            {
                initial_encoder_ = msg->position[0];
                current_encoder_ = 0.0;
                prev_encoder_ = 0.0;
                initialized_ = true;
            }
            else
            {
                // Subtract the initial offset so the effective position starts at 0.
                current_encoder_ = msg->position[0] - initial_encoder_;
            }
        }
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Use angular velocity from IMU (z-axis)
        imu_angular_z_ = msg->angular_velocity.z;
    }

    void timer_callback()
    {
        auto current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        // Calculate distance traveled using the new distance_per_count_
        double delta_counts = current_encoder_ - prev_encoder_;
        prev_encoder_ = current_encoder_;
        double delta_distance = delta_counts * distance_per_count_;

        // Update pose estimates. Use IMU angular velocity for orientation.
        theta_ += imu_angular_z_ * dt;
        x_ += delta_distance * std::cos(theta_);
        y_ += delta_distance * std::sin(theta_);

        // Compute measured speed in meters per second.
        double joint_speed_measured = delta_counts / dt; // Example calculation for joint speed
        double measured_speed = (joint_speed_measured / (720.0 * 4.0)) * ((13.0 * 19.0) / (70.0 * 30.0)) * (2.0 * M_PI) * 0.033;

        // Publish Odometry message.
        auto odom = nav_msgs::msg::Odometry();
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();
        if(dt > 0)
        {
            odom.twist.twist.linear.x = delta_distance / dt;
            odom.twist.twist.angular.z = imu_angular_z_;
        }
        else
        {
            odom.twist.twist.linear.x = 0.0;
            odom.twist.twist.angular.z = 0.0;
        }
        odom_pub_->publish(odom);

        // Broadcast TF transform.
        geometry_msgs::msg::TransformStamped odom_tf;
        odom_tf.header.stamp = current_time;
        odom_tf.header.frame_id = "odom";
        odom_tf.child_frame_id = "base_link";
        odom_tf.transform.translation.x = x_;
        odom_tf.transform.translation.y = y_;
        odom_tf.transform.translation.z = 0.0;
        odom_tf.transform.rotation.x = q.x();
        odom_tf.transform.rotation.y = q.y();
        odom_tf.transform.rotation.z = q.z();
        odom_tf.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(odom_tf);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_time_;

    // Odometry state
    double x_, y_, theta_;
    double prev_encoder_, current_encoder_;
    double initial_encoder_;
    double imu_angular_z_;

    double counts_per_rev_;
    double wheel_radius_;
    double distance_per_count_;

    // New member variable to check for first joint callback.
    bool initialized_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomNode>());
    rclcpp::shutdown();
    return 0;
}
