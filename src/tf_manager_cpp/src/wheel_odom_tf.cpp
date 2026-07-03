#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
class WheelOdomNode : public rclcpp::Node
{
public:
  WheelOdomNode()
  : Node("wheel_odom_node")
  {
    wheel_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/wheel_speed", 10,
      std::bind(&WheelOdomNode::wheelCallback, this, std::placeholders::_1));

    steer_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/steering_angle", 10,
      std::bind(&WheelOdomNode::steerCallback, this, std::placeholders::_1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&WheelOdomNode::update, this));

    last_time_ = now();
  }

private:

  double wheelbase_ = 0.37;   // 앞축과 뒷축 간 거리 (m)

  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;

  double v_ = 0.0;
  double steering_ = 0.0;

  rclcpp::Time last_time_;


  void wheelCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    v_ = msg->data;  
  }

  void steerCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    steering_ = msg->data; 
  }


  void update()
  {
    rclcpp::Time now_time = now();
    double dt = (now_time - last_time_).seconds();
    last_time_ = now_time;

    double omega = v_ / wheelbase_ * tan(steering_);

    x_ += v_ * cos(yaw_) * dt;
    y_ += v_ * sin(yaw_) * dt;
    yaw_ += omega * dt;

    publishOdom(now_time, omega);
    publishTF(now_time);
  }


  void publishOdom(const rclcpp::Time & time, double omega)
  {
    nav_msgs::msg::Odometry odom;

    odom.header.stamp = time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_);

    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x = v_;
    odom.twist.twist.angular.z = omega;

    odom_pub_->publish(odom);
  }


  void publishTF(const rclcpp::Time & time)
  {
    geometry_msgs::msg::TransformStamped tf;

    tf.header.stamp = time;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_link";

    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_);

    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(tf);
  }

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr wheel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steer_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelOdomNode>());
  rclcpp::shutdown();
  return 0;
}
