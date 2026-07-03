#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class SensorStaticTF : public rclcpp::Node
{
public:
  SensorStaticTF() : Node("sensor_static_tf_node")
  {
    broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    publish_lidar_tf();
    publish_imu_tf();
  }

private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;

  void publish_lidar_tf()
  {
    geometry_msgs::msg::TransformStamped tf;

    tf.header.stamp = this->get_clock()->now();
    tf.header.frame_id = "base_link";
    tf.child_frame_id = "laser";

    tf.transform.translation.x = 0.31;
    tf.transform.translation.y = 0.0;
    tf.transform.translation.z = 0.20;

    tf.transform.rotation.x = 0.0;
    tf.transform.rotation.y = 0.0;
    tf.transform.rotation.z = 0.0;
    tf.transform.rotation.w = 1.0;

    broadcaster_->sendTransform(tf);
  }

  void publish_imu_tf()
  {
    geometry_msgs::msg::TransformStamped tf;

    tf.header.stamp = this->get_clock()->now();
    tf.header.frame_id = "base_link";
    tf.child_frame_id = "imu_link";

    tf.transform.translation.x = 0.27;
    tf.transform.translation.y = 0.00;
    tf.transform.translation.z = 0.13;

    tf.transform.rotation.x = 0.0;
    tf.transform.rotation.y = 0.0;
    tf.transform.rotation.z = 0.0;
    tf.transform.rotation.w = 1.0;

    broadcaster_->sendTransform(tf);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorStaticTF>());
  rclcpp::shutdown();
  return 0;
}
