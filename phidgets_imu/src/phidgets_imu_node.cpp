#include "phidgets_imu/imu_ros_i.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::node::Node::SharedPtr nh = rclcpp::node::Node::make_shared("phidgets_imu");
  rclcpp::node::Node::SharedPtr nh_private = rclcpp::node::Node::make_shared("phidgets_imu");
  phidgets::ImuRosI imu(nh, nh_private);
  rclcpp::spin(nh);
  return 0;
}
