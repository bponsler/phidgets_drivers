#include "phidgets_imu/phidgets_imu_nodelet.h"

typedef phidgets::PhidgetsImuNodelet PhidgetsImuNodelet;

PhidgetsImuNodelet::PhidgetsImuNodelet()
  :
  rclcpp::node::Node("phidgets_imu_node")
{
  //NODELET_INFO("Initializing Phidgets IMU Nodelet");
  std::cerr << "Initializing Phidgets IMU Nodelet" << std::endl;
  
  // TODO: Do we want the single threaded or multithreaded NH?
  rclcpp::node::Node::SharedPtr nh_private = rclcpp::node::Node::make_shared("~");

  imu_ = new ImuRosI(rclcpp::node::Node::SharedPtr(this), nh_private);
}

#include "class_loader/class_loader_register_macro.h"

CLASS_LOADER_REGISTER_CLASS(PhidgetsImuNodelet, rclcpp::Node);
