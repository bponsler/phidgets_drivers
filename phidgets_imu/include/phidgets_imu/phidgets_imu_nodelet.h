#ifndef PHIDGETS_IMU_PHIDGETS_IMU_NODELET_H
#define PHIDGETS_IMU_PHIDGETS_IMU_NODELET_H

#include <rclcpp/node.hpp>

#include "phidgets_imu/imu_ros_i.h"

namespace phidgets {

class PhidgetsImuNodelet : public rclcpp::node::Node
{
  public:
    PhidgetsImuNodelet();
    virtual void onInit();

  private:
    Imu * imu_;  // FIXME: change to smart pointer
};

} // namespace phidgets

#endif // PHIDGETS_IMU_PHIDGETS_IMU_NODELET_H
