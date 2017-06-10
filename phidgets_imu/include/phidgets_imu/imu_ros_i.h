#ifndef PHIDGETS_IMU_IMU_ROS_I_H
#define PHIDGETS_IMU_IMU_ROS_I_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <tf2/transform_datatypes.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <phidgets_api/imu.h>

using namespace std;

namespace phidgets {

const float G = 9.81;

class ImuRosI : public Imu
{
  typedef sensor_msgs::msg::Imu              ImuMsg;
  typedef sensor_msgs::msg::MagneticField    MagMsg;

  public:

    ImuRosI(rclcpp::node::Node::SharedPtr nh, rclcpp::node::Node::SharedPtr nh_private);

    bool calibrateService(const std::shared_ptr<rmw_request_id_t> requestHeader,
			  const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                          const std::shared_ptr<std_srvs::srv::Empty::Response> res);

  private:

    rclcpp::node::Node::SharedPtr nh_;
    rclcpp::node::Node::SharedPtr nh_private_;
    rclcpp::publisher::Publisher<ImuMsg>::SharedPtr  imu_publisher_;
    rclcpp::publisher::Publisher<MagMsg>::SharedPtr  mag_publisher_;
    rclcpp::publisher::Publisher<std_msgs::msg::Bool>::SharedPtr  cal_publisher_;
    rclcpp::service::Service<std_srvs::srv::Empty>::SharedPtr cal_srv_;

    /**@brief updater object of class Update. Used to add diagnostic tasks, set ID etc. refer package API.
     * Added for diagnostics */
    diagnostic_updater::Updater diag_updater_;
    boost::shared_ptr<diagnostic_updater::TopicDiagnostic> imu_publisher_diag_ptr_;

    // diagnostics
    bool is_connected_;
    int error_number_;
    double target_publish_freq_;

    bool initialized_;
    boost::mutex mutex_;
    ros2_time::Time last_imu_time_;
    int serial_number_;

    ImuMsg imu_msg_;
    MagMsg mag_msg_;

    ros2_time::Time time_zero_;

    // params

    std::string frame_id_;
    int period_;  // rate in ms

    double angular_velocity_stdev_;
    double linear_acceleration_stdev_;
    double magnetic_field_stdev_;

    // compass correction params (see http://www.phidgets.com/docs/1044_User_Guide)
    double cc_mag_field_;
    double cc_offset0_;
    double cc_offset1_;
    double cc_offset2_;
    double cc_gain0_;
    double cc_gain1_;
    double cc_gain2_;
    double cc_T0_;
    double cc_T1_;
    double cc_T2_;
    double cc_T3_;
    double cc_T4_;
    double cc_T5_;

    void calibrate();
    void initDevice();
    void dataHandler(CPhidgetSpatial_SpatialEventDataHandle* data, int count);
    void attachHandler();
    void detachHandler();
    void errorHandler(int error);
    void processImuData(CPhidgetSpatial_SpatialEventDataHandle* data, int i);

    /**@brief Main diagnostic method that takes care of collecting diagnostic data.
     * @param stat The stat param is what is the diagnostic tasks are added two. Internally published by the
     * 		    diagnostic_updater package.
     * Added for diagnostics */
    void phidgetsDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
};

} //namespace phidgets

#endif // PHIDGETS_IMU_IMU_ROS_I_H
