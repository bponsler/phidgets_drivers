#include <boost/make_shared.hpp>
#include "phidgets_imu/imu_ros_i.h"

namespace phidgets {

ImuRosI::ImuRosI(rclcpp::node::Node::SharedPtr nh, rclcpp::node::Node::SharedPtr nh_private):
  Imu(),
  nh_(nh),
  nh_private_(nh_private),
  is_connected_(false),
  serial_number_(-1),
  error_number_(0),
  target_publish_freq_(0.0),
  initialized_(false)
{
  //ROS_INFO ("Starting Phidgets IMU");
  std::cerr << "Starting Phidgets IMU" << std::endl;

  // **** get parameters

  rclcpp::parameter_client::SyncParametersClient client(nh_private_);
  period_ = client.get_parameter ("period", 8);  // 8 ms
  frame_id_ = client.get_parameter ("frame_id", std::string("imu"));
  angular_velocity_stdev_ = client.get_parameter ("angular_velocity_stdev", 0.02 * (M_PI / 180.0)); // 0.02 deg/s resolution, as per manual
  linear_acceleration_stdev_ = client.get_parameter ("linear_acceleration_stdev", 300.0 * 1e-6 * G); // 300 ug as per manual
  magnetic_field_stdev_ = client.get_parameter ("magnetic_field_stdev", 0.095 * (M_PI / 180.0)); // 0.095°/s as per manual
  if (client.has_parameter("serial_number")) { // optional param serial_number, default is -1
    serial_number_ = client.get_parameter ("serial_number", -1);
    //ROS_INFO_STREAM("Searching for device with serial number: " << serial_number_);
    std::cerr << "Searching for device with serial number: " << serial_number_ << std::endl;
  }

  bool has_compass_params =
      client.has_parameter ("cc_mag_field")
      && client.has_parameter ("cc_offset0")
      && client.has_parameter ("cc_offset1")
      && client.has_parameter ("cc_offset2")
      && client.has_parameter ("cc_gain0")
      && client.has_parameter ("cc_gain1")
      && client.has_parameter ("cc_gain2")
      && client.has_parameter ("cc_t0")
      && client.has_parameter ("cc_t1")
      && client.has_parameter ("cc_t2")
      && client.has_parameter ("cc_t3")
      && client.has_parameter ("cc_t4")
      && client.has_parameter ("cc_t5");

  cc_mag_field_ = client.get_parameter ("cc_mag_field", cc_mag_field_);
  cc_offset0_ = client.get_parameter ("cc_offset0", cc_offset0_);
  cc_offset1_ = client.get_parameter ("cc_offset1", cc_offset1_);
  cc_offset2_ = client.get_parameter ("cc_offset2", cc_offset2_);
  cc_gain0_ = client.get_parameter ("cc_gain0", cc_gain0_);
  cc_gain1_ = client.get_parameter ("cc_gain1", cc_gain1_);
  cc_gain2_ = client.get_parameter ("cc_gain2", cc_gain2_);
  cc_T0_ = client.get_parameter ("cc_t0", cc_T0_);
  cc_T1_ = client.get_parameter ("cc_t1", cc_T1_);
  cc_T2_ = client.get_parameter ("cc_t2", cc_T2_);
  cc_T3_ = client.get_parameter ("cc_t3", cc_T3_);
  cc_T4_ = client.get_parameter ("cc_t4", cc_T4_);
  cc_T5_ = client.get_parameter ("cc_t5", cc_T5_);

  // **** advertise topics

  imu_publisher_ = nh_->create_publisher<ImuMsg>(
    "imu/data_raw", 5);
  mag_publisher_ = nh_->create_publisher<MagMsg>(
    "imu/mag", 5);
  cal_publisher_ = nh_->create_publisher<std_msgs::msg::Bool>(
    "imu/is_calibrated", 5);

  // Set up the topic publisher diagnostics monitor for imu/data_raw
  // 1. The frequency status component monitors if imu data is published
  // within 10% tolerance of the desired frequency of 1.0 / period
  // 2. The timstamp status component monitors the delay between
  // the header.stamp of the imu message and the real (ros) time
  // the maximum tolerable drift is +- 100ms
  target_publish_freq_ = 1000.0 / static_cast<double>(period_);
  imu_publisher_diag_ptr_ = boost::make_shared<diagnostic_updater::TopicDiagnostic>(
        "imu/data_raw",
        boost::ref(diag_updater_),
        diagnostic_updater::FrequencyStatusParam(&target_publish_freq_, &target_publish_freq_, 0.1, 5),
        diagnostic_updater::TimeStampStatusParam(-0.1, 0.1)
        );

  // **** advertise services

  cal_srv_ = nh_->create_service<std_srvs::srv::Empty>(
      "imu/calibrate",
      std::bind(&ImuRosI::calibrateService, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  // **** initialize variables and device

  // ---- IMU message

  imu_msg_.header.frame_id = frame_id_;

  // build covariance matrices

  double ang_vel_var = angular_velocity_stdev_ * angular_velocity_stdev_;
  double lin_acc_var = linear_acceleration_stdev_ * linear_acceleration_stdev_;

  for (int i = 0; i < 3; ++i)
  for (int j = 0; j < 3; ++j)
  {
    int idx = j*3 +i;

    if (i == j)
    {
      imu_msg_.angular_velocity_covariance[idx]    = ang_vel_var;
      imu_msg_.linear_acceleration_covariance[idx] = lin_acc_var;
    }
    else
    {
      imu_msg_.angular_velocity_covariance[idx]    = 0.0;
      imu_msg_.linear_acceleration_covariance[idx] = 0.0;
    }
  }

  // ---- magnetic field message

  mag_msg_.header.frame_id = frame_id_;

  // build covariance matrix

  double mag_field_var = magnetic_field_stdev_ * magnetic_field_stdev_;

  for (int i = 0; i < 3; ++i)
  for (int j = 0; j < 3; ++j)
  {
    int idx = j * 3 + i;

    if (i == j)
    {
      mag_msg_.magnetic_field_covariance[idx] = mag_field_var;
    }
    else
    {
      mag_msg_.magnetic_field_covariance[idx] = 0.0;
    }
  }

  // init diagnostics, we set the hardware id properly when the device is connected
  diag_updater_.setHardwareID("none");
  diag_updater_.add("IMU Driver Status", this, &phidgets::ImuRosI::phidgetsDiagnostics);

  initDevice();

  if (has_compass_params)
  {
    int result = CPhidgetSpatial_setCompassCorrectionParameters(imu_handle_, cc_mag_field_,
          cc_offset0_, cc_offset1_, cc_offset2_, cc_gain0_, cc_gain1_, cc_gain2_,
          cc_T0_, cc_T1_, cc_T2_, cc_T3_, cc_T4_, cc_T5_);
    if (result)
    {
      const char *err;
      CPhidget_getErrorDescription(result, &err);
      std::cerr << "Problem while trying to set compass correction params: '" << err << "'." << std::endl;
    }
  }
  else
  {
    //ROS_INFO("No compass correction params found.");
    std::cerr << "No compass correction params found." << std::endl;
  }
}

void ImuRosI::initDevice()
{
        //ROS_INFO_STREAM("Opening device");
        std::cerr << "Opening device" << std::endl;

	open(serial_number_); // optional param serial_number, default is -1

	//ROS_INFO("Waiting for IMU to be attached...");
	std::cerr << "Waiting for IMU to be attached..." << std::endl;
	int result = waitForAttachment(10000);
	if(result)
	{
		is_connected_ = false;
		error_number_ = result;
		diag_updater_.force_update();
		const char *err;
		CPhidget_getErrorDescription(result, &err);
		//ROS_FATAL("Problem waiting for IMU attachment: %s Make sure the USB cable is connected and you have executed the phidgets_api/share/setup-udev.sh script.", err);
		std::cerr << "Problem waiting for IMU attachment: " << err << " Make sure the USB cable is connected and you have executed the phidgets_api/share/setup-udev.sh script." << std::endl;
	}

  // calibrate on startup
  calibrate();

  // set the hardware id for diagnostics
  diag_updater_.setHardwareIDf("%s-%d",
                               getDeviceName().c_str(),
                               getDeviceSerialNumber());
}

bool ImuRosI::calibrateService(const std::shared_ptr<rmw_request_id_t> request_header,
			       const std::shared_ptr<std_srvs::srv::Empty::Request> req,
			       const std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
  calibrate();
  return true;
}

void ImuRosI::calibrate()
{
  //ROS_INFO("Calibrating IMU...");
  std::cerr << "Calibrating IMU..." << std::endl;
  zero();
  //ROS_INFO("Calibrating IMU done.");
  std::cerr << "Calibrating IMU done." << std::endl;

  time_zero_ = ros2_time::Time::now();

  // publish message
  std_msgs::msg::Bool is_calibrated_msg;
  is_calibrated_msg.data = true;
  cal_publisher_->publish(is_calibrated_msg);
}

void ImuRosI::processImuData(CPhidgetSpatial_SpatialEventDataHandle* data, int i)
{
  // **** calculate time from timestamp
  ros2_time::Duration time_imu(data[i]->timestamp.seconds +
			       data[i]->timestamp.microseconds * 1e-6);

  ros2_time::Time time_now = time_zero_ + time_imu;

  double timediff = time_now.toSec() - ros2_time::Time::now().toSec();
  if (fabs(timediff) > 0.1) {
    //ROS_WARN("IMU time lags behind by %f seconds, resetting IMU time offset!", timediff);
    std::cerr << "IMU time lags behind by " << timediff << " seconds, resetting IMU time offset!" << std::endl;
    time_zero_ = ros2_time::Time::now() - time_imu;
    time_now = ros2_time::Time::now();
  }

  // **** initialize if needed

  if (!initialized_)
  {
    last_imu_time_ = time_now;
    initialized_ = true;
  }

  // **** create and publish imu message

  std::shared_ptr<ImuMsg> imu_msg =
    std::make_shared<ImuMsg>(imu_msg_);

  imu_msg->header.stamp = time_now.toStamp();

  // set linear acceleration
  imu_msg->linear_acceleration.x = - data[i]->acceleration[0] * G;
  imu_msg->linear_acceleration.y = - data[i]->acceleration[1] * G;
  imu_msg->linear_acceleration.z = - data[i]->acceleration[2] * G;

  // set angular velocities
  imu_msg->angular_velocity.x = data[i]->angularRate[0] * (M_PI / 180.0);
  imu_msg->angular_velocity.y = data[i]->angularRate[1] * (M_PI / 180.0);
  imu_msg->angular_velocity.z = data[i]->angularRate[2] * (M_PI / 180.0);

  imu_publisher_->publish(imu_msg);
  imu_publisher_diag_ptr_->tick(time_now);

  // **** create and publish magnetic field message

  std::shared_ptr<MagMsg> mag_msg =
    std::make_shared<MagMsg>(mag_msg_);

  mag_msg->header.stamp = time_now.toStamp();

  if (data[i]->magneticField[0] != PUNK_DBL)
  {
    // device reports data in Gauss, multiply by 1e-4 to convert to Tesla
    mag_msg->magnetic_field.x = data[i]->magneticField[0] * 1e-4;
    mag_msg->magnetic_field.y = data[i]->magneticField[1] * 1e-4;
    mag_msg->magnetic_field.z = data[i]->magneticField[2] * 1e-4;
  }
  else
  {
    double nan = std::numeric_limits<double>::quiet_NaN();

    mag_msg->magnetic_field.x = nan;
    mag_msg->magnetic_field.y = nan;
    mag_msg->magnetic_field.z = nan;
  }

  mag_publisher_->publish(mag_msg);

  // diagnostics
  diag_updater_.update();
}

void ImuRosI::dataHandler(CPhidgetSpatial_SpatialEventDataHandle *data, int count)
{
  for(int i = 0; i < count; i++)
    processImuData(data, i);
}

void ImuRosI::attachHandler()
{
  Imu::attachHandler();
  is_connected_ = true;
  // Reset error number to no error if the prev error was disconnect
  if (error_number_ == 13) error_number_ = 0;
  diag_updater_.force_update();

  // Set device params. This is in attachHandler(), since it has to be repeated on reattachment.
  setDataRate(period_);
}

void ImuRosI::detachHandler()
{
  Imu::detachHandler();
  is_connected_ = false;
  diag_updater_.force_update();
}

void ImuRosI::errorHandler(int error)
{
  Imu::errorHandler(error);
  error_number_ = error;
  diag_updater_.force_update();
}

void ImuRosI::phidgetsDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (is_connected_)
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "The Phidget is connected.");
    stat.add("Device Serial Number", getDeviceSerialNumber());
    stat.add("Device Name", getDeviceName());
    stat.add("Device Type", getDeviceType());
  }
  else
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "The Phidget is not connected. Check the USB.");
  }

  if (error_number_ != 0)
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "The Phidget reports error.");
    stat.add("Error Number", error_number_);
    stat.add("Error message", getErrorDescription(error_number_));
  }
}

} // namespace phidgets

