#include "inertial_sense.h"
#include <chrono>
#include <stddef.h>
#include <unistd.h>
#include <tf/tf.h>
#include <ros/console.h>

#define SET_CALLBACK(DID, __type, __cb_fun) \
    IS_.BroadcastBinaryData(DID, 1, \
    [this](InertialSense*i, p_data_t* data, int pHandle)\
    { \
        /*ROS_INFO("Got message %d", DID); */\
        this->__cb_fun(reinterpret_cast<__type*>(data->buf));\
    })



InertialSenseROS::InertialSenseROS() :
  nh_(), nh_private_("~"), initialized_(false)
{
  connect();
  set_navigation_dt_ms();

  /// Start Up ROS service servers
  refLLA_set_current_srv_ = nh_.advertiseService("set_refLLA_current", &InertialSenseROS::set_current_position_as_refLLA, this);
  refLLA_set_value_srv_ = nh_.advertiseService("set_refLLA_value", &InertialSenseROS::set_refLLA_to_value, this);
  mag_cal_srv_ = nh_.advertiseService("single_axis_mag_cal", &InertialSenseROS::perform_mag_cal_srv_callback, this);
  multi_mag_cal_srv_ = nh_.advertiseService("multi_axis_mag_cal", &InertialSenseROS::perform_multi_mag_cal_srv_callback, this);
  firmware_update_srv_ = nh_.advertiseService("firmware_update", &InertialSenseROS::update_firmware_srv_callback, this);
  wheel_enc_sub_ = nh_.subscribe("joint_states", 20, &InertialSenseROS::wheel_enc_callback, this);
  armadillo_pose_sub_ = nh.subscribe("armadillo_pose", 20, &InertialSenseROS::armadillo_pose_callback, this);
  //pos_measurement = nh_.subscribe("")
  
  // Stop all broadcasts
  IS_.StopBroadcasts();

  configure_parameters();
  configure_data_streams();

  nh_private_.param<bool>("enable_log", log_enabled_, false);
  if (log_enabled_)
  {
    start_log();
  }

  configure_ascii_output();
  configure_rtk();

  initialized_ = true;
}


void InertialSenseROS::configure_data_streams()
{
  SET_CALLBACK(DID_GPS1_POS, gps_pos_t, GPS_pos_callback); // we always need GPS for Fix status
  SET_CALLBACK(DID_GPS1_VEL, gps_vel_t, GPS_vel_callback); // we always need GPS for Fix status
  SET_CALLBACK(DID_STROBE_IN_TIME, strobe_in_time_t, strobe_in_time_callback); // we always want the strobe
  nh_private_.param<bool>("stream_INS", INS_.enabled, true);
  if (INS_.enabled)
  {
    INS_.pub = nh_.advertise<nav_msgs::Odometry>("ins", 1);
    SET_CALLBACK(DID_INS_1, ins_1_t, INS1_callback);
    SET_CALLBACK(DID_INS_2, ins_2_t, INS2_callback);
    SET_CALLBACK(DID_DUAL_IMU, dual_imu_t, IMU_callback);
//    SET_CALLBACK(DID_INL2_VARIANCE, nav_dt_ms, inl2_variance_t, INS_variance_callback);
  }

  // Set up the IMU ROS stream
  nh_private_.param<bool>("stream_IMU", IMU_.enabled, false);
  if (IMU_.enabled)
  {
    IMU_.pub = nh_.advertise<sensor_msgs::Imu>("imu", 1);
    SET_CALLBACK(DID_INS_1, ins_1_t, INS1_callback);
    SET_CALLBACK(DID_INS_2, ins_2_t, INS2_callback);
    SET_CALLBACK(DID_DUAL_IMU, dual_imu_t, IMU_callback);
  }

  // Set up the GPS ROS stream - we always need GPS information for time sync, just don't always need to publish it
  nh_private_.param<bool>("stream_GPS", GPS_.enabled, false);
  if (GPS_.enabled)
    GPS_.pub = nh_.advertise<inertial_sense::GPS>("gps", 1);

  nh_private_.param<bool>("stream_GPS_raw", GPS_obs_.enabled, false);
  nh_private_.param<bool>("stream_GPS_raw", GPS_eph_.enabled, false);
  if (GPS_obs_.enabled)
  {
    GPS_obs_.pub = nh_.advertise<inertial_sense::GNSSObservation>("gps/obs", 50);
    GPS_eph_.pub = nh_.advertise<inertial_sense::GNSSEphemeris>("gps/eph", 50);
    GPS_eph_.pub2 = nh_.advertise<inertial_sense::GlonassEphemeris>("gps/geph", 50);
    SET_CALLBACK(DID_GPS1_RAW, gps_raw_t, GPS_raw_callback);
    SET_CALLBACK(DID_GPS_BASE_RAW, gps_raw_t, GPS_raw_callback);
    SET_CALLBACK(DID_GPS2_RAW, gps_raw_t, GPS_raw_callback);
  }

  // Set up the GPS info ROS stream
  nh_private_.param<bool>("stream_GPS_info", GPS_info_.enabled, false);
  if (GPS_info_.enabled)
  {
    GPS_info_.pub = nh_.advertise<inertial_sense::GPSInfo>("gps/info", 1);
    SET_CALLBACK(DID_GPS1_SAT, gps_sat_t, GPS_info_callback);
  }

  // Set up the magnetometer ROS stream
  nh_private_.param<bool>("stream_mag", mag_.enabled, false);
  if (mag_.enabled)
  {
    mag_.pub = nh_.advertise<sensor_msgs::MagneticField>("mag", 1);
    //    mag_.pub2 = nh_.advertise<sensor_msgs::MagneticField>("mag2", 1);
    SET_CALLBACK(DID_MAGNETOMETER_1, magnetometer_t, mag_callback);
  }

  // Set up the barometer ROS stream
  nh_private_.param<bool>("stream_baro", baro_.enabled, false);
  if (baro_.enabled)
  {
    baro_.pub = nh_.advertise<sensor_msgs::FluidPressure>("baro", 1);
    SET_CALLBACK(DID_BAROMETER, barometer_t, baro_callback);
  }

  // Set up the preintegrated IMU (coning and sculling integral) ROS stream
  nh_private_.param<bool>("stream_preint_IMU", dt_vel_.enabled, false);
  if (dt_vel_.enabled)
  {
    dt_vel_.pub = nh_.advertise<inertial_sense::PreIntIMU>("preint_imu", 1);
    SET_CALLBACK(DID_PREINTEGRATED_IMU, preintegrated_imu_t, preint_IMU_callback);
  }

}

void InertialSenseROS::start_log()
{
  std::string filename = cISLogger::CreateCurrentTimestamp();
  ROS_INFO_STREAM("Creating log in " << filename << " folder");
  IS_.SetLoggerEnabled(true, filename, cISLogger::LOGTYPE_DAT, RMC_PRESET_PPD_ROBOT);
}

void InertialSenseROS::configure_ascii_output()
{
  //  int NMEA_rate = nh_private_.param<int>("NMEA_rate", 0);
  //  int NMEA_message_configuration = nh_private_.param<int>("NMEA_configuration", 0x00);
  //  int NMEA_message_ports = nh_private_.param<int>("NMEA_ports", 0x00);
  //  ascii_msgs_t msgs = {};
  //  msgs.options = (NMEA_message_ports & NMEA_SER0) ? RMC_OPTIONS_PORT_SER0 : 0; // output on serial 0
  //  msgs.options |= (NMEA_message_ports & NMEA_SER1) ? RMC_OPTIONS_PORT_SER1 : 0; // output on serial 1
  //  msgs.gpgga = (NMEA_message_configuration & NMEA_GPGGA) ? NMEA_rate : 0;
  //  msgs.gpgll = (NMEA_message_configuration & NMEA_GPGLL) ? NMEA_rate : 0;
  //  msgs.gpgsa = (NMEA_message_configuration & NMEA_GPGSA) ? NMEA_rate : 0;
  //  msgs.gprmc = (NMEA_message_configuration & NMEA_GPRMC) ? NMEA_rate : 0;
  //  IS_.SendData(DID_ASCII_BCAST_PERIOD, (uint8_t*)(&msgs), sizeof(ascii_msgs_t), 0);

}

void InertialSenseROS::connect()
{
  nh_private_.param<std::string>("port", port_, "/dev/ttyUSB0");
  nh_private_.param<int>("baudrate", baudrate_, 921600);
  nh_private_.param<std::string>("frame_id", frame_id_, "body");

  /// Connect to the uINS
  ROS_INFO("Connecting to serial port \"%s\", at %d baud", port_.c_str(), baudrate_);
  if (! IS_.Open(port_.c_str(), baudrate_))
  {
    ROS_FATAL("inertialsense: Unable to open serial port \"%s\", at %d baud", port_.c_str(), baudrate_);
    exit(0);
  }
  else
  {
    // Print if Successful
    ROS_INFO("Connected to uINS %d on \"%s\", at %d baud", IS_.GetDeviceInfo().serialNumber, port_.c_str(), baudrate_);
  }
}

void InertialSenseROS::set_navigation_dt_ms()
{
  // Make sure the navigation rate is right, if it's not, then we need to change and reset it.
  int nav_dt_ms = IS_.GetFlashConfig().startupNavDtMs;
  if (nh_private_.getParam("navigation_dt_ms", nav_dt_ms))
  {
    if (nav_dt_ms != IS_.GetFlashConfig().startupNavDtMs)
    {
      uint32_t data = nav_dt_ms;
      IS_.SendData(DID_FLASH_CONFIG, (uint8_t*)(&data), sizeof(uint32_t), offsetof(nvm_flash_cfg_t, startupNavDtMs));
      ROS_INFO("navigation rate change from %dms to %dms, resetting uINS to make change", IS_.GetFlashConfig().startupNavDtMs, nav_dt_ms);
      sleep(3);
      reset_device();
    }
  }
}

void InertialSenseROS::configure_parameters()
{
  set_vector_flash_config<float>("INS_rpy", 3, offsetof(nvm_flash_cfg_t, insRotation));
  set_vector_flash_config<float>("INS_xyz", 3, offsetof(nvm_flash_cfg_t, insOffset));
  set_vector_flash_config<float>("GPS_ant1_xyz", 3, offsetof(nvm_flash_cfg_t, gps1AntOffset));
  set_vector_flash_config<float>("GPS_ant2_xyz", 3, offsetof(nvm_flash_cfg_t, gps2AntOffset));
  set_vector_flash_config<double>("GPS_ref_lla", 3, offsetof(nvm_flash_cfg_t, refLla));

  set_flash_config<float>("inclination", offsetof(nvm_flash_cfg_t, magInclination), 1.14878541071f);
  set_flash_config<float>("declination", offsetof(nvm_flash_cfg_t, magDeclination), 0.20007290992f);
  set_flash_config<int>("dynamic_model", offsetof(nvm_flash_cfg_t, insDynModel), 8);
  set_flash_config<int>("ser1_baud_rate", offsetof(nvm_flash_cfg_t, ser1BaudRate), 921600);
}


void InertialSenseROS::configure_rtk()
{
  bool RTK_rover, RTK_base, dual_GNSS;
  nh_private_.param<bool>("RTK_rover", RTK_rover, false);
  nh_private_.param<bool>("RTK_base", RTK_base, false);
  nh_private_.param<bool>("dual_GNSS", dual_GNSS, false);
  std::string RTK_server_IP, RTK_correction_type;
  int RTK_server_port;
  nh_private_.param<std::string>("RTK_server_IP", RTK_server_IP, "127.0.0.1");
  nh_private_.param<int>("RTK_server_port", RTK_server_port, 7777);
  nh_private_.param<std::string>("RTK_correction_type", RTK_correction_type, "UBLOX");
  ROS_ERROR_COND(RTK_rover && RTK_base, "unable to configure uINS to be both RTK rover and base - default to rover");
  ROS_ERROR_COND(RTK_rover && dual_GNSS, "unable to configure uINS to be both RTK rover as dual GNSS - default to dual GNSS");

  uint32_t RTKCfgBits = 0;
  if (dual_GNSS)
  {
    RTK_rover = false;
    ROS_INFO("InertialSense: Configured as dual GNSS (compassing)");
    RTK_state_ = DUAL_GNSS;
    RTKCfgBits |= RTK_CFG_BITS_COMPASSING;
    SET_CALLBACK(DID_GPS1_RTK_MISC, gps_rtk_misc_t, RTK_Misc_callback);
    SET_CALLBACK(DID_GPS1_RTK_REL, gps_rtk_rel_t, RTK_Rel_callback);
    RTK_.enabled = true;
    RTK_.pub = nh_.advertise<inertial_sense::RTKInfo>("RTK/info", 10);
    RTK_.pub2 = nh_.advertise<inertial_sense::RTKRel>("RTK/rel", 10);
  }

  if (RTK_rover)
  {
    RTK_base = false;
    std::string RTK_connection =  RTK_correction_type + ":" + RTK_server_IP + ":" + std::to_string(RTK_server_port);
    ROS_INFO("InertialSense: Configured as RTK Rover");
    RTK_state_ = RTK_ROVER;
    RTKCfgBits |= RTK_CFG_BITS_GPS1_RTK_ROVER;

    if (IS_.OpenServerConnection(RTK_connection))
      ROS_INFO_STREAM("Successfully connected to " << RTK_connection << " RTK server");
    else
      ROS_ERROR_STREAM("Failed to connect to base server at " << RTK_connection);

    SET_CALLBACK(DID_GPS1_RTK_MISC, gps_rtk_misc_t, RTK_Misc_callback);
    SET_CALLBACK(DID_GPS1_RTK_REL, gps_rtk_rel_t, RTK_Rel_callback);
    RTK_.enabled = true;
    RTK_.pub = nh_.advertise<inertial_sense::RTKInfo>("RTK/info", 10);
    RTK_.pub2 = nh_.advertise<inertial_sense::RTKRel>("RTK/rel", 10);
  }

  else if (RTK_base)
  {
    std::string RTK_connection =  RTK_server_IP + ":" + std::to_string(RTK_server_port);
    RTK_.enabled = true;
    ROS_INFO("InertialSense: Configured as RTK Base");
    RTK_state_ = RTK_BASE;
    RTKCfgBits |= RTK_CFG_BITS_BASE_OUTPUT_GPS1_UBLOX_SER0;

    if (IS_.CreateHost(RTK_connection))
    {
      ROS_INFO_STREAM("Successfully created " << RTK_connection << " as RTK server");
      initialized_ = true;
      return;
    }
    else
      ROS_ERROR_STREAM("Failed to create base server at " << RTK_connection);
  }
  IS_.SendData(DID_FLASH_CONFIG, reinterpret_cast<uint8_t*>(&RTKCfgBits), sizeof(RTKCfgBits), offsetof(nvm_flash_cfg_t, RTKCfgBits));
}

template <typename T>
void InertialSenseROS::set_vector_flash_config(std::string param_name, uint32_t size, uint32_t offset){
  std::vector<double> tmp(size,0);
  T v[size];
  if (nh_private_.hasParam(param_name))
    nh_private_.getParam(param_name, tmp);
  for (int i = 0; i < size; i++)
  {
    v[i] = tmp[i];
  }
  
  IS_.SendData(DID_FLASH_CONFIG, reinterpret_cast<uint8_t*>(&v), sizeof(v), offset);
  IS_.GetFlashConfig() = IS_.GetFlashConfig();
}

template <typename T>
void InertialSenseROS::set_flash_config(std::string param_name, uint32_t offset, T def)
{
  T tmp;
  nh_private_.param<T>(param_name, tmp, def);
  IS_.SendData(DID_FLASH_CONFIG, reinterpret_cast<uint8_t*>(&tmp), sizeof(T), offset);
}

void InertialSenseROS::INS1_callback(const ins_1_t * const msg)
{
  odom_msg.header.frame_id = frame_id_;

  odom_msg.pose.pose.position.x = msg->ned[0];
  odom_msg.pose.pose.position.y = msg->ned[1];
  odom_msg.pose.pose.position.z = msg->ned[2];
}

//void InertialSenseROS::INS_variance_callback(const inl2_variance_t * const msg)
//{
//  // We have to convert NED velocity covariance into body-fixed
//  tf::Matrix3x3 cov_vel_NED;
//  cov_vel_NED.setValue(msg->PvelNED[0], 0, 0, 0, msg->PvelNED[1], 0, 0, 0, msg->PvelNED[2]);
//  tf::Quaternion att;
//  tf::quaternionMsgToTF(odom_msg.pose.pose.orientation, att);
//  tf::Matrix3x3 R_NED_B(att);
//  tf::Matrix3x3 cov_vel_B = R_NED_B.transposeTimes(cov_vel_NED * R_NED_B);

//  // Populate Covariance Matrix
//  for (int i = 0; i < 3; i++)
//  {
//    // Position and velocity covariance is only valid if in NAV mode (with GPS)
//    if (insStatus_ & INS_STATUS_NAV_MODE)
//    {
//      odom_msg.pose.covariance[7*i] = msg->PxyzNED[i];
//      for (int j = 0; j < 3; j++)
//        odom_msg.twist.covariance[6*i+j] = cov_vel_B[i][j];
//    }
//    else
//    {
//      odom_msg.pose.covariance[7*i] = 0;
//      odom_msg.twist.covariance[7*i] = 0;
//    }
//    odom_msg.pose.covariance[7*(i+3)] = msg->PattNED[i];
//    odom_msg.twist.covariance[7*(i+3)] = msg->PWBias[i];
//  }
//}


void InertialSenseROS::INS2_callback(const ins_2_t * const msg)
{
  odom_msg.header.stamp = ros_time_from_week_and_tow(msg->week, msg->timeOfWeek);
  odom_msg.header.frame_id = frame_id_;

  odom_msg.pose.pose.orientation.w = msg->qn2b[0];
  odom_msg.pose.pose.orientation.x = msg->qn2b[1];
  odom_msg.pose.pose.orientation.y = msg->qn2b[2];
  odom_msg.pose.pose.orientation.z = msg->qn2b[3];

  odom_msg.twist.twist.linear.x = msg->uvw[0];
  odom_msg.twist.twist.linear.y = msg->uvw[1];
  odom_msg.twist.twist.linear.z = msg->uvw[2];

  lla_[0] = msg->lla[0];
  lla_[1] = msg->lla[1];
  lla_[2] = msg->lla[2];

  odom_msg.twist.twist.angular.x = imu1_msg.angular_velocity.x;
  odom_msg.twist.twist.angular.y = imu1_msg.angular_velocity.y;
  odom_msg.twist.twist.angular.z = imu1_msg.angular_velocity.z;
  if (INS_.enabled)
    INS_.pub.publish(odom_msg);
}


void InertialSenseROS::IMU_callback(const dual_imu_t* const msg)
{
  imu1_msg.header.stamp = imu2_msg.header.stamp = ros_time_from_start_time(msg->time);
  imu1_msg.header.frame_id = imu2_msg.header.frame_id = frame_id_;

  imu1_msg.angular_velocity.x = msg->I[0].pqr[0];
  imu1_msg.angular_velocity.y = msg->I[0].pqr[1];
  imu1_msg.angular_velocity.z = msg->I[0].pqr[2];
  imu1_msg.linear_acceleration.x = msg->I[0].acc[0];
  imu1_msg.linear_acceleration.y = msg->I[0].acc[1];
  imu1_msg.linear_acceleration.z = msg->I[0].acc[2];

  //  imu2_msg.angular_velocity.x = msg->I[1].pqr[0];
  //  imu2_msg.angular_velocity.y = msg->I[1].pqr[1];
  //  imu2_msg.angular_velocity.z = msg->I[1].pqr[2];
  //  imu2_msg.linear_acceleration.x = msg->I[1].acc[0];
  //  imu2_msg.linear_acceleration.y = msg->I[1].acc[1];
  //  imu2_msg.linear_acceleration.z = msg->I[1].acc[2];

  if (IMU_.enabled)
  {
    IMU_.pub.publish(imu1_msg);
    //    IMU_.pub2.publish(imu2_msg);
  }
}


void InertialSenseROS::GPS_pos_callback(const gps_pos_t * const msg)
{
  GPS_week_ = msg->week;
  GPS_towOffset_ = msg->towOffset;
  if (GPS_.enabled)
  {
    gps_msg.header.stamp = ros_time_from_week_and_tow(msg->week, msg->timeOfWeekMs/1e3);
    gps_msg.fix_type = msg->status & GPS_STATUS_FIX_MASK;
    gps_msg.header.frame_id =frame_id_;
    gps_msg.num_sat = (uint8_t)(msg->status & GPS_STATUS_NUM_SATS_USED_MASK);
    gps_msg.cno = msg->cnoMean;
    gps_msg.latitude = msg->lla[0];
    gps_msg.longitude = msg->lla[1];
    gps_msg.altitude = msg->lla[2];
    gps_msg.posEcef.x = msg->ecef[0];
    gps_msg.posEcef.y = msg->ecef[1];
    gps_msg.posEcef.z = msg->ecef[2];
    gps_msg.hMSL = msg->hMSL;
    gps_msg.hAcc = msg->hAcc;
    gps_msg.vAcc = msg->vAcc;
    gps_msg.pDop = msg->pDop;
    publishGPS();
  }
}

void InertialSenseROS::GPS_vel_callback(const gps_vel_t * const msg)
{
	if (GPS_.enabled)
	{
		gps_velEcef.header.stamp = ros_time_from_week_and_tow(GPS_week_, msg->timeOfWeekMs/1e3);
		gps_velEcef.vector.x = msg->velEcef[0];
		gps_velEcef.vector.y = msg->velEcef[1];
		gps_velEcef.vector.z = msg->velEcef[2];
		publishGPS();
	}
}

void InertialSenseROS::publishGPS()
{
	if (gps_velEcef.header.stamp == gps_msg.header.stamp)
	{
		gps_msg.velEcef = gps_velEcef.vector;
		GPS_.pub.publish(gps_msg);
	}
}

void InertialSenseROS::update()
{
	IS_.Update();
}

void InertialSenseROS::strobe_in_time_callback(const strobe_in_time_t * const msg)
{
  // create the subscriber if it doesn't exist
  if (strobe_pub_.getTopic().empty())
    strobe_pub_ = nh_.advertise<std_msgs::Header>("strobe_time", 1);
  
  std_msgs::Header strobe_msg;
  strobe_msg.stamp = ros_time_from_week_and_tow(msg->week, msg->timeOfWeekMs * 1e-3);
  strobe_pub_.publish(strobe_msg);
}


void InertialSenseROS::GPS_info_callback(const gps_sat_t* const msg)
{
  gps_info_msg.header.stamp =ros_time_from_tow(msg->timeOfWeekMs/1e3);
  gps_info_msg.header.frame_id = frame_id_;
  gps_info_msg.num_sats = msg->numSats;
  for (int i = 0; i < 50; i++)
  {
    gps_info_msg.sattelite_info[i].sat_id = msg->sat[i].svId;
    gps_info_msg.sattelite_info[i].cno = msg->sat[i].cno;
  }
  GPS_info_.pub.publish(gps_info_msg);
}


void InertialSenseROS::mag_callback(const magnetometer_t* const msg)
{
  sensor_msgs::MagneticField mag_msg;
  mag_msg.header.stamp = ros_time_from_start_time(msg->time);
  mag_msg.header.frame_id = frame_id_;
  mag_msg.magnetic_field.x = msg->mag[0];
  mag_msg.magnetic_field.y = msg->mag[1];
  mag_msg.magnetic_field.z = msg->mag[2];

  mag_.pub.publish(mag_msg);
}

void InertialSenseROS::baro_callback(const barometer_t * const msg)
{
  sensor_msgs::FluidPressure baro_msg;
  baro_msg.header.stamp = ros_time_from_start_time(msg->time);
  baro_msg.header.frame_id = frame_id_;
  baro_msg.fluid_pressure = msg->bar;

  baro_.pub.publish(baro_msg);
}

void InertialSenseROS::preint_IMU_callback(const preintegrated_imu_t * const msg)
{
  inertial_sense::PreIntIMU preintIMU_msg;
  preintIMU_msg.header.stamp = ros_time_from_start_time(msg->time);
  preintIMU_msg.header.frame_id = frame_id_;
  preintIMU_msg.dtheta.x = msg->theta1[0];
  preintIMU_msg.dtheta.y = msg->theta1[1];
  preintIMU_msg.dtheta.z = msg->theta1[2];

  preintIMU_msg.dvel.x = msg->vel1[0];
  preintIMU_msg.dvel.y = msg->vel1[1];
  preintIMU_msg.dvel.z = msg->vel1[2];

  preintIMU_msg.dt = msg->dt;

  dt_vel_.pub.publish(preintIMU_msg);
}

void InertialSenseROS::RTK_Misc_callback(const gps_rtk_misc_t* const msg)
{
  if (RTK_.enabled)
  {
    inertial_sense::RTKInfo rtk_info;
    rtk_info.header.stamp = ros_time_from_week_and_tow(GPS_week_, msg->timeOfWeekMs/1000.0);
    rtk_info.baseAntcount = msg->baseAntennaCount;
    rtk_info.baseEph = msg->baseBeidouEphemerisCount + msg->baseGalileoEphemerisCount + msg->baseGlonassEphemerisCount
                       + msg->baseGpsEphemerisCount;
    rtk_info.baseObs = msg->baseBeidouObservationCount + msg->baseGalileoObservationCount + msg->baseGlonassObservationCount
                       + msg->baseGpsObservationCount;
    rtk_info.BaseLLA[0] = msg->baseLla[0];
    rtk_info.BaseLLA[1] = msg->baseLla[1];
    rtk_info.BaseLLA[2] = msg->baseLla[2];

    rtk_info.roverEph = msg->roverBeidouEphemerisCount + msg->roverGalileoEphemerisCount + msg->roverGlonassEphemerisCount
                        + msg->roverGpsEphemerisCount;
    rtk_info.roverObs = msg->roverBeidouObservationCount + msg->roverGalileoObservationCount + msg->roverGlonassObservationCount
                        + msg->roverGpsObservationCount;
    rtk_info.cycle_slip_count = msg->cycleSlipCount;
    RTK_.pub.publish(rtk_info);
  }
}


void InertialSenseROS::RTK_Rel_callback(const gps_rtk_rel_t* const msg)
{
  if (RTK_.enabled)
  {
    inertial_sense::RTKRel rtk_rel;
    rtk_rel.header.stamp = ros_time_from_week_and_tow(GPS_week_, msg->timeOfWeekMs/1000.0);
    rtk_rel.differential_age = msg->differentialAge;
    rtk_rel.ar_ratio = msg->arRatio;
    rtk_rel.vector_to_base.x = msg->vectorToBase[0];
    rtk_rel.vector_to_base.y = msg->vectorToBase[1];
    rtk_rel.vector_to_base.z = msg->vectorToBase[2];
    rtk_rel.distance_to_base = msg->distanceToBase;
    rtk_rel.heading_to_base = msg->headingToBase;
    RTK_.pub2.publish(rtk_rel);
  }
}

void InertialSenseROS::GPS_raw_callback(const gps_raw_t * const msg)
{
  switch(msg->dataType)
  {
  case raw_data_type_observation:
    for (int i = 0; i < msg->obsCount; i++)
      GPS_obs_callback((obsd_t*)&msg->data.obs[i]);
    break;

  case raw_data_type_ephemeris:
    GPS_eph_callback((eph_t*)&msg->data.eph);
    break;

  case raw_data_type_glonass_ephemeris:
    GPS_geph_callback((geph_t*)&msg->data.gloEph);
    break;

  default:
    break;
  }
}

void InertialSenseROS::GPS_obs_callback(const obsd_t * const msg)
{
  inertial_sense::GNSSObservation obs;
  obs.time.time = msg->time.time;
  obs.time.sec = msg->time.sec;
  obs.sat = msg->sat;
  obs.rcv = msg->rcv;
  obs.SNR = msg->SNR[0];
  obs.LLI = msg->LLI[0];
  obs.code = msg->code[0];
  obs.qualL = msg->qualL[0];
  obs.qualP = msg->qualP[0];
  obs.L = msg->L[0];
  obs.P = msg->P[0];
  obs.D = msg->D[0];
  GPS_obs_.pub.publish(obs);
}

void InertialSenseROS::GPS_eph_callback(const eph_t * const msg)
{
  inertial_sense::GNSSEphemeris eph;
  eph.sat = msg->sat;
  eph.iode = msg->iode;
  eph.iodc = msg->iodc;
  eph.sva = msg->sva;
  eph.svh = msg->svh;
  eph.week = msg->week;
  eph.code = msg->code;
  eph.flag = msg->flag;
  eph.toe.time = msg->toe.time;
  eph.toc.time = msg->toc.time;
  eph.ttr.time = msg->ttr.time;
  eph.toe.sec = msg->toe.sec;
  eph.toc.sec = msg->toc.sec;
  eph.ttr.sec = msg->ttr.sec;
  eph.A = msg->A;
  eph.e = msg->e;
  eph.i0 = msg->i0;
  eph.OMG0 = msg->OMG0;
  eph.omg = msg->omg;
  eph.M0 = msg->M0;
  eph.deln = msg->deln;
  eph.OMGd = msg->OMGd;
  eph.idot = msg->idot;
  eph.crc = msg->crc;
  eph.crs = msg->crs;
  eph.cuc = msg->cuc;
  eph.cus = msg->cus;
  eph.cic = msg->cic;
  eph.cis = msg->cis;
  eph.toes = msg->toes;
  eph.fit = msg->fit;
  eph.f0 = msg->f0;
  eph.f1 = msg->f1;
  eph.f2 = msg->f2;
  eph.tgd[0] = msg->tgd[0];
  eph.tgd[1] = msg->tgd[1];
  eph.tgd[2] = msg->tgd[2];
  eph.tgd[3] = msg->tgd[3];
  eph.Adot = msg->Adot;
  eph.ndot = msg->ndot;
  GPS_eph_.pub.publish(eph);
}

void InertialSenseROS::GPS_geph_callback(const geph_t * const msg)
{
  inertial_sense::GlonassEphemeris geph;
  geph.sat = msg->sat;
  geph.iode = msg->iode;
  geph.frq = msg->frq;
  geph.svh = msg->svh;
  geph.sva = msg->sva;
  geph.age = msg->age;
  geph.toe.time = msg->toe.time;
  geph.tof.time = msg->tof.time;
  geph.toe.sec = msg->toe.sec;
  geph.tof.sec = msg->tof.sec;
  geph.pos[0] = msg->pos[0];
  geph.pos[1] = msg->pos[1];
  geph.pos[2] = msg->pos[2];
  geph.vel[0] = msg->vel[0];
  geph.vel[1] = msg->vel[1];
  geph.vel[2] = msg->vel[2];
  geph.acc[0] = msg->acc[0];
  geph.acc[1] = msg->acc[1];
  geph.acc[2] = msg->acc[2];
  geph.taun = msg->taun;
  geph.gamn = msg->gamn;
  geph.dtaun = msg->dtaun;
  GPS_eph_.pub2.publish(geph);
}

bool InertialSenseROS::set_current_position_as_refLLA(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  (void)req;
  res.success = true;
  IS_.SendData(DID_FLASH_CONFIG, reinterpret_cast<uint8_t*>(&lla_), sizeof(lla_), offsetof(nvm_flash_cfg_t, refLla));
}

bool InertialSenseROS::set_refLLA_to_value(inertial_sense::refLLAUpdate::Request &req, inertial_sense::refLLAUpdate::Response &res)
{
  res.success = true;
  IS_.SendData(DID_FLASH_CONFIG, reinterpret_cast<uint8_t*>(&req.lla), sizeof(req.lla), offsetof(nvm_flash_cfg_t, refLla));
  ROS_INFO("refLLA Updated");
}
bool InertialSenseROS::perform_mag_cal_srv_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  (void)req;
  res.success = true;
  uint32_t single_axis_command = 1;
  IS_.SendData(DID_MAG_CAL, reinterpret_cast<uint8_t*>(&single_axis_command), sizeof(uint32_t), offsetof(mag_cal_t, enMagRecal));
}

bool InertialSenseROS::perform_multi_mag_cal_srv_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  (void)req;
  res.success = true;
  uint32_t multi_axis_command = 0;
  IS_.SendData(DID_MAG_CAL, reinterpret_cast<uint8_t*>(&multi_axis_command), sizeof(uint32_t), offsetof(mag_cal_t, enMagRecal));
}

void InertialSenseROS::reset_device()
{
  // send reset command
  config_t reset_command;
  reset_command.system = 99;
  reset_command.invSystem = ~reset_command.system;
  IS_.SendData(DID_CONFIG, reinterpret_cast<uint8_t*>(&reset_command), sizeof(config_t), 0);
  sleep(1);
}

bool InertialSenseROS::update_firmware_srv_callback(inertial_sense::FirmwareUpdate::Request &req, inertial_sense::FirmwareUpdate::Response &res)
{
  IS_.Close();
  vector<InertialSense::bootloader_result_t> results = IS_.BootloadFile("*", req.filename, 921600);
  if (!results[0].error.empty())
  {
    res.success = false;
    res.message = results[0].error;
    return false;
  }
  IS_.Open(port_.c_str(), baudrate_);
  return true;
}

void InertialSenseROS::armadillo_pose_callback(const geometry_msgs::PoseWithCovarianceStamped &msg) //TODO: Should this be a pointer message? (PoseWithCovariancePtr)
{
    pos_measurement_t pos_msg;
    pos_msg.timeOfWeek = msg.header.stamp.sec;
    pos_msg.ned[0] = msg.pose.position.x;
    pos_msg.ned[1] = msg.pose.position.y;
    pos_msg.ned[2] = msg.pose.position.z;
    pos_msg.psi = msg.pose.orientation.y; //TODO: Calculate heading from quaternion
    pos_msg.accuracyCovUD[0] =msg.pose.covariance[0]; //TODO: Fix this.

  IS_.SendData(DID_POSITION_MEASUREMENT, reinterpret_cast<uint8_t*>(&pos_msg), sizeof(pos_msg), 0);
}
void InertialSenseROS::wheel_enc_callback(const sensor_msgs::JointStateConstPtr &msg)
{
  wheel_encoder_t wheel_enc_msg;
  wheel_enc_msg.timeOfWeek = tow_from_ros_time(msg->header.stamp);
  wheel_enc_msg.status = 0;
  wheel_enc_msg.theta_l = msg->position[0];
  wheel_enc_msg.theta_r = msg->position[1];
  wheel_enc_msg.omega_l = msg->velocity[0];
  wheel_enc_msg.omega_r = msg->velocity[1];
#if 0
  ROS_INFO("WHEEL: %14.4f %8.3f %8.3f %8.1f %8.1f", 
    wheel_enc_msg.timeOfWeekMs,
    wheel_enc_msg.theta_l,
    wheel_enc_msg.theta_r,
    wheel_enc_msg.omega_l,
    wheel_enc_msg.omega_r);
#endif
  IS_.SendData(DID_WHEEL_ENCODER, reinterpret_cast<uint8_t*>(&wheel_enc_msg), sizeof(wheel_encoder_t), 0);
}

void InertialSenseROS::configure_wheel_encoders()
{
  wheel_config_t wheel_encoder_config;
  std::vector<double> q_i2l, t_i2l;
  nh_private_.getParam("q_wheel_enc", q_i2l);
  nh_private_.getParam("t_wheel_enc", t_i2l);
  nh_private_.getParam("diameter", wheel_encoder_config.diameter);
  nh_private_.getParam("distance", wheel_encoder_config.distance);
  IS_.SendData(DID_WHEEL_CONFIG, reinterpret_cast<uint8_t*>(&wheel_encoder_config), sizeof(wheel_config_t), 0);
}

ros::Time InertialSenseROS::ros_time_from_week_and_tow(const uint32_t week, const double timeOfWeek)
{
  ros::Time rostime(0, 0);
  //  If we have a GPS fix, then use it to set timestamp
  if (GPS_towOffset_)
  {
    uint64_t sec = UNIX_TO_GPS_OFFSET + floor(timeOfWeek) + week*7*24*3600;
    uint64_t nsec = (timeOfWeek - floor(timeOfWeek))*1e9;
    rostime = ros::Time(sec, nsec);
  }
  else
  {
    // Otherwise, estimate the uINS boot time and offset the messages
    if (!got_first_message_)
    {
      got_first_message_ = true;
      INS_local_offset_ = ros::Time::now().toSec() - timeOfWeek;
    }
    else // low-pass filter offset to account for drift
    {
      double y_offset = ros::Time::now().toSec() - timeOfWeek;
      INS_local_offset_ = 0.005 * y_offset + 0.995 * INS_local_offset_;
    }
    // Publish with ROS time
    rostime = ros::Time(INS_local_offset_ + timeOfWeek);
  }
  return rostime;
}

ros::Time InertialSenseROS::ros_time_from_start_time(const double time)
{
  ros::Time rostime(0, 0);
  
  //  If we have a GPS fix, then use it to set timestamp
  if (GPS_towOffset_ > 0.001)
  {
    uint64_t sec = UNIX_TO_GPS_OFFSET + floor(time + GPS_towOffset_) + GPS_week_*7*24*3600;
    uint64_t nsec = (time + GPS_towOffset_ - floor(time + GPS_towOffset_))*1e9;
    rostime = ros::Time(sec, nsec);
  }
  else
  {
    // Otherwise, estimate the uINS boot time and offset the messages
    if (!got_first_message_)
    {
      got_first_message_ = true;
      INS_local_offset_ = ros::Time::now().toSec() - time;
    }
    else // low-pass filter offset to account for drift
    {
      double y_offset = ros::Time::now().toSec() - time;
      INS_local_offset_ = 0.005 * y_offset + 0.995 * INS_local_offset_;
    }
    // Publish with ROS time
    rostime = ros::Time(INS_local_offset_ + time);
  }
  return rostime;
}

ros::Time InertialSenseROS::ros_time_from_tow(const double tow)
{
  return ros_time_from_week_and_tow(GPS_week_, tow);
}

double InertialSenseROS::tow_from_ros_time(const ros::Time &rt)
{
  return (rt.sec - UNIX_TO_GPS_OFFSET - GPS_week_*604800) + rt.nsec*1.0e-9;
}


int main(int argc, char**argv)
{
  ros::init(argc, argv, "inertial_sense_node");
  InertialSenseROS thing;
  while (ros::ok())
  {
    ros::spinOnce();
    thing.update();
  }
  return 0;
}
