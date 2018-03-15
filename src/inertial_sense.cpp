#include "inertial_sense.h"
#include <chrono>

#include <ros/console.h>

static void data_callback(InertialSense* i, p_data_t* data);

InertialSenseROS::InertialSenseROS() :
    nh_(), nh_private_("~"), IMU_offset_(0,0), GPS_to_week_offset_(0)
{
    nh_private_.param<std::string>("port", port_, "/dev/ttyUSB0");
    nh_private_.param<int>("baudrate", baudrate_, 3000000);
    nh_private_.param<std::string>("frame_id", frame_id_, "body");

    // Connect to the uINS
    if (!inertialSenseInterface_.Open(port_.c_str(), baudrate_))
    {
        ROS_ERROR("Unable to open uINS-2 on port %s at %d baud", port_.c_str(), baudrate_);
        return;
    }
    else
    {
        ROS_INFO_STREAM("InertialSense uINS-2 Connected on port " << port_.c_str() << " at " << baudrate_ << " baud");
    }
    inertialSenseInterface_.StopBroadcasts();	// Stop streaming any prior messages


    // Configure the uINS
    flash_cfg_ = inertialSenseInterface_.GetFlashConfig();

    // Change the flash based on parameters
    std::vector<double> INS_rpy(3, 0.0);
    std::vector<double> INS_xyz(3, 0.0);
    std::vector<double> GPS_ant_xyz(3, 0.0);
    std::vector<double> GPS_ref_lla(3, 0.0);

    if (nh_private_.hasParam("INS_rpy"))
        nh_private_.getParam("INS_rpy", INS_rpy);
    if (nh_private_.hasParam("INS_xyz"))
        nh_private_.getParam("INS_xyz", INS_xyz);
    if (nh_private_.hasParam("GPS_ant_xyz"))
        nh_private_.getParam("GPS_ant_xyz", GPS_ant_xyz);
    if (nh_private_.hasParam("GPS_ref_lla"))
        nh_private_.getParam("GPS_ref_lla", GPS_ref_lla);

    float mag_inclination, mag_declination ,mag_magnitude;
    nh_private_.param<float>("inclination", mag_inclination, 1.14878541071);
    nh_private_.param<float>("declination", mag_declination, 0.20007290992);
    nh_private_.param<float>("mag_magnitude", mag_magnitude, 1.0); // 51.6619nT <-- Not sure how this works

    int dynamic_model;
    nh_private_.param<int>("dynamic_model", dynamic_model, 8);

    flash_cfg_.insRotation[0] = INS_rpy[0];
    flash_cfg_.insRotation[1] = INS_rpy[1];
    flash_cfg_.insRotation[2] = INS_rpy[2];

    flash_cfg_.insOffset[0] = INS_xyz[0];
    flash_cfg_.insOffset[1] = INS_xyz[1];
    flash_cfg_.insOffset[2] = INS_xyz[2];

    flash_cfg_.gpsAntOffset[0] = GPS_ant_xyz[0];
    flash_cfg_.gpsAntOffset[1] = GPS_ant_xyz[1];
    flash_cfg_.gpsAntOffset[2] = GPS_ant_xyz[2];

    flash_cfg_.refLla[0] = GPS_ref_lla[0];
    flash_cfg_.refLla[1] = GPS_ref_lla[1];
    flash_cfg_.refLla[2] = GPS_ref_lla[2];

    flash_cfg_.magInclination = mag_inclination;
    flash_cfg_.magDeclination = mag_declination;
    flash_cfg_.magMagnitude = mag_magnitude;

    flash_cfg_.insDynModel = dynamic_model;

    inertialSenseInterface_.SetFlashConfig(flash_cfg_);


    // Set up the INS ROS stream
    nh_private_.param<bool>("sINS", INS_.stream_on, true);
    nh_private_.param<int>("sINS_rate", INS_.stream_rate, 100);
    if (INS_.stream_on)
    {
        INS_.pub = nh_.advertise<nav_msgs::Odometry>("ins", 1);
        inertialSenseInterface_.BroadcastBinaryData(DID_INS_1, (int)(1000/INS_.stream_rate), &data_callback);
        inertialSenseInterface_.BroadcastBinaryData(DID_INS_2, (int)(1000/INS_.stream_rate), &data_callback);
        inertialSenseInterface_.BroadcastBinaryData(DID_DUAL_IMU, (int)(1000/INS_.stream_rate), &data_callback);
    }

    // Set up the IMU ROS stream
    nh_private_.param<bool>("sIMU", IMU_.stream_on, true);
    nh_private_.param<int>("sIMU_rate", IMU_.stream_rate, 100);
    if (IMU_.stream_on)
    {
        IMU_.pub = nh_.advertise<sensor_msgs::Imu>("imu1", 1);
        IMU_.pub2 = nh_.advertise<sensor_msgs::Imu>("imu2", 1);
        uint32_t update_rate = 1000/IMU_.stream_rate;
        if (INS_.stream_on)
        {
            update_rate = (INS_.stream_rate > IMU_.stream_rate) ? 1000/IMU_.stream_rate : update_rate;
        }
        inertialSenseInterface_.BroadcastBinaryData(DID_DUAL_IMU, update_rate, &data_callback);
    }

    // Set up the GPS ROS stream
    nh_private_.param<bool>("sGPS", GPS_.stream_on, true);
    nh_private_.param<int>("sGPS_rate", GPS_.stream_rate, 10);
    if (GPS_.stream_on)
    {
        GPS_.pub = nh_.advertise<inertial_sense::GPS>("gps", 1);
        inertialSenseInterface_.BroadcastBinaryData(DID_GPS, 1000/GPS_.stream_rate, &data_callback);
    }

     // Set up the GPS info ROS stream
     nh_private_.param<bool>("sGPS_info", GPS_info_.stream_on, true);
     nh_private_.param<int>("sGPS_info_rate", GPS_info_.stream_rate, 10);
     if (GPS_info_.stream_on)
     {
         GPS_info_.pub = nh_.advertise<inertial_sense::GPSInfo>("gps/info", 1);
         inertialSenseInterface_.BroadcastBinaryData(DID_GPS_CNO, 1000/GPS_info_.stream_rate, &data_callback);
     }

    // Set up the magnetometer ROS stream
    nh_private_.param<bool>("smag", mag_.stream_on, true);
    nh_private_.param<int>("smag_rate", mag_.stream_rate, 100);
    if (mag_.stream_on)
    {
        mag_.pub = nh_.advertise<sensor_msgs::MagneticField>("mag1", 1);
        mag_.pub2 = nh_.advertise<sensor_msgs::MagneticField>("mag2", 1);
        inertialSenseInterface_.BroadcastBinaryData(DID_MAGNETOMETER_1, 1000/mag_.stream_rate, &data_callback);
        inertialSenseInterface_.BroadcastBinaryData(DID_MAGNETOMETER_2, 1000/mag_.stream_rate, &data_callback);
    }

    // Set up the barometer ROS stream
    nh_private_.param<bool>("sbaro", baro_.stream_on, true);
    nh_private_.param<int>("sbaro_rate", baro_.stream_rate, 100);
    if (baro_.stream_on)
    {
        baro_.pub = nh_.advertise<sensor_msgs::FluidPressure>("baro", 1);
        inertialSenseInterface_.BroadcastBinaryData(DID_BAROMETER, 1000/baro_.stream_rate, &data_callback);
    }

    // Set up the delta_theta_vel (coning and sculling integral) ROS stream
    nh_private_.param<bool>("sdelta_theta_vel", dt_vel_.stream_on, false);
    nh_private_.param<int>("sdelta_theta_vel_rate", dt_vel_.stream_rate, 100);
    if (dt_vel_.stream_on)
    {
        dt_vel_.pub = nh_.advertise<inertial_sense::DThetaVel>("delta_theta_vel", 1);
        inertialSenseInterface_.BroadcastBinaryData(DID_DELTA_THETA_VEL, 1000/dt_vel_.stream_rate, &data_callback);
    }

    // ask for device info every 2 seconds
    inertialSenseInterface_.BroadcastBinaryData(DID_DEV_INFO, 2000, &data_callback);
}

void InertialSenseROS::INS1_callback()
{
    odom_msg.header.frame_id = frame_id_;

    odom_msg.pose.pose.position.x = d_.ins1.ned[0];
    odom_msg.pose.pose.position.y = d_.ins1.ned[1];
    odom_msg.pose.pose.position.z = d_.ins1.ned[2];
}


void InertialSenseROS::INS2_callback()
{
    uint64_t seconds = GPS_UTC_OFFSET + d_.ins2.week*7*24*3600 + floor(d_.ins2.timeOfWeek);
    uint64_t nsec = (d_.ins2.timeOfWeek - floor(d_.ins2.timeOfWeek))*1e9;
    odom_msg.header.stamp = ros::Time(seconds, nsec);

    odom_msg.header.frame_id = frame_id_;

    odom_msg.pose.pose.orientation.w = d_.ins2.qn2b[0];
    odom_msg.pose.pose.orientation.x = d_.ins2.qn2b[1];
    odom_msg.pose.pose.orientation.y = d_.ins2.qn2b[2];
    odom_msg.pose.pose.orientation.z = d_.ins2.qn2b[3];

    odom_msg.twist.twist.linear.x = d_.ins2.uvw[0];
    odom_msg.twist.twist.linear.y = d_.ins2.uvw[1];
    odom_msg.twist.twist.linear.z = d_.ins2.uvw[2];

    odom_msg.twist.twist.angular.x = imu1_msg.angular_velocity.x;
    odom_msg.twist.twist.angular.y = imu1_msg.angular_velocity.y;
    odom_msg.twist.twist.angular.z = imu1_msg.angular_velocity.z;
    INS_.pub.publish(odom_msg);
}
void InertialSenseROS::IMU_callback()
{

    ros::Time imu_time(0, 0);
    //  If we have a GPS fix, then use it to timestamp IMU messages
    if (got_GPS_fix_)
    {
        uint64_t sec = GPS_UTC_OFFSET + GPS_week_seconds + floor(d_.imu.time + GPS_to_week_offset_);
        uint64_t nsec = (d_.imu.time + GPS_to_week_offset_ - floor(d_.imu.time + GPS_to_week_offset_))*1e9;
        imu_time = ros::Time(sec, nsec);
    }
    else
    {
        uint64_t sec = floor(d_.imu.time);
        uint64_t nsec = (d_.imu.time - sec)*1e9;
        imu_time = ros::Time(sec, nsec) + IMU_offset_;
    }


    // Calculate an offset so we can sync IMU messages to GPS until we have a GPS fix
    if (first_IMU_message_)
    {
        first_IMU_message_ = false;
        IMU_offset_ = ros::Time::now() - imu_time;
        imu_time += IMU_offset_;
    }
    imu1_msg.header.stamp = imu2_msg.header.stamp = imu_time;
    imu1_msg.header.frame_id = imu2_msg.header.frame_id = frame_id_;

    imu1_msg.angular_velocity.x = d_.dualImu.I[0].pqr[0];
    imu1_msg.angular_velocity.y = d_.dualImu.I[0].pqr[1];
    imu1_msg.angular_velocity.z = d_.dualImu.I[0].pqr[2];
    imu1_msg.linear_acceleration.x = d_.dualImu.I[0].acc[0];
    imu1_msg.linear_acceleration.y = d_.dualImu.I[0].acc[1];
    imu1_msg.linear_acceleration.z = d_.dualImu.I[0].acc[2];

    imu2_msg.angular_velocity.x = d_.dualImu.I[1].pqr[0];
    imu2_msg.angular_velocity.y = d_.dualImu.I[1].pqr[1];
    imu2_msg.angular_velocity.z = d_.dualImu.I[1].pqr[2];
    imu2_msg.linear_acceleration.x = d_.dualImu.I[1].acc[0];
    imu2_msg.linear_acceleration.y = d_.dualImu.I[1].acc[1];
    imu2_msg.linear_acceleration.z = d_.dualImu.I[1].acc[2];

    if (IMU_.stream_on)
    {
        IMU_.pub.publish(imu1_msg);
        IMU_.pub2.publish(imu2_msg);
    }
}


void InertialSenseROS::GPS_callback()
{
    uint64_t seconds = GPS_UTC_OFFSET + d_.gps.pos.week*7*24*3600 + floor(d_.gps.pos.timeOfWeekMs/1e3);
    uint64_t nsec = (d_.gps.pos.timeOfWeekMs/1e3 - floor(d_.gps.pos.timeOfWeekMs/1e3))*1e9;
    GPS_week_seconds = d_.gps.pos.week*7*24*3600;
    gps_msg.header.stamp = ros::Time(seconds, nsec);
    gps_msg.fix_type = d_.gps.pos.status & GPS_STATUS_FIX_TYPE_MASK;
    gps_msg.header.frame_id =frame_id_;
    gps_msg.num_sat = (uint8_t)(d_.gps.pos.status & GPS_STATUS_NUM_SATS_USED_MASK);
    gps_msg.cno = d_.gps.pos.cno;
    gps_msg.latitude = d_.gps.pos.lla[0];
    gps_msg.longitude = d_.gps.pos.lla[1];
    gps_msg.altitude = d_.gps.pos.lla[2];
    gps_msg.hMSL = d_.gps.pos.hMSL;
    gps_msg.hAcc = d_.gps.pos.hAcc;
    gps_msg.vAcc = d_.gps.pos.vAcc;
    gps_msg.pDop = d_.gps.pos.pDop;
    gps_msg.linear_velocity.x = d_.gps.vel.ned[0];
    gps_msg.linear_velocity.y = d_.gps.vel.ned[1];
    gps_msg.linear_velocity.z = d_.gps.vel.ned[2];
    gps_msg.ground_speed_2d = d_.gps.vel.s2D;
    gps_msg.ground_speed_3d = d_.gps.vel.s3D;
    gps_msg.course = d_.gps.vel.course;
    gps_msg.cAcc = d_.gps.vel.cAcc;
    gps_msg.messages_per_second = d_.gps.rxps;
    GPS_.pub.publish(gps_msg);

    if (!got_GPS_fix_)
    {
        if (d_.gps.pos.status & GPS_STATUS_FIX_TYPE_3D_FIX)
        {
            got_GPS_fix_ = true;
        }
        GPS_to_week_offset_ = d_.gps.towOffset;
    }
}

void InertialSenseROS::GPS_Info_callback()
{
    uint64_t seconds = GPS_UTC_OFFSET + GPS_week_seconds + floor(d_.gpsCNO.timeOfWeekMs/1e3);
    uint64_t nsec = (d_.gpsCNO.timeOfWeekMs - floor(d_.gpsCNO.timeOfWeekMs))*1e6;
    //  ROS_INFO("dsec = %d dnsec = %d", ros::Time::now().sec - seconds, ros::Time::now().nsec - nsec);
    gps_info_msg.header.stamp = ros::Time(seconds, nsec);
    gps_info_msg.header.frame_id = frame_id_;
    gps_info_msg.num_sats = d_.gpsCNO.numSats;
    for (int i = 0; i < 50; i++)
    {
        gps_info_msg.sattelite_info[i].sat_id = d_.gpsCNO.info[i].svId;
        gps_info_msg.sattelite_info[i].cno = d_.gpsCNO.info[i].cno;
    }
    GPS_info_.pub.publish(gps_info_msg);
}


void InertialSenseROS::mag_callback(int mag_number)
{
    sensor_msgs::MagneticField mag_msg;
    ros::Time mag_time(0, 0);
    //  If we have a GPS fix, then use it to timestamp mag messages
    if (got_GPS_fix_)
    {
        uint64_t sec = GPS_UTC_OFFSET + GPS_week_seconds + floor(d_.mag.time + GPS_to_week_offset_);
        uint64_t nsec = (d_.mag.time + GPS_to_week_offset_ - floor(d_.mag.time + GPS_to_week_offset_))*1e9;
        mag_time = ros::Time(sec, nsec);
    }
    else
    {
        uint64_t sec = floor(d_.mag.time);
        uint64_t nsec = (d_.mag.time - sec)*1e9;
        mag_time = ros::Time(sec, nsec) + IMU_offset_;
    }


    // Calculate an offset so we can sync mag messages to GPS until we have a GPS fix
    if (first_IMU_message_)
    {
        first_IMU_message_ = false;
        IMU_offset_ = ros::Time::now() - mag_time;
        mag_time += IMU_offset_;
    }

    mag_msg.header.stamp = mag_time;
    mag_msg.header.frame_id = frame_id_;
    mag_msg.magnetic_field.x = d_.mag.mag[0];
    mag_msg.magnetic_field.y = d_.mag.mag[1];
    mag_msg.magnetic_field.z = d_.mag.mag[2];

    if(mag_number == 1)
    {
        mag_.pub.publish(mag_msg);
    }
    else
    {
        mag_.pub2.publish(mag_msg);
    }
}

void InertialSenseROS::baro_callback()
{
    sensor_msgs::FluidPressure baro_msg;
    ros::Time baro_time(0, 0);
    //  If we have a GPS fix, then use it to timestamp baro messages
    if (got_GPS_fix_)
    {
        uint64_t sec = GPS_UTC_OFFSET + GPS_week_seconds + floor(d_.baro.time + GPS_to_week_offset_);
        uint64_t nsec = (d_.baro.time + GPS_to_week_offset_ - floor(d_.baro.time + GPS_to_week_offset_))*1e9;
        baro_time = ros::Time(sec, nsec);
    }
    else
    {
        uint64_t sec = floor(d_.baro.time);
        uint64_t nsec = (d_.baro.time - sec)*1e9;
        baro_time = ros::Time(sec, nsec) + IMU_offset_;
    }
    // Calculate an offset so we can sync mag messages to GPS until we have a GPS fix
    if (first_IMU_message_)
    {
        first_IMU_message_ = false;
        IMU_offset_ = ros::Time::now() - baro_time;
        baro_time += IMU_offset_;
    }

    baro_msg.header.stamp = baro_time;
    baro_msg.header.frame_id = frame_id_;
    baro_msg.fluid_pressure = d_.baro.bar;

    baro_.pub.publish(baro_msg);
}

void InertialSenseROS::dtheta_vel_callback()
{
    inertial_sense::DThetaVel dthetavel_msg;

    ros::Time imu_time(0, 0);
    //  If we have a GPS fix, then use it to timestamp imu messages
    if (got_GPS_fix_)
    {
        uint64_t sec = GPS_UTC_OFFSET + GPS_week_seconds + floor(d_.imu.time + GPS_to_week_offset_);
        uint64_t nsec = (d_.imu.time + GPS_to_week_offset_ - floor(d_.imu.time + GPS_to_week_offset_))*1e9;
        imu_time = ros::Time(sec, nsec);
    }
    else
    {
        uint64_t sec = floor(d_.imu.time);
        uint64_t nsec = (d_.imu.time - sec)*1e9;
        imu_time = ros::Time(sec, nsec) + IMU_offset_;
    }
    // Calculate an offset so we can sync mag messages to GPS until we have a GPS fix
    if (first_IMU_message_)
    {
        first_IMU_message_ = false;
        IMU_offset_ = ros::Time::now() - imu_time;
        imu_time += IMU_offset_;
    }

    dthetavel_msg.header.stamp = imu_time;
    dthetavel_msg.header.frame_id = frame_id_;
    dthetavel_msg.theta.x = d_.dThetaVel.theta[0];
    dthetavel_msg.theta.y = d_.dThetaVel.theta[1];
    dthetavel_msg.theta.z = d_.dThetaVel.theta[2];

    dthetavel_msg.vel.x = d_.dThetaVel.uvw[0];
    dthetavel_msg.vel.y = d_.dThetaVel.uvw[1];
    dthetavel_msg.vel.z = d_.dThetaVel.uvw[2];

    dthetavel_msg.dt = d_.dThetaVel.dt;

    dt_vel_.pub.publish(dthetavel_msg);
}

void InertialSenseROS::callback(p_data_t* data)
{
    copyDataPToStructP(&d_, data, sizeof(uDatasets));
    switch(data->hdr.id)
    {
    case DID_INS_1:
        INS1_callback();
        break;
    case DID_INS_2:
        INS2_callback();
        break;

    case DID_DUAL_IMU:
        IMU_callback();
        break;

    case DID_GPS:
        GPS_callback();
        break;

    case DID_GPS_CNO:
        GPS_Info_callback();
        break;

    case DID_MAGNETOMETER_1:
        mag_callback(1);
        break;
    case DID_MAGNETOMETER_2:
        mag_callback(2);
        break;

    case DID_BAROMETER:
        baro_callback();
        break;

    case DID_DELTA_THETA_VEL:
        dtheta_vel_callback();
        break;
    }

}

void InertialSenseROS::update()
{
    inertialSenseInterface_.Update();
}




// I really, really hate this global pointer
InertialSenseROS* ISROSPTr;
static void data_callback(InertialSense* i, p_data_t* data)
{
    ISROSPTr->callback(data);
}

int main(int argc, char**argv)
{
    ros::init(argc, argv, "inertial_sense_node");
    InertialSenseROS thing;
    ISROSPTr = &thing;

    while(ros::ok())
    {
        thing.update();
        ros::spinOnce();
    }

    return 0;
}
