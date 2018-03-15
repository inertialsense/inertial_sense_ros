# inertial_sense

A ROS wrapper for the InertialSense uINS2 GPS-INS sensor

## Functionality 
- INS full odometry streaming
- Dual IMU streaming
- Full GPS data streaming
- Magnetomter streaming
- Barometer streaming
- Coning and Sculling integral streaming
- Timestamping uses GPS timestamps when available and syncs sensor messages with ROS time if unavailable.
- Flash configuration via parameters
- Changing dynamic model via parameter

## ToDo
- Adjusting rate of streaming and flash configuration on-the-fly with dynamic reconfigure

## Installation
This is a ROS package, with the InertialSenseSDK as a submodule, so just create a catkin workspace, clone this into the `src` folder, pull down the submodule and build

``` bash
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
git clone https://github.com/inertialsense/inertial_sense_ros.git
cd inertial_sense
git submodule update --init --recursive
cd ../..
catkin_make
```

## Running the Node

```bash
rosrun inertial_sense inertial_sense_node
```

For changing parameter values and topic remapping from the command line using `rosrun` refer to the [Remapping Arguments](http://wiki.ros.org/Remapping%20Arguments) page. For setting vector parameters, use the following syntax:

``` bash
rosparam set /inertial_sense_node/GPS_ref_lla "[40.25, -111.67, 1556.59]"
rosrun inertial_sense inertial_sense_node
```

For setting parameters and topic remappings from a launch file, refer to the [Roslaunch for Larger Projects](http://wiki.ros.org/roslaunch/Tutorials/Roslaunch%20tips%20for%20larger%20projects) page.


## Parameters
* `~port` (string, default: "/dev/ttyUSB0")
    - Serial port to connect to
* `~baudrate` (int, default: 3000000)
    - baudrate of serial communication 
* `~frame_id` (string, default "body") 
   - frame id of all measurements 
* `~sINS` (bool, default: true)
   - Whether to stream the full 12-DOF odometry measurement
* `~sINS_rate` (int, default: 100)
   - The rate of odometry measurement streaming (Hz)
* `~sIMU`(bool, default: true)
   - Whether to stream IMU measurements
* `~sIMU_rate`(int, default: 100) 
   - The rate of IMU measurement streaming (Hz)
* `~sGPS`(bool, default: true) 
   - If true, the node will stream GPS measurements
* `~sGPS_rate`(int, default: 10) 
   - The rate of GPS message streaming (Hz)
* `~sGPS_info`(bool, default: true) 
   - If true, the node will stream GPS measurements
* `~sGPS_info_rate`(int, default: 10) 
    - The rate of GPS message streaming (Hz)
* `~sbaro` (bool default: true)
    - If true, the node will stream barometer measurements
* `~sbaro_rate` (int, default: 100)
    - The rate of barometer streaming (Hz)
* `~smag` (bool, default: true)
    - If true, the node will stream both magnetometer measurements
* `~smag_rate` (int, default: 100)
    - The rate of magnetometer streaming (Hz)
* `~sdelta_theta_vel` (bool, default: false)
    - If true, the node will stream coning and sculling integral versions of IMU measurements
* `~sdelta_theta_vel_rate` (int, default: 100)
    - The rate of coning and sculling integral message streaming

* `~INS_rpy` (vector(3), default: {0, 0, 0})
    - The roll, pitch, yaw rotation from the INS frame to the output frame
* `~INS_xyz` (vector(3), default: {0, 0, 0})
    - The NED translation vector between the INS frame and the output frame (wrt output frame)
* `~GPS_ang_xyz` (vector(3), default: {0, 0, 0})
    - The NED translation vector between the INS frame and the GPS antenna (wrt INS frame)
* `~GPS_ref_lla` (vector(3), default: {0, 0, 0})
    - The Reference longitude, latitude and altitude for NED calculation in degrees, degrees and meters
* `~inclination` (float, default: 1.14878541071)
    - The inclination of earth's magnetic field (radians)
* `~declination` (float, default: 0.20007290992)
    - The declination of earth's magnetic field (radians)
* `mag_magnitude` (float, default: 1.0)
    - Earth magnetic field (magnetic north) magnitude (nominally 1)
* `dynamic_model` (int, default: 8)
    - Dynamic model used in internal filter of uINS.
       - 0 = portable
       - 2 = stationary
       - 3 = pedestrian
       - 4 = automotive
       - 5 = sea
       - 6 = airborne 1G
       - 7 = airborne 2G
       - 8 = airborne 4G
       - 9 = wrist

## Topics
- `imu1/`(sensor_msgs/Imu) 
    - Imu measurements from IMU1 (NED frame)
- `imu2/`(sensor_msgs/Imu) 
    - Imu measurements from IMU2 (NED frame)
- `ins/`(nav_msgs/Odometry) 
    - full 12-DOF measurements from onboard estimator (NED frame)
- `gps/`(inertial_sense/GPS) 
    - full GPS measurement from onbaord GPS
- `gps/info`(inertial_sense/GPSInfo)
    - sattelite information and carrier noise ratio array for each sattelite
- `mag1` (sensor_msgs/MagneticField)
    + magnetic field measurement from magnetometer 1
- `mag2` (sensor_msgs/MagneticField)
    + magnetic field measurement from magnetometer 2
- `baro` (sensor_msgs/FluidPressure)
    + barometer measurements in kPa
- `delta_theta_vel` (inertial_sense/DThetaVel)
    + coning and sculling integral representation of IMU measurements
