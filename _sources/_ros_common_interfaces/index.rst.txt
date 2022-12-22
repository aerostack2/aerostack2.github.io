ROS Common interfaces
=====================

Aerostack2 uses common ROS2 interfaces like **topics**, **services** and **actions**.

This page describes common ROS2 interfaces for inter-process communication used by Aerostack2. The goal of this set of interfaces is to facilitate the interoperability of the ROS2 nodes in Aerostack2.

Using AS2 interfaces
####################

AS2 interfaces names **should not** be typed manually. Names are stored in:

.. code-block:: cpp

 #include "as2_core/names/topics.hpp"
 #include "as2_core/names/services.hpp"
 #include "as2_core/names/actions.hpp"

You can access to a interface name via `as2_names`, e.g. `as2_names::topics::platform::info`.

* **Topics**

Platform represents the robot.

.. list-table:: PLATFORM (TOPICS)
   :widths: 50 50 50 50
   :header-rows: 1

   * - Topic name
     - ROS message
     - Quality of Service
     - Description
   * - platform/info
     - as2_msgs/PlatformInfo
     - DefaultQoS
     - | Information about the robot: connection_state,
       | arming_state, offboard_state, 
       | fly status and control_mode.


Sensor measurements are values corresponding to direct measurements recorded by sensors.

.. list-table:: SENSOR MEASUREMENTS
   :widths: 50 50 50 50
   :header-rows: 1

   * - Topic name
     - ROS message
     - Quality of Service
     - Description
   * - sensor_measurement/camera
     - sensor_msgs/Image
     - SensorDataQoS
     - | Raw image data received from
       | camera (a general camera).
   * - sensor_measurement/camera/front
     - sensor_msgs/Image
     - SensorDataQoS
     - | Raw image data received from 
       | front camera.
   * - sensor_measurement/camera/bottom
     - sensor_msgs/Image
     - SensorDataQoS
     - | Raw image data received from
       | the bottom camera.
   * - sensor_measurement/gps
     - sensor_msgs/NavSatFix
     - SensorDataQoS
     - | GPS (Global Positioning 
       | System) coordinates
   * - sensor_measurement/imu
     - sensor_msgs/Imu
     - SensorDataQoS
     - | Inertial Measurement Unit 
       | data

These values correspond to the robot localization in the environment together with kinematic values (e.g., speed) as they are believed by the robot. These values may be obtained, for example, by fusing sensor measurements with the help of extended Kalman filters (EKF).

.. list-table:: SELF LOCALIZATION
   :widths: 50 50 50 50
   :header-rows: 1

   * - Topic name
     - ROS message
     - Quality of Service
     - Description
   * - self_localization/odom
     - nav_msgs/Odometry
     - SensorDataQoS
     - | Estimated odometry (pose and twist).

Represents the Control Manager.

.. list-table:: CONTROLLER
   :widths: 50 50 50 50
   :header-rows: 1

   * - Topic name
     - ROS message
     - Quality of Service
     - Description
   * - controller/info
     - as2_msgs/ControllerInfo
     - DefaultQoS
     - | Information about the controller state.

These messages are motion values to be considered as goals by controllers.

.. list-table:: MOTION REFERENCE
   :widths: 50 50 50 50
   :header-rows: 1

   * - Topic name
     - ROS message
     - Quality of Service
     - Description
   * - motion_reference/pose
     - geometry_msgs/PoseStamped
     - SensorDataQoS
     - | Pose reference for the controller. 
       | The following values from the message 
       | are used <x, y, z, yaw> (the values 
       | for <pitch, roll> are discarded)
   * - motion_reference/speed
     - geometry_msgs/TwistStamped
     - SensorDataQoS
     - | Speed reference for the controller. 
       | The following values from the message 
       | are used <d_x, d_y, d_z, d_yaw> (the 
       | values for <d_pitch, d_roll> are 
       | discarded)
   * - motion_reference/waypoints
     - as2_msgs/TrajectoryWaypoints
     - DefaultQoS
     - | Reference path to be tracked by the 
       | robot. The path is defined as a list 
       | of points. Each point is defined with 
       | a pose <x, y, z> and a yaw mode 
       | (Keep Yaw, Path Facing or Generate 
       | Yaw Trajectory).
   * - motion_reference/trajectory
     - trajectory_msgs/JointTrajectoryPoint
     - SensorDataQoS
     - | Desired trajectory reference with 
       | points, velocities and accelerations.

.. list-table:: ACTUATOR_COMMANDS
   :widths: 50 50 50 50
   :header-rows: 1

   * - Topic name
     - ROS message
     - Quality of Service
     - Description
   * - actuator_command/thrust
     - mavros_msgs/Thrust
     - SensorDataQoS
     - | Actuator command for the multirotor 
       | specifying thrust (N: Newtons)
   * - actuator_command/pose
     - geometry_msgs/PoseStamped
     - SensorDataQoS
     - | Actuator command for the multirotor 
       | specifying x, y, z, roll, pitch and yaw
   * - actuator_command/speed
     - geometry_msgs/TwistStamped
     - SensorDataQoS
     - | Actuator command for the multirotor 
       | specifying dx, dy, dz, roll rate, 
       | pitch rate and yaw rate

* **Services**

.. list-table:: PLATFORM (SERVICES)
   :widths: 50 50 50 50
   :header-rows: 1

   * - Service name
     - ROS service
     - Description
   * - platform/set_arming_state
     - std_srvs/SetBool
     - Arm / Disarm platform.
   * - platform/set_offboard_mode
     - std_srvs/SetBool
     - Enable offboard mode.
   * - platform/set_control_mode
     - as2_srvs/SetControlMode
     - Set platform control mode.
   * - platform/takeoff
     - std_srvs/SetBool
     - Takeoff platform.
   * - platform/land
     - std_srvs/SetBool
     - Land platform.
   * - platform/state_machine_event
     - as2_srvs/SetPlatformStateMachineEvent
     - | Send event to Platform 
       | State Machine.
   * - platform/list_control_mode
     - as2_srvs/ListControlModes
     - List available platform control modes.
