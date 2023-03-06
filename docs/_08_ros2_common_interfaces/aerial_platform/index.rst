.. _ros2_common_interfaces_aerial_platform:

================
Aerial Platform
================

.. contents:: Contents
   :depth: 2
   :local:



.. _ros2_common_interfaces_aerial_platform_topics:

------
Topics
------

* Platform publish its finite state machine (FSM) status.

.. list-table:: Aerostack2 Aerial Platform Info Topic
   :widths: 50 50 50 50
   :header-rows: 1

   * - Name
     - Type
     - Quality of Service
     - Description
   * - platform/info
     - as2_msgs/PlatformInfo
     - DefaultQoS
     - | Information about the robot: connection_state,
       | arming_state, offboard_state, 
       | fly status and control_mode

* Platform suscribe to actuator commands.

.. list-table:: Aerostack2 Aerial Platform Actuator Commands Topics
   :widths: 50 50 50 50
   :header-rows: 1

   * - Name
     - Type
     - Quality of Service
     - Description
   * - actuator_command/pose
     - geometry_msgs/PoseStamped
     - SensorDataQoS
     - | Actuator command for the multirotor 
       | specifying x, y and z (m: meters) and 
       | roll, pitch and yaw (rad: radians)
   * - actuator_command/twist
     - geometry_msgs/TwistStamped
     - SensorDataQoS
     - | Actuator command for the multirotor 
       | specifying vx, vy and vz (m/s: meters per seconds) and 
       | roll rate, pitch rate and yaw rate (rad/s: radians per seconds)
   * - actuator_command/thrust
     - as2_msgs/Thrust
     - SensorDataQoS
     - | Actuator command for the multirotor 
       | specifying thrust (N: Newtons)

* Platform publish sensor data.

.. TBD: Add all sensors, like odometry, lidar, etc.

.. list-table:: Aerostack2 Aerial Platform Sensor Measurements Topics
   :widths: 50 50 50 50
   :header-rows: 1

   * - Name
     - Type
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



.. _ros2_common_interfaces_aerial_platform_services:

--------
Services
--------

* Platform services.

.. list-table:: Aerostack2 Aerial Platform Services
   :widths: 50 50 50
   :header-rows: 1

   * - Name
     - Type
     - Description
   * - platform/set_arming_state
     - std_srvs/SetBool
     - Arm / Disarm platform
   * - platform/set_offboard_mode
     - std_srvs/SetBool
     - Enable / Disable offboard mode
   * - platform/set_control_mode
     - as2_srvs/SetControlMode
     - Set platform control mode
   * - platform/takeoff
     - std_srvs/SetBool
     - Takeoff platform
   * - platform/land
     - std_srvs/SetBool
     - Land platform
   * - platform/state_machine_event
     - as2_srvs/SetPlatformStateMachineEvent
     - Send event to Platform State Machine
   * - platform/list_control_mode
     - as2_srvs/ListControlModes
     - List available platform control modes

