.. _ros2_common_interfaces_motion_controller:

=================
Motion Controller
=================

.. contents:: Contents
   :depth: 2
   :local:



.. _ros2_common_interfaces_motion_controller_topics:

------
Topics
------

* Motion controller publish its state.

.. list-table:: Aerostack2 Motion Controller Info Topic
   :widths: 50 50 50 50
   :header-rows: 1

   * - Name
     - Type
     - Quality of Service
     - Description
   * - controller/info
     - as2_msgs/ControllerInfo
     - DefaultQoS
     - Information about the controller state.

* Motion controller suscribe to motion reference.


.. list-table:: Aerostack2 Motion Controller Motion References Topics
   :widths: 50 50 50 50
   :header-rows: 1

   * - Name
     - Type
     - Quality of Service
     - Description
   * - motion_reference/pose
     - geometry_msgs/PoseStamped
     - SensorDataQoS
     - | Actuator command for the multirotor 
       | specifying x, y and z (m: meters) and 
       | roll, pitch and yaw (rad: radians)
   * - motion_reference/twist
     - geometry_msgs/TwistStamped
     - SensorDataQoS
     - | Actuator command for the multirotor 
       | specifying vx, vy and vz (m/s: meters per seconds) and 
       | roll rate, pitch rate and yaw rate (rad/s: radians per seconds)
   * - motion_reference/trajectory
     - as2_msgs/TrajectoryPoint
     - DefaultQoS
     - | Reference trajectory point specifying 
       | x, y and z (m: meters),
       | vx, vy and vz (m/s: meters per seconds),
       | ax, ay and az (m/s²: meters per seconds squared) and
       | yaw angle (rad: radians)

* Motion controller publish actuator commands to aerial platform.

.. list-table:: Aerostack2 Motion Controller Actuator Commands Topics
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



.. _ros2_common_interfaces_motion_controller_services:

--------
Services
--------

* Motion controller services.

.. list-table:: Aerostack2 Motion Controller Services
   :widths: 50 50 50
   :header-rows: 1

   * - Name
     - Type
     - Description
   * - controller/set_control_mode
     - as2_srvs/SetControlMode
     - Set controller control mode
   * - controller/list_control_mode
     - as2_srvs/ListControlModes
     - List available controller control modes

