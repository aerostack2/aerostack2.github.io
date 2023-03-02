.. _ros2_common_interfaces_state_estimator:

=================
State Estimator
=================

.. contents:: Contents
   :depth: 2
   :local:



.. _ros2_common_interfaces_state_estimator_topics:

------
Topics
------

* State Estimator publishes the platform localization.

.. list-table:: Aerostack2 State Estimator Topics
   :widths: 50 50 50 50
   :header-rows: 1

   * - Name
     - Type
     - Quality of Service
     - Description
   * - self_localization/pose
     - geometry_msgs/PoseStamped
     - SensorDataQoS
     - | Localization of the multirotor
       | specifying x, y and z (m: meters) and 
       | roll, pitch and yaw (rad: radians)
   * - self_localization/twist
     - geometry_msgs/TwistStamped
     - SensorDataQoS
     - | Localization of the multirotor
       | specifying vx, vy and vz (m/s: meters per seconds) and 
       | roll rate, pitch rate and yaw rate (rad/s: radians per seconds)

.. TBD: Add services and actions