.. _ros2_common_interfaces_behaviors:


.. TBD: Generalize this to a generic behaviors

=========
Behaviors
=========

.. contents:: Contents
   :depth: 2
   :local:



.. _ros2_common_interfaces_behaviors_actions:

-------
Actions
-------

Action messages part of behavior definition.

.. list-table:: BEHAVIORS
   :widths: 50 50 50
   :header-rows: 1

   * - Action name
     - ROS action
     - Description
   * - TakeOffBehavior
     - as2_msgs/TakeOff
     - Do takeoff with a given height and speed.
   * - GoToWaypointBehavior
     - as2_msgs/GoToWaypoint
     - | Go to a specific position with a 
       | given cruise speed and yaw mode.
   * - FollowPathBehavior
     - as2_msgs/FollowPath
     - Follow a given path.
   * - LandBehavior
     - as2_msgs/Land
     - Land with a given speed.