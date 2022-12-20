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

.. list-table:: Platform
   :widths: 50 50 50 50
   :header-rows: 1

   * - Topic name
     - ROS message
     - Quality of Service
     - Description
   * - platform/info
     - as2_msgs/PlatformInfo
     - DefaultQoS
     - | Information about the robot: connection_state, arming_state, 
       | offboard_state, fly status and control_mode.
