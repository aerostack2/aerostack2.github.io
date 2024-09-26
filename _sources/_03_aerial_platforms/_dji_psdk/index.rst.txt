.. _aerial_platform_dji_matrice_psdk_psdk:

=======================
DJI Matrice Series PSDK
=======================

.. contents:: Table of Contents
   :depth: 3
   :local:



.. _aerial_platform_dji_matrice_psdk_introduction:

------------
Introduction
------------

DJI Matrice Series using `DJI Onboard PSDK <https://github.com/dji-sdk/Onboard-SDK>`_ has compatibility with DJI M300, DJI M350 and DJI M30.

.. figure:: resources/DJI_M300.jpg
   :scale: 15
   :class: with-shadow



.. _aerial_platform_dji_matrice_psdk_installation:

------------
Installation
------------

.. _aerial_platform_matrice_psdk_installation_prerequisites:

Prerequisites
=============

Refer to the `DJI PSDK ROS2 <https://github.com/umdlife/psdk_ros2>`_ for the harware and software requirements.


.. _aerial_platform_dji_matrice_psdk_installation_package:

Install platform package
========================

* For binary installation, install by running:

.. code-block:: bash

   sudo apt install ros-humble-as2-platform-dji-psdk


* For source installation, follow the steps below:

.. code-block:: bash

    # If you have installed Aerostack2 from sources we recommend to clone the package in the src folder of your workspace otherwise you can clone it in any ROS 2 workspace you want.
    cd ~/aerostack2_ws/src/aerostack2/as2_aerial_platforms
    git clone git@github.com:aerostack2/as2_platform_dji_psdk.git
    cd ~/aerostack2_ws
    rosdep install as2_platform_dji_psdk --from-paths src --ignore-src -r -y
    colcon build --packages-up-to as2_platform_dji_psdk

.. _aerial_platform_dji_matrice_psdk_as2_common_interface:

---------------------------
Aerostack2 Common Interface
---------------------------

For more details about platform control modes and sensors, see :ref:`Aerostack2 Aerial Platform Concepts <as2_concepts_aerial_platform>`.



.. _aerial_platform_dji_matrice_psdk_as2_common_interface_control_modes:

Control Modes
=============

These are supported control modes:

.. list-table:: Control Modes DJI PSDK Platform
   :widths: 50 50 50
   :header-rows: 1

   * - Control Mode
     - Yaw Mode
     - Reference Frame
   * - Speed
     - Speed
     - ENU


.. _aerial_platform_dji_matrice_psdk_as2_common_interface_sensors:

Sensors
=======

These are supported sensors:
  
.. list-table:: Sensors DJI PSDK Platform
   :widths: 50 50 50
   :header-rows: 1

   * - Sensor
     - Topic
     - Type
   * - Odometry
     - sensor_measurements/odom
     - nav_msgs/Odometry
   * - IMU
     - sensor_measurements/imu
     - sensor_msgs/Imu
   * - Battery
     - sensor_measurements/battery
     - sensor_msgs/BatteryState
   * - GPS
     - sensor_measurements/gps
     - sensor_msgs/NavSatFix
   * - Camera
     - sensor_measurements/camera
     - sensor_msgs/Image


.. _aerial_platform_dji_matrice_psdk_platform_launch:

---------------
Platform Launch
---------------

Aerostack2 provides a launch file for this platform:

.. code-block:: bash

  ros2 launch as2_platform_dji_psdk as2_platform_dji_psdk.launch.py

Also, `ROS 2 PSDK Wrapper <https://github.com/umdlife/psdk_ros2>`_ must be launched before the platform:

.. code-block:: bash

  ros2 launch as2_platform_dji_psdk psdk_wrapper.launch.py

To see all the **available parameters**, use the **'-s'** flag to show the description of each parameter in the launch file.
