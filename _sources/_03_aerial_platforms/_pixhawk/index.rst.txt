.. _aerial_platform_px4:

=========
Pixhawk 4
=========

.. contents:: Table of Contents
   :depth: 3
   :local:



.. _aerial_platform_px4_introduction:

------------
Introduction
------------

PX4 is an autopilot, which information can be found `here <https://docs.px4.io/main/en/>`_.

.. figure:: resources/px4_vision.jpg
   :scale: 50
   :class: with-shadow



.. _aerial_platform_px4_installation:

------------
Installation
------------

Refer to the `PX4 documentation <https://docs.px4.io/main/en/ros2/>`_ for installation instructions using XRCE-DDS.


.. _aerial_platform_px4_as2_common_interface:

---------------------------
Aerostack2 Common Interface
---------------------------

For more details about platform control modes and sensors, see :ref:`Aerostack2 Aerial Platform Concepts <as2_concepts_aerial_platform>`.



.. _aerial_platform_px4_as2_common_interface_control_modes:

Control Modes
=============

These are supported control modes:

.. list-table:: Control Modes Pixhawk Platform
   :widths: 50 50 50
   :header-rows: 1

   * - Control Mode
     - Yaw Mode
     - Reference Frame
   * - Hover
     - None
     - None
   * - Speed
     - Speed
     - ENU
   * - Speed
     - Angle
     - ENU
   * - Attitude
     - Angle
     - None
   * - Acro
     - None
     - None



.. _aerial_platform_px4_as2_common_interface_sensors:

Sensors
=======

These are supported sensors:
  
.. list-table:: Sensors Pixhawk Platform
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



.. _aerial_platform_px4_platform_launch:

---------------
Platform Launch
---------------

.. Aerostack2 Pixhawk platform provides a launch file, which parameters are:

.. .. list-table:: Pixhawk Platform Parameters
..    :widths: 50 50 50
..    :header-rows: 1

..    * - Parameter
..      - Type
..      - Description
..    * - namespace
..      - string
..      - Namespace of the platform, also named as drone id.
..    * - config
..      - string
..      - | Optional. File yaml path with the config file that set: 
..        | command frequency in Hz (cmd_freq), info frequency in Hz (info_freq), 
..        | mass in Kg (mass), maximum thrust in N (max_thrust), minimum thrust in N (min_thrust) and
..        | file path with the control modes configuration (control_modes_file). Default the file in the package.
..    * - external_odom
..      - bool
..      - Optional. Use external odometry source. Default false.
..    * - use_sim_time
..      - bool
..      - Optional. Syncronize simulation time with node time. Default false.

Aerostack2 provides a launch file for this platform:

.. code-block:: bash

  ros2 launch as2_platform_pixhawk pixhawk_launch.py

To see all the **available parameters**, use the **'-s'** flag to show the description of each parameter in the launch file.
