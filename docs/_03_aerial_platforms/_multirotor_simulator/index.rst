.. _aerial_platform_multirotor_simulator:

====================
Multirotor Simulator
====================

.. contents:: Table of Contents
   :depth: 3
   :local:



.. _aerial_platform_multirotor_simulator_introduction:

------------
Introduction
------------

For simulation purposes with `Multirotor simulator <https://github.com/RPS98/multirotor_simulator>`__ , Aerostack2 provides with a platform that serves as an entry point for aerial robotics simulated in this environment.

.. figure:: resources/multirotor_simulator_main.png
   :scale: 50
   :class: with-shadow

.. _aerial_platform_multirotor_simulator_installation:

------------
Installation
------------

* For binary installation, install by running:

.. code-block:: bash

   sudo apt install ros-humble-as2-platform-multirotor-simulator

* For source installation, clone Aerostack2 repository into your workspace and build it. See it in :ref:`Aerostack2 installation guide <getting_started_ubuntu_installation_source>`.




.. _aerial_platform_multirotor_simulator_as2_common_interface:

---------------------------
Aerostack2 Common Interface
---------------------------

For more details about platform control modes and sensors, see :ref:`Aerostack2 Aerial Platform Concepts <as2_concepts_aerial_platform>`.



.. _aerial_platform_multirotor_simulator_as2_common_interface_control_modes:

Control Modes
=============

These are supported control modes:

.. list-table:: Control Modes Multirotor Simulator Platform
   :widths: 50 50 50
   :header-rows: 1

   * - Control Mode
     - Yaw Mode
     - Reference Frame
   * - Hover
     - None
     - None
   * - Position
     - Angle
     - ENU
   * - Position
     - Speed
     - ENU
   * - Speed
     - Angle
     - ENU
   * - Speed
     - Speed
     - ENU
   * - Trajectory
     - Angle
     - ENU
   * - Trajectory
     - Speed
     - ENU
   * - Acro
     - None
     - None


.. _aerial_platform_multirotor_simulator_as2_common_interface_sensors:

Sensors
=======

These are supported sensors:
  
.. list-table:: Sensors Multirotor Simulator Platform
   :widths: 50 50 50
   :header-rows: 1

   * - Sensor
     - Topic
     - Type
   * - IMU
     - sensor_measurements/imu
     - sensor_msgs/msg/Imu
   * - GPS
     - sensor_measurements/gps
     - sensor_msgs/msg/NavSatFix

Gimbal
======

Gimbal is supported in simulation. These are the supported gimbal model types:

.. list-table:: Gimbal Control Modes Multirotor Simulator Platform
   :widths: 50 50 50 50
   :header-rows: 1

   * - Gimbal type
     - Topic
     - Type
     - Control mode id
   * - gimbal_position
     - platform/{gimbal_name}/gimbal_command
     - as2_msgs/msg/GimbalControl
     - "0"

Gimbal state is published in the following topics:

.. list-table:: Gimbal State Multirotor Simulator Platform
   :widths: 50 50
   :header-rows: 1

   * - Topic
     - Type
   * - sensor_measurements/{gimbal_name}
     - geometry_msgs/msg/PoseStamped

.. _aerial_platform_multirotor_simulator_config_simulation:

-----------------
Config Simulation
-----------------

Configuration files are located in ``as2_platform_multirotor_simulator/config/``:

* ``control_modes.yaml``: available input control modes for the aerostack2 aerial platform.
* ``platform_config_file.yaml``: ROS 2 configuration, with tf names, frequencies and aerostack2 parameters.
* ``uav_config.yaml``: UAV configuration, with the drone's physical parameters and controller configuration.
* ``world_config.yaml``: world configuration, as parameters override for each drone namespace in the configuration file.


.. _aerial_platform_multirotor_simulator_platform_launch:

---------------
Platform Launch
---------------

Aerostack2 provides a launch file for this platform:

.. code-block:: bash

  ros2 launch as2_platform_multirotor_simulator as2_platform_multirotor_simulator.launch.py

To see all the **available parameters**, use the **'-s'** flag to show the description of each parameter in the launch file.
