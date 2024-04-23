.. _aerial_platform_crazyflie:

==================
Bitcraze Crazyflie
==================

.. contents:: Table of Contents
   :depth: 3
   :local:



.. _aerial_platform_crazyflie_introduction:

------------
Introduction
------------

Crazyflie 2.1 by Bitcraze can be found `here <https://www.bitcraze.io/products/crazyflie-2-1/>`_

.. figure:: resources/Crazyflie2_0.jpeg
   :scale: 50
   :class: with-shadow



.. _aerial_platform_crazyflie_installation:

------------
Installation
------------



.. _aerial_platform_crazyflie_installation_prerequisites:

Prerequisites
=============

First thing to do is to configure your Crazyflie(s). In order to do that, please follow these steps:

* Install the `cfclient <https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/installation/install/>`_.
* Setup `UDEV permissions <https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/installation/usb_permissions/>`_ for using the Crazyradio.
* Perform a `firmware upgrade <https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/userguides/userguide_client/#firmware-upgrade>`_.
* Configure the radio address for your crazyflie. To do this, connect your crazyflie, then open “Connect->Configure 2.X” to open the address configuration dialog.
  Please use a different address for each crazyflie to be used in a swarm.

These steps are meant to be done once.



.. _aerial_platform_crazyflie_installation_package:

Install platform package
========================

* For binary installation, install by running:

.. code-block:: bash

   sudo apt install ros-humble-as2-platform-crazyflie

.. warning:: This package is not available for binary installation yet. Cooming soon.

* For source installation, clone the platform repository into your workspace and build it. 

.. code-block:: bash

    # If you have installed Aerostack2 from sources we recommend to clone the package in the src folder of your workspace otherwise you can clone it in any ROS 2 workspace you want.
    cd ~/aerostack2_ws/src/aerostack2/as2_aerial_platforms
    git clone git@github.com:aerostack2/as2_platform_crazyflie.git
    cd ~/aerostack2_ws
    rosdep install as2_platform_crazyflie --from-paths src --ignore-src -r -y
    colcon build --packages-up-to as2_platform_crazyflie


See it in :ref:`Aerostack2 installation guide <getting_started_ubuntu_installation_source>`.



.. _aerial_platform_crazyflie_as2_common_interface:

---------------------------
Aerostack2 Common Interface
---------------------------

For more details about platform control modes and sensors, see :ref:`Aerostack2 Aerial Platform Concepts <as2_concepts_aerial_platform>`.



.. _aerial_platform_crazyflie_as2_common_interface_control_modes:

Control Modes
=============

These are supported control modes:

.. list-table:: Control Modes Gazebo Platform
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
   * - Speed in a plane
     - Speed
     - ENU



.. _aerial_platform_crazyflie_as2_common_interface_sensors:

Sensors
=======

These are supported sensors:
  
.. list-table:: Sensors Gazebo Platform
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



.. _aerial_platform_crazyflie_platform_launch:

---------------
Platform Launch
---------------

In order to map the adresses of the crazyflie(s) we just configured to the input adresses of the platfom, we need to use a ``.yaml`` configuration file with the format as the following example:

.. code-block:: yaml

    cf0:
        uri: radio://0/80/2M/E7E7E7E700
    cf1:
        uri: radio://0/80/2M/E7E7E7E701

In this case, we are mapping two Crazyflies with uri's ``radio://0/80/2M/E7E7E7E700`` and ``radio://0/80/2M/E7E7E7E701`` for namespaces ``cf0`` and ``cf1`` respectively.
As you can see for the radio id part of the uri, the same Crazyradio is going to be used for both. 

This configuration file's path shall serve as an input to the platform launch parameter ``swarm_config_file``. Note that this platform is launched only once, regardless of the number of crazyflies beeing used. 

Aerostack2 Crazyflie platform provides a launch file, which parameters are:

.. list-table:: Gazebo Platform Parameters
   :widths: 50 50 50
   :header-rows: 1

   * - Parameter
     - Type
     - Description
   * - swarm_config_file
     - string
     - Path to the swarm URI's configuration file.
   * - control_modes_file
     - string
     - Optional. File path with the control modes configuration. Default the one in the package.
   * - external_odom
     - bool
     - Optional. Use external odometry source. Default false.
   * - external_odom_topic
     - bool
     - Optional. Topic name of external odometry source. Default 'external_odom'.
   * - controller_type
     - int
     - Optional. Controller type: Any(0), PID(1), Mellinger(2), INDI(3). Default PID.
   * - estimator_type
     - int
     - Optional. Estimator type: Any(0), complementary(1), kalman(2). Default Kalman.

Example of launch command with a Crazyflies with flow deck:

.. code-block:: bash

  ros2 launch as2_platform_crazyflie crazyflie_swarm_launch.py swarm_config_file:=uris_yaml_path

Example of launch command with a Crazyflies with motion capture system:

.. code-block:: bash

  ros2 launch as2_platform_crazyflie crazyflie_swarm_launch.py swarm_config_file:=uris_yaml_path external_odom:=true external_odom_topic:=mocap_topic_name



.. _aerial_platform_crazyflie_platform_examples:

--------
Examples
--------

Basic example
=============

.. toctree::
   :maxdepth: 1

   ../../_02_examples/crazyflie/project_crazyflie/index.rst