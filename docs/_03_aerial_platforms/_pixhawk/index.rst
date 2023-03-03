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

TDB



.. _aerial_platform_px4_installation:

------------
Installation
------------



.. _aerial_platform_px4_installation_prerequisites:

Prerequisites
=============

TBD: PX4 Autopilot is close to be released native to work in ROS2.

Since PX4 has a lot of external dependencies we opted to separate it in a diferent repo.
The complete installations step are as follows:

In order to be able to use a Pixhawk autopilot some additional software must be installed:
This documentation is obtained from ( ADD LINK )


Fast-DDS-Gen
------------

.. code-block:: bash

  sudo apt install -y openjdk-11-jdk
  
.. code-block:: bash

  git clone --recursive https://github.com/eProsima/Fast-DDS-Gen.git -b v1.0.4 \
      && cd Fast-DDS-Gen \
      && ./gradlew assemble \
      && sudo ./gradlew install

In order to validate the installation in another terminal run:

.. code-block:: bash

  # Check FastRTPSGen version
  fastrtpsgen_version_out=""
  if [[ -z $FASTRTPSGEN_DIR ]]; then
    fastrtpsgen_version_out="$FASTRTPSGEN_DIR/$(fastrtpsgen -version)"
  else
    fastrtpsgen_version_out=$(fastrtpsgen -version)
  fi
  if [[ -z $fastrtpsgen_version_out ]]; then
    echo "FastRTPSGen not found! Please build and install FastRTPSGen..."
    exit 1
  else
    fastrtpsgen_version="${fastrtpsgen_version_out: -5:-2}"
    if ! [[ $fastrtpsgen_version =~ ^[0-9]+([.][0-9]+)?$ ]] ; then
      fastrtpsgen_version="1.0"
      [ ! -v $verbose ] && echo "FastRTPSGen version: ${fastrtpsgen_version}"
    else
      [ ! -v $verbose ] && echo "FastRTPSGen version: ${fastrtpsgen_version_out: -5}"
    fi
  fi


The expected output is something similar to:

.. code-block:: bash 

  openjdk version "13.0.7" 2021-04-20
  OpenJDK Runtime Environment (build 13.0.7+5-Ubuntu-0ubuntu120.04)
  OpenJDK 64-Bit Server VM (build 13.0.7+5-Ubuntu-0ubuntu120.04, mixed mode)


pyros-genmsg
------------

A pyros-genmsg dependency is needed

.. code-block:: bash

  pip3 install pyros-genmsg
  


.. _aerial_platform_px4_installation_package:

Install platform package
========================

We recommend to have this in a diferent colcon_ws since px4_msgs packages spends a lot of time in compiling and we only want to do it once.

.. code-block:: bash

  mkdir -p ~/px4_ws/src && cd ~/px4_ws/src
  git clone git@github.com:aerostack2/as2_platform_pixhawk.git
  vcs import --recursive < as2_platform_pixhawk/dependencies.repos
  cd ~/px4_ws
  colcon build --symlink-install
 
.. note::
  Remind to ``source ~/aerostack2_ws/install/setup.bash`` before runing ``colcon build``



.. _aerial_platform_px4_as2_common_interface:

---------------------------
Aerostack2 Common Interface
---------------------------

For more details about platform control modes and sensors, see :ref:`Aerostack2 Aerial Platform Concepts <as2_concepts_aerial_platform>`.



.. _aerial_platform_px4_as2_common_interface_control_modes:

Control Modes
=============

These are supported control modes:

.. list-table:: Control Modes Ignition Gazebo Platform
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
  
.. list-table:: Sensors Ignition Gazebo Platform
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

Aerostack2 Pixhawk platform provides a launch file, which parameters are:

.. list-table:: Ignition Gazebo Platform Parameters
   :widths: 50 50 50
   :header-rows: 1

   * - Parameter
     - Type
     - Description
   * - namespace
     - string
     - Namespace of the platform, also named as drone id.
   * - config
     - string
     - | Optional. File yaml path with the config file that set: 
       | command frequency in Hz (cmd_freq), info frequency in Hz (info_freq), 
       | mass in Kg (mass), maximum thrust in N (max_thrust), minimum thrust in N (min_thrust) and
       | file path with the control modes configuration (control_modes_file). Default the file in the package.
   * - external_odom
     - bool
     - Optional. Use external odometry source. Default false.
   * - use_sim_time
     - bool
     - Optional. Syncronize simulation time with node time. Default false.

Example of launch command:

.. code-block:: bash

  ros2 launch as2_platform_pixhawk pixhawk_launch.launch namespace:=drone1

