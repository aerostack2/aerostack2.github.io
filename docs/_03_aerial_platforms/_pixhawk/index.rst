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

You can see the `PX4 Autopilot ROS2 Guide <https://docs.px4.io/main/en/ros/ros2_comm.html>`_ for it installation, or follow the steps below.

We will use a colcon workspace for the installation:

  .. code-block:: bash

    mkdir -p ~/px4_ws/src

.. note::
  You can add the following line to your ``~/.bashrc`` file to source the workspace automatically.

  .. code-block:: bash

    echo "source ~/px4_ws/install/setup.bash" >> ~/.bashrc



.. _aerial_platform_px4_installation_dependencies:

PX4 Dependencies
================

Follow the steps below to install the dependencies for PX4:

1. Clone the PX4 Messages repository:

  .. code-block:: bash

    cd ~/px4_ws/src
    git clone https://github.com/PX4/px4_msgs.git

3. Clone the PX4 ROS COM:

  .. code-block:: bash

    cd ~/px4_ws/src
    git clone https://github.com/PX4/px4_ros_com.git

4. Build workspace:

  .. code-block:: bash

    cd ~/px4_ws
    colcon build --symlink-install



.. _aerial_platform_px4_installation_platform:

Install platform package
========================

Clone the Aerostack2 Pixhawk Platform repository into the workspace created in the section before:

.. code-block:: bash

  cd ~/px4_ws/src
  git clone https://github.com/aerostack2/as2_platform_pixhawk.git

Build it:

.. code-block:: bash

  source ~/px4_ws/install/setup.bash
  cd ~/px4_ws
  colcon build --symlink-install



.. _aerial_platform_px4_installation_xrce_dds:

XRCE-DDS (PX4-FastDDS Bridge)
=============================

For more information about XRCE-DDS, see `PX4 Autopilot XRCE-DDS bridge <https://docs.px4.io/main/en/middleware/xrce_dds.html>`_.
For it installation, clone it into the workspace and build it using colcon:

.. code-block:: bash

  cd ~/px4_ws/src
  git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git

Build it:

.. code-block:: bash

  cd ~/px4_ws
  colcon build --symlink-install



.. _aerial_platform_px4_installation_px4_autopilot:

PX4 Autopilot
=============

For more information about PX4 Autopilot, see `PX4 Autopilot <https://docs.px4.io/main/en/>`_.

* For simulation using Gazebo Classic, clone it and set PX4_FOLDER environment variable:

  1. Clone repository

    .. code-block:: bash

      mkdir -p ~/px4_ws/thirdparties
      cd ~/px4_ws/thirdparties
      git clone https://github.com/PX4/PX4-Autopilot.git --recursive

  2. Add colcon ignore file

    .. code-block:: bash

      cd ~/px4_ws/thirdparties/
      touch COLCON_IGNORE

  3. Set PX4_FOLDER environment variable

    .. code-block:: bash

      echo "export PX4_FOLDER=~/px4_ws/thirdparties/PX4-Autopilot" >> ~/.bashrc

* For using `PX4 Vision Autonomy Development Kit <https://docs.px4.io/main/en/complete_vehicles/px4_vision_kit.html>`_ see our :ref:`PX4 Vision Guide <aerial_platform_px4_installation_px4_autopilot_px4_vision>`.

.. toctree::
  :hidden:

  px4_vision/setup.rst



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

