.. _aerial_platform_gazebo:

======
Gazebo
======

.. contents:: Table of Contents
   :depth: 3
   :local:



.. _aerial_platform_gazebo_introduction:

------------
Introduction
------------

For simulation purposes with `Gazebo simulator <https://gazebosim.org/api/gazebo>`__ , Aerostack2 provides with a platform that serves as an entry point for aerial robotics simulated in this environment.



.. _aerial_platform_gazebo_installation:

------------
Installation
------------



.. _aerial_platform_gazebo_installation_prerequisites:

Prerequisites
=============

Ignition Gazebo is required for simulation using this simulator. The default Gazebo version for working
and simulating with Aerostack2 is Gazebo Fortress, which can be installed following the
`Gazebo Fortress installation guide <https://gazebosim.org/docs/fortress/install_ubuntu>`__.

Using 'Gazebo Harmonic'
-----------------------

Gazebo Harmonic is also supported by Aerostack2. For simulations to work with this version,
make sure Gazebo Fortress is completely removed from your machine, since both versions are not
compatible with Aerostack2 at the same time. To do so, first run:

.. code-block:: bash

  sudo apt-get remove ignition-fortress
  sudo apt remove ignition*
  sudo apt autoremove

To ensure no ignition-fortress packages are cached with the Aerostack2 compilation, run the following
`Aerostack2 CLI <https://aerostack2.github.io/_09_development/_cli/index.html#development-cli>`_ command to clean Aerostack2 if you have it compilled:

.. code-block:: bash

  as2 clean -a
  source ~/.bashrc

After that, you can follow the `Gazebo Harmonic installation guide <https://gazebosim.org/docs/harmonic/install_ubuntu>`__
to install the new Gazebo version. 

.. warning:: An additional package must be installed for Aerostack2 to build:

  .. code-block:: bash

    sudo apt install ros-humble-ros-gzharmonic

.. note:: Gazebo Fortress can be re-installed to your machine after Aerostack2 has been compiled with Gazebo Harmonic if you need it, but compiling Aerostack2 with Fortress again will not be possible.

.. _aerial_platform_gazebo_installation_package:

Install platform package
========================

* For binary installation, install by running:

.. code-block:: bash

   sudo apt install ros-humble-as2-platform-gazebo

.. warning:: This package is not available for binary installation yet. Cooming soon.

* For source installation, clone Aerostack2 repository into your workspace and build it. See it in :ref:`Aerostack2 installation guide <getting_started_ubuntu_installation_source>`.



.. _aerial_platform_gazebo_installation_assets:

Install simulation assets
=========================

* For binary installation, install by running:

.. code-block:: bash

   sudo apt install ros-humble-as2-gazebo-assets

.. warning:: This package is not available for binary installation yet. Cooming soon.

* For source installation, clone Aerostack2 repository into your workspace and build it. See it in :ref:`Aerostack2 installation guide <getting_started_ubuntu_installation_source>`.



.. _aerial_platform_gazebo_as2_common_interface:

---------------------------
Aerostack2 Common Interface
---------------------------

For more details about platform control modes and sensors, see :ref:`Aerostack2 Aerial Platform Concepts <as2_concepts_aerial_platform>`.
For more details, about bridge between Gazebo and ROS, see the :ref:`Aerostack2 Gazebo ROS Bridge <aerial_platform_gazebo_ros_gz_bridge>`.



.. _aerial_platform_gazebo_as2_common_interface_control_modes:

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
     - FLU



.. _aerial_platform_gazebo_as2_common_interface_sensors:

Sensors
=======

These are supported sensors:
  
.. list-table:: Sensors Gazebo Platform
   :widths: 50 50 50
   :header-rows: 1

   * - Sensor
     - Topic
     - Type
   * - IMU
     - sensor_measurements/imu
     - sensor_msgs/msg/Imu
   * - Magnetometer
     - sensor_measurements/magnetic_field
     - sensor_msgs/msg/MagneticField
   * - Air pressure
     - sensor_measurements/air_pressure
     - sensor_msgs/msg/FluidPressure
   * - Battery
     - sensor_measurements/battery
     - sensor_msgs/msg/BatteryState
   * - Camera
     - sensor_measurements/{model_name}/image_raw
     - sensor_msgs/msg/Image
   * - Depth camera
     - sensor_measurements/{model_name}/depth
     - sensor_msgs/msg/Image
   * - Camera
     - sensor_measurements/{model_name}/camera_info
     - sensor_msgs/msg/CameraInfo
   * - Lidar
     - sensor_measurements/{model_name}/scan
     - sensor_msgs/msg/LaserScan
   * - Lidar
     - sensor_measurements/{model_name}/points
     - sensor_msgs/msg/PointCloud2
   * - Camera
     - sensor_measurements/{model_name}/points
     - sensor_msgs/msg/PointCloud2

Gimbal
======

Gimbal is supported in simulation. These are the supported gimbal model types:

.. list-table:: Gimbal Control Modes Ignition Gazebo Platform
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
   * - gimbal_speed
     - platform/{gimbal_name}/gimbal_command
     - as2_msgs/msg/GimbalControl
     - "1"

Gimbal state is published in the following topics:

.. list-table:: Gimbal State Ignition Gazebo Platform
   :widths: 50 50
   :header-rows: 1

   * - Topic
     - Type
   * - sensor_measurements/{gimbal_name}/twist
     - geometry_msgs/msg/Vector3Stamped
   * - sensor_measurements/{gimbal_name}/attitude
     - geometry_msgs/msg/QuaternionStamped

.. _aerial_platform_gazebo_config_simulation:

-----------------
Config Simulation
-----------------

There are two aerial models available for simulation. These models are:

* Quadrotor base.
  
.. figure:: images/quadrotor.png
   :width: 400
   :scale: 50
   :class: with-shadow
   
   Quadrotor base model

* Hexrotor base.

.. figure:: images/hexrotor.png
   :width: 400
   :scale: 50
   :class: with-shadow
   
   Hexrotor base model

In order to add an aerial model with sensors attached to it to the simulated world, Aerostack2 uses a configuration file, with YAML format, with the following structure:

.. code-block:: yaml

  world_name: "empty"
  drones:
    - model_name: "drone0"
      model_type: "quadrotor_base"
      xyz:
        - -2.0
        - 0.0
        - 0.3
      rpy:
        - 0
        - 0
        - 0.0
      flight_time: 60
      payload:
        - model_name: "hd_camera0"
          model_type: "hd_camera"
        - model_name: "gimbal0"
          model_type: "gimbal_position"
          payload:
            model_name: "hd_camera1"
            model_type: "hd_camera"

    - model_name: "drone1"
      model_type: "quadrotor_base"
      xyz:
        - 2.0
        - 0.0
        - 0.3
      rpy:
        - 0
        - 0
        - 0.0
      flight_time: 60
      payload:
        - model_name: "gimbal1"
          model_type: "gimbal_speed"
          payload:
            model_name: "hd_camera1"
            model_type: "hd_camera"

    - model_name: "drone2"
      model_type: "quadrotor_base"
      xyz:
        - 0.0
        - 0.0
        - 0.3
      rpy:
        - 0
        - 0
        - 0.0
      flight_time: 60
      payload:
        - model_name: "hd_camera2"
          model_type: "hd_camera"

Where:

* ``world_name``: name of the defined world in sdf format.
* ``drones``: list of drones to be included in the world.
  
Each of the ``drones`` is defined by:

* ``model_type``: model of the drone defined in sdf format.
* ``model_name``: namespace
* ``xyz``: spawn position
* ``rpy``: spawn orientation 
* ``payload``: list of sensors/gimbal attached to the model

Each element of the ``payload`` is defined by:

* ``model_type``: name of the sensor/gimbal inside the simulation (this case ``gps`` and ``gimbal_speed``)
* ``model_name``: name of the sensor/gimbal defined in sdf format.

If a drone ``payload`` contains a gimbal, a gimbal should contain a payload which must containt a sensor.

New models, sensors and worlds are defined in the ``as2_gazebo_assets`` package. For more information on how to create new assets, go to the `Gazebo Fortress tutorial page <https://gazebosim.org/docs/fortress/tutorials>`_.



.. _aerial_platform_gazebo_platform_launch:

---------------
Platform Launch
---------------

Aerostack2 Gazebo platform provides a launch file, which parameters are:

.. list-table:: Gazebo Platform Parameters
   :widths: 50 50 50
   :header-rows: 1

   * - Parameter
     - Type
     - Description
   * - namespace
     - string
     - Namespace of the platform, also named as drone id. 
   * - simulation_config_file
     - string
     - Path to the simulation configuration file.
   * - control_modes_file
     - string
     - Optional. File path with the control modes configuration. Default the one in the package.
   * - platform_config_file
     - string
     - Optional. File path with additional platform parameters.
   * - log_level
     - string
     - Optional. Set Logging level. Default 'info'.  
   * - use_sim_time
     - bool
     - Optional. Syncronize simulation time with node time. Default false.


Example of launch command:

.. code-block:: bash

  ros2 launch as2_platform_gazebo platform_gazebo_launch.py namespace:=drone_sim_0 simulation_config_file:=world_json_path

For launch the simulation, run the following command:

.. code-block:: bash

  ros2 launch as2_gazebo_assets launch_simulation.py simulation_config_file:=world_json_path


Additionally, for launching teleoperation and trying out a basic mission, continue to the [Gazebo Example Project](https://aerostack2.github.io/_02_examples/gazebo/project_gazebo/index.html)
