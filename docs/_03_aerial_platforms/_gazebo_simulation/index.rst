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

  ros2 launch as2_platform_gazebo platform_gazebo_launch.py namespace:=drone_sim_0 config_file:=world_json_path

For launch the simulation, run the following command:

.. code-block:: bash

  ros2 launch as2_gazebo_assets launch_simulation.py config_file:=world_json_path

------------------------
Adding new Gazebo Assets
------------------------

New worlds and models can be added and configured to work with Aerostack2 in Gazebo simulations.

Adding New Worlds
=================

The ``as2_gazebo_assets`` package has a ``worlds`` folder in which more ``.sdf`` models can be added to use as worlds in your YAML simulation configuration file. The ``world_name`` tag
in your configuration file must match the name of an existing world in this directory. An empty world would look like the following template

.. code-block:: xml

  <?xml version="1.0" ?>
  <sdf version="1.6">
    <world name="empty">
          <physics name="4ms" type="ignored">
            <max_step_size>0.004</max_step_size>
            <real_time_factor>1.0</real_time_factor>
          </physics>
          <plugin
              filename="ignition-gazebo-physics-system"
              name="ignition::gazebo::systems::Physics">
          </plugin>
          <plugin
              filename="ignition-gazebo-scene-broadcaster-system"
              name="ignition::gazebo::systems::SceneBroadcaster">
          </plugin>
          <plugin
              filename="ignition-gazebo-user-commands-system"
              name="ignition::gazebo::systems::UserCommands">
          </plugin>
          <plugin
              filename="ignition-gazebo-sensors-system"
              name="ignition::gazebo::systems::Sensors">
              <render_engine>ogre2</render_engine>
          </plugin>

          <light type="directional"
              name="sun">
              <cast_shadows>true</cast_shadows>
              <pose>0 0 10 0 0 0</pose>
              <diffuse>0.8 0.8 0.8 1</diffuse>
              <specular>0.2 0.2 0.2 1</specular>
              <attenuation>
                  <range>1000</range>
                  <constant>0.9</constant>
                  <linear>0.01</linear>
                  <quadratic>0.001</quadratic>
              </attenuation>
              <direction>-0.5 0.1 -0.9</direction>
          </light> 
    </world>
  </sdf>

Worlds can also be added from an external source. For Gazebo to find the world file when the simulation is launched, make sure you have your Gazebo sources environment variable
set to look for world files in your external folder. The content of this variable can be checked by running

.. code-block:: bash

  echo $GZ_SIM_RESOURCE_PATH

Add the path to your external ``worlds`` folder using

.. code-block:: bash

  export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:<path/to/directory>/worlds

.. note:: If your are adding your worlds and models from a project which follows the project structure like this `project_gazebo <https://github.com/aerostack2/project_gazebo>`__. example, these environment variables can be set from your launch .bash file so that Gazebo can always find your models.

Adding New Models and Objects
=============================

Your new worlds can include any model that can be found by Gazebo in the ``models`` directory inside ``as2_gazebo_assets`` or in an external directory that has been added to the
same environment variable that was mentioned above

.. code-block:: bash

  export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:<path/to/directory>/models

Although static models can be added directly to your world, models with a specific function that will operate with Aerostack2 would be better added to your world as an Object. Objects
are models that are added to your world by configuring them in your simulation configuration file. This models have to be added to a models directory where Gazebo can find them, just as
any other model. The models are then loaded into your world like in the next simulation configuration example:

.. code-block:: yaml

  world_name: "empty"
  drones:
    - model_type: "crazyflie"
      model_name: "crazyflie0"
      payload:
        - model_name: "hd_camera0"
          model_type: "hd_camera"
        - model_name: "gimbal0"
          model_type: "gimbal_speed"
          payload:
            model_name: "gb0_hd_camera"
            model_type: "hd_camera"
      xyz:
        - -1.0
        - 0.0
        - 3.0

  objects:
    - model_name: "ground_plane"
      model_type: "ground_plane"
    - model_name: "aruco_gate"
      model_type: "aruco_gate_1"
      object_bridges:
        - "pose"
      xyz:
        - 1.0
        - 0.0
        - 0.0
   
The ``model_type`` tag is, again, the name of an existing model to be loaded. By adding models as objects, it is possible to configure object bridges using the ``object_bridges`` tag.
In the example, a ``pose`` bridge is added to the aruco gate for it to publish its pose in a ROS 2 topic. For this to work, the systems::PosePublisher Gazebo plugin was added to the
``aruco_gate_1`` SDF model.

The next table shows the bridges that can be added to the ``object_bridges`` tag

.. list-table:: Available Object Bridges
   :widths: 50 50 50
   :header-rows: 1

   * - object_bridge
     - Type
     - Topic
   * - pose
     - geometry_msgs/msg/PoseStamped
     - /<model_name>/<model_name>/pose
   * - azimuth
     - std_msgs::msg::Float32
     - /<model_name>/gps/azimuth
   * - gps
     - sensor_msgs/msg/NavSatFix
     - /<model_name>/sensor_measurements/gps

The topics specified in the 'Topic' column provide information about your model in Gazebo that can be used by any ROS 2 node.

Adding New Platform Models
==========================

Adding new platform models so they can fly using Aerostack2 requires the creation of a Jinja template. Aerostack2 uses Jinja to templatize the Gazebo Platform models so that all the components,
links, joints and topics have the right names and the platform can be built properly.

Adding New Quadrotor Models
---------------------------

To add a new quadrotor model, like a BitCraze Crazyflie for instance, a file ``model-name.sdf.jinja`` must be added to your model folder, with the following content

``TODO?: REPLACE THIS WITH A LINK TO THE TEMPALTE``

.. code-block:: xml

  <?xml version="1.0"?>

  <sdf version='1.6'>
    <model name='{{ namespace }}'>
      <include merge="true">
          <uri>model://MODEL_NAME</uri>
      </include>

        <plugin
            filename="ignition-gazebo-pose-publisher-system"
            name="ignition::gazebo::systems::PosePublisher">
            <publish_link_pose>true</publish_link_pose>
            <publish_sensor_pose>true</publish_sensor_pose>
            <publish_collision_pose>false</publish_collision_pose>
            <publish_visual_pose>false</publish_visual_pose>
            <publish_nested_model_pose>true</publish_nested_model_pose>
            <publish_model_pose>false</publish_model_pose>
            <use_pose_vector_msg>true</use_pose_vector_msg>
            <static_publisher>true</static_publisher>
            <static_update_frequency>100</static_update_frequency>
        </plugin>
        <plugin
          filename="ignition-gazebo-multicopter-motor-model-system"
          name="gz::sim::systems::MulticopterMotorModel">
          <robotNamespace>model/{{ namespace }}</robotNamespace>
          <jointName>m1_joint</jointName>
          <linkName>m1</linkName>
          <turningDirection>ccw</turningDirection>
          <timeConstantUp>0.0125</timeConstantUp>
          <timeConstantDown>0.025</timeConstantDown>
          <maxRotVelocity>800.0</maxRotVelocity>
          <motorConstant>8.54858e-06</motorConstant>
          <momentConstant>0.016</momentConstant>
          <commandSubTopic>command/motor_speed</commandSubTopic>
          <motorNumber>0</motorNumber>
          <rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>
          <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
          <motorSpeedPubTopic>motor_speed/0</motorSpeedPubTopic>
          <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
          <motorType>velocity</motorType>
        </plugin>
        <plugin
          filename="ignition-gazebo-multicopter-motor-model-system"
          name="gz::sim::systems::MulticopterMotorModel">
          <robotNamespace>model/{{ namespace }}</robotNamespace>
          <jointName>m2_joint</jointName>
          <linkName>m2</linkName>
          <turningDirection>ccw</turningDirection>
          <timeConstantUp>0.0125</timeConstantUp>
          <timeConstantDown>0.025</timeConstantDown>
          <maxRotVelocity>800.0</maxRotVelocity>
          <motorConstant>8.54858e-06</motorConstant>
          <momentConstant>0.016</momentConstant>
          <commandSubTopic>command/motor_speed</commandSubTopic>
          <motorNumber>1</motorNumber>
          <rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>
          <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
          <motorSpeedPubTopic>motor_speed/1</motorSpeedPubTopic>
          <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
          <motorType>velocity</motorType>
        </plugin>
        <plugin
          filename="ignition-gazebo-multicopter-motor-model-system"
          name="gz::sim::systems::MulticopterMotorModel">
          <robotNamespace>model/{{ namespace }}</robotNamespace>
          <jointName>m3_joint</jointName>
          <linkName>m3</linkName>
          <turningDirection>cw</turningDirection>
          <timeConstantUp>0.0125</timeConstantUp>
          <timeConstantDown>0.025</timeConstantDown>
          <maxRotVelocity>800.0</maxRotVelocity>
          <motorConstant>8.54858e-06</motorConstant>
          <momentConstant>0.016</momentConstant>
          <commandSubTopic>command/motor_speed</commandSubTopic>
          <motorNumber>2</motorNumber>
          <rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>
          <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
          <motorSpeedPubTopic>motor_speed/2</motorSpeedPubTopic>
          <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
          <motorType>velocity</motorType>
        </plugin>
        <plugin
          filename="ignition-gazebo-multicopter-motor-model-system"
          name="gz::sim::systems::MulticopterMotorModel">
          <robotNamespace>model/{{ namespace }}</robotNamespace>
          <jointName>m4_joint</jointName>
          <linkName>m4</linkName>
          <turningDirection>cw</turningDirection>
          <timeConstantUp>0.0125</timeConstantUp>
          <timeConstantDown>0.025</timeConstantDown>
          <maxRotVelocity>800.0</maxRotVelocity>
          <motorConstant>8.54858e-06</motorConstant>
          <momentConstant>0.016</momentConstant>
          <commandSubTopic>command/motor_speed</commandSubTopic>
          <motorNumber>3</motorNumber>
          <rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>
          <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
          <motorSpeedPubTopic>motor_speed/3</motorSpeedPubTopic>
          <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
          <motorType>velocity</motorType>
        </plugin>

        <plugin
          filename="ignition-gazebo-multicopter-control-system"
          name="gz::sim::systems::MulticopterVelocityControl">
          <robotNamespace>model/{{ namespace }}</robotNamespace>
          <commandSubTopic>cmd_vel</commandSubTopic>
          <enableSubTopic>velocity_controller/enable</enableSubTopic>
          <comLinkName>base_link</comLinkName>
          <velocityGain>3.3 3.3 3.3</velocityGain>
          <attitudeGain>1.25 1.65 0.9</attitudeGain>
          <angularRateGain>0.2 0.25 0.09</angularRateGain>
          <maximumLinearAcceleration>2 2 2</maximumLinearAcceleration>
          <maximumLinearVelocity>5 5 5</maximumLinearVelocity>
          <maximumAngularVelocity>3 3 3</maximumAngularVelocity>
          <linearVelocityNoiseMean>0 0 0.05</linearVelocityNoiseMean>
          <linearVelocityNoiseStdDev>0.1105 0.1261 0.00947</linearVelocityNoiseStdDev>
          <angularVelocityNoiseMean>0 0 0</angularVelocityNoiseMean>
          <angularVelocityNoiseStdDev>0.004 0.004 0.004</angularVelocityNoiseStdDev>

          <rotorConfiguration>
            <rotor>
              <jointName>m1_joint</jointName>
              <forceConstant>8.54858e-06</forceConstant>
              <momentConstant>0.016</momentConstant>
              <direction>1</direction>
            </rotor>
            <rotor>
              <jointName>m2_joint</jointName>
              <forceConstant>8.54858e-06</forceConstant>
              <momentConstant>0.016</momentConstant>
              <direction>1</direction>
            </rotor>
            <rotor>
              <jointName>m3_joint</jointName>
              <forceConstant>8.54858e-06</forceConstant>
              <momentConstant>0.016</momentConstant>
              <direction>-1</direction>
            </rotor>
            <rotor>
              <jointName>m4_joint</jointName>
              <forceConstant>8.54858e-06</forceConstant>
              <momentConstant>0.016</momentConstant>
              <direction>-1</direction>
            </rotor>
          </rotorConfiguration>
        </plugin>

        {% for sensor in sensors -%}
          <!-- Payload {{ sensor.model }} -->
            {% if sensor.model == 'gimbal_position' -%}

                {# Gimbal position - include or basic render  #}
                {% include 'models/gimbal/position_gimbal.sdf.jinja' with context %}

            {% elif sensor.model == 'gimbal_speed' -%}

                {# Gimbal speed - include or basic render  #}
                {% include 'models/gimbal/speed_gimbal.sdf.jinja' with context %}

            {% elif sensor.model == 'hd_camera' and not sensor.gimbaled -%}

                {% include 'models/hd_camera/hd_camera.sdf.jinja' with context %}

            {% elif sensor.model == 'vga_camera' and not sensor.gimbaled -%}

                {% include 'models/vga_camera/vga_camera.sdf.jinja' with context %}

            {% elif sensor.model == 'semantic_camera' and not sensor.gimbaled -%}

                {% include 'models/semantic_camera/semantic_camera.sdf.jinja' with context %}

            {% elif sensor.model == 'rgbd_camera' and not sensor.gimbaled -%}

                {% include 'models/rgbd_camera/rgbd_camera.sdf.jinja' with context %}

            {% elif sensor.gimbaled -%}

            {% else -%}
                <include>
                    <name>{{ sensor.name }}</name>
                    <uri>model://{{ sensor.model }}</uri>
                    <pose
                        relative_to="base_link">
                        {{ sensor.pose }}
                    </pose>
                </include>
                <joint
                    name="{{ sensor.name }}_joint" type="fixed">
                    <parent>base_link</parent>
                    <child>{{ sensor.name }}</child>
                </joint>
            {% endif -%}
        {% endfor -%}

        <plugin
          filename="ignition-gazebo-odometry-publisher-system"
          name="ignition::gazebo::systems::OdometryPublisher">
          <dimensions>3</dimensions>
          <odom_publish_frequency>100</odom_publish_frequency>
        </plugin>

        <plugin
            filename="ignition-gazebo-air-pressure-system"
            name="ignition::gazebo::systems::AirPressure">
        </plugin>
        <plugin
            filename="ignition-gazebo-imu-system"
            name="ignition::gazebo::systems::Imu">
        </plugin>
        <plugin
            filename="ignition-gazebo-magnetometer-system"
            name="ignition::gazebo::systems::Magnetometer">
        </plugin>

    </model>
  </sdf>

For the quadrotor, this template assumes your model has 4 links, one for each rotor, with the names m1, m2, m3, m4. This four links are required to have one joint each. In the example,
they are named m1_joint, m2_joint, m3_joint and m4_joint. These names can be changed to match your links names. The '_joint' is, however, necessary (if m1 is changed for YOUR_LINK, then its joint would have to be YOUR_LINK_joint).
In addition to this, the base link of your quadrotor model must be 'base_link'.

With this template, your model can be renderized to match all the expected naming by Aerostack2 and is ready to fly using the Gazebo Simulator.

Adding New Sensors
==================

New sensors to get information from your platform can be added to the Aerostack2 Gazebo simulation.

.. note:: The steps to add a new sensor are almost identical (steps 1 and 2) for any sensor of one of the types already included in Aerostack2, which are 'Camera', 'Depth Camera' and 'Lidar'. All of these are included in the gz-sensors library and publish similar Gazebo topics. Adding a different type of sensor or a sensor that uses a custom Gazebo plugin would be more complicated.


1. Create your sensor model
---------------------------

For a sensor model to work with Gazebo, your ``model.sdf`` must include a Gazebo plugin that publishes some information. This can be either an existing plugin in the ``gz-sensor`` library or a custom plugin.
In the case of sensors of type 'Camera' or 'Depth Camera', a ``model.sdf.jinja`` template is required for your model and its Gazebo and ROS 2 topics to be properly built. Here is the template of a camera for Gazebo:

.. code-block:: xml

  {% if sensor.model == 'gimbal_speed' or sensor.model == 'gimbal_position' -%}

      <model name='{{ sensor.sensor_attached }}'>

  {% else -%}

      <model name='{{ sensor.name }}'>
      <pose
          relative_to="base_link">
          {{ sensor.pose }}
      </pose>

  {% endif -%}

      <link name="hd_camera">
          <inertial>
              <mass>0.005</mass>
              <inertia>
                  <ixx>8.33e-06</ixx>
                  <ixy>0</ixy>
                  <ixz>0</ixz>
                  <iyy>8.33e-06</iyy>
                  <iyz>0</iyz>
                  <izz>8.33e-06</izz>
              </inertia>
          </inertial>
          <sensor name="camera" type="camera">
              <always_on>1</always_on>
              <update_rate>20</update_rate>
              <camera name="camera">
                  <horizontal_fov>1.0472</horizontal_fov>
                  <lens>
                      <intrinsics>
                          <!-- fx = fy = width / ( 2 * tan (hfov / 2 ) ) -->
                          <fx>1108.5</fx>
                          <fy>1108.5</fy>
                          <!-- cx = ( width + 1 ) / 2 -->
                          <cx>640.5</cx>
                          <!-- cy = ( height + 1 ) / 2 -->
                          <cy>480.5</cy>
                          <s>0</s>
                      </intrinsics>
                  </lens>
                  <image>
                      <width>1280</width>
                      <height>960</height>
                      <format>R8G8B8</format>
                  </image>
                  <clip>
                      <near>0.01</near>
                      <far>300</far>
                  </clip>
                  <noise>
                      <type>gaussian</type>
                      <mean>0</mean>
                      <stddev>0.007</stddev>
                  </noise>

                  {% if sensor.model == 'gimbal_speed' or sensor.model == 'gimbal_position' -%}

                      <optical_frame_id>/{{ namespace }}/{{ sensor.name }}/_0/_1/_2/{{ sensor.sensor_attached }}/{{ sensor.sensor_attached_type }}/camera/optical_frame</optical_frame_id>

                  {% else -%}

                      <optical_frame_id>/{{ namespace }}/{{ sensor.name }}/{{ sensor.model }}/camera/optical_frame</optical_frame_id>

                  {% endif -%}
                  
                  </camera>
              </sensor>
          </link>
  +
      <frame name="mount_point"/>
  </model>

  {% if sensor.model != 'gimbal_speed' and sensor.model != 'gimbal_position' -%}

  <joint
      name="{{ sensor.name }}_joint" type="fixed">
      <parent>base_link</parent>
      <child>{{ sensor.name }}</child>
  </joint>

  {% endif -%}

Parameters can be adjusted to the intrinsics of your own camera.

2. Add your sensor model
------------------------

Just like any other model, the sensor models can be either added to the ``as2_gazebo_assets/models`` folder or to the ``models`` folder in your project.

3. Add your sensor to a Payload class
-------------------------------------

In Aerostack2, sensors are added to drone models as a mounted payload. Sensors of the same type are grouped into a sensor type enumerator that  includes all the valid sensor types that have been implemented. The sensor type enumerator also lists the needed bridges for the sensor Gazebo data topics.

.. code-block:: python

  class CameraTypeEnum(str, Enum):
    """Valid camera model types."""

    VGA_CAM = 'vga_camera'
    HD_CAM = 'hd_camera'
    SEMANTIC_CAM = 'semantic_camera'

    @staticmethod
    def nodes(
        drone_model_name: str,
        sensor_model_name: str,
        sensor_model_type: str,
        gimbal_name: str,
        gimbaled: bool
    ) -> List[Node]:
        """
        Return custom bridges (nodes) needed for camera model.

        :param world_name: gz world name
        :param model_name: gz drone model name
        :param payload: gz payload (sensor) model type
        :param sensor_name: gz payload (sensor) model name
        :param model_prefix: ros model prefix, defaults to ''
        :return: list with bridges
        """
        nodes = [gz_custom_bridges.static_tf_node(
            drone_model_name, sensor_model_name, sensor_model_type, gimbal_name, gimbaled)
        ]
        return nodes

    @staticmethod
    def bridges(
        world_name: str,
        drone_model_name: str,
        sensor_model_name: str,
        sensor_model_type: str,
        sensor_model_prefix: str = '',
    ) -> List[Bridge]:
        """
        Return bridges needed for camera model.

        :param world_name: gz world name
        :param model_name: gz drone model name
        :param payload: gz payload (sensor) model type
        :param sensor_name: gz payload (sensor) model name
        :param model_prefix: ros model prefix, defaults to ''
        :return: list with bridges
        """
        bridges = [
            gz_bridges.image(world_name, drone_model_name, sensor_model_name,
                             sensor_model_type, sensor_model_prefix),
            gz_bridges.camera_info(world_name, drone_model_name, sensor_model_name,
                                   sensor_model_type, sensor_model_prefix)
        ]
        return bridges

Here, the ``camera`` sensor from the ``gz-sensor`` library publishes two Gazebo topics: one for the image and one for the camera info. Thus, two ``gz_bridges`` are created.

For Aerostack2 to load your sensor model and create the corresponding bridges, the name of your model must be added as a 'Valid camera model type' at the beginning of the class definition.

.. code-block:: python

  class CameraTypeEnum(str, Enum):
    """Valid camera model types."""

    VGA_CAM = 'vga_camera'
    HD_CAM = 'hd_camera'
    SEMANTIC_CAM = 'semantic_camera'
    YOUR_CAMERA = 'your_camera_model'

.. note:: If your are creating a new sensor of an already existing type, this is all you have to do to have your sensor working with Aerostack2. If your sensor is of a different type or is a custom one, continue with the following steps.

4. Adding a bridge for your sensor
----------------------------------

To use your sensor data in ROS 2 nodes, you need Gazebo-ROS 2 bridge. Aerostack2 includes a series of bridges for the already supported sensor types. If your sensor is of a different type, a new bridge is required.
The available bridges can be found in ``as2_simulation_assets/as2_gazebo_assets/src/as2_gazebo_assets_bridges``. There is a bridge for each of the Gazebo topics used by the implemented sensor types from the gz-sensor library.
This is the structure of a bridge in Aerostack2

.. code-block:: python

  def image(world_name, drone_model_name, sensor_model_name,
          sensor_model_type, sensor_model_prefix=''):
    """Image bridge."""
    sensor_prefix = prefix(world_name, drone_model_name, sensor_model_name, sensor_model_type)
    return Bridge(
        gz_topic=f'{sensor_prefix}/camera/image',
        ros_topic=f'sensor_measurements/{sensor_model_prefix}/image_raw',
        gz_type='ignition.msgs.Image',
        ros_type='sensor_msgs/msg/Image',
        direction=BridgeDirection.GZ_TO_ROS,
    )

This bridge consists in a node that subscribe to the specified gazebo topic, reformats the message into the ROS 2 message type, and publishes the information again. To know what ROS 2 type you need to connect your Gazebo message to, check
the `Gazebo Transport bridges list <https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge>`_. Make sure you modify the ``gz_topic`` to match the one to which your Gazebo sensor is publishing to, which should be defined in your custom plugin. 

You may not need the available world, drone and sensor names and types. Just remember they are available for you to construct the topics names, and that they are the names defined in the ``world.yaml`` file in which you setup the simulation configuration.

5. Creating a payload type enumerator
-------------------------------------

With your new bridge created, you can finally create the Payload 'TypeEnum' for your new sensor type. Following the structure presented in step 3

.. code-block:: python

  class YourTypeEnum(str, Enum):
    """Valid model types."""

    YOUR_SENSOR = 'your_sensor_model'

    @staticmethod
    def nodes(
        drone_model_name: str,
        sensor_model_name: str,
        sensor_model_type: str,
        gimbal_name: str,
        gimbaled: bool
    ) -> List[Node]:
        """
        Return custom bridges (nodes) needed for your model, only if needed.

        :param world_name: gz world name
        :param model_name: gz drone model name
        :param payload: gz payload (sensor) model type
        :param sensor_name: gz payload (sensor) model name
        :param model_prefix: ros model prefix, defaults to ''
        :return: list with bridges
        """
        nodes = [gz_custom_bridges.static_tf_node(
            drone_model_name, sensor_model_name, sensor_model_type, gimbal_name, gimbaled)
        ]
        return nodes

    @staticmethod
    def bridges(
        world_name: str,
        drone_model_name: str,
        sensor_model_name: str,
        sensor_model_type: str,
        sensor_model_prefix: str = '',
    ) -> List[Bridge]:
        """
        Return bridges needed for your model.

        :param world_name: gz world name
        :param model_name: gz drone model name
        :param payload: gz payload (sensor) model type
        :param sensor_name: gz payload (sensor) model name
        :param model_prefix: ros model prefix, defaults to ''
        :return: list with bridges
        """
        bridges = [
            gz_bridges.your_bridge(world_name, drone_model_name, sensor_model_name,
                             sensor_model_type, sensor_model_prefix),

        ]
        return bridges

With your sensor added as a valid type and the bridge (or bridges) created, your sensor is ready to work with the Aerostack2 Gazebo simulation.