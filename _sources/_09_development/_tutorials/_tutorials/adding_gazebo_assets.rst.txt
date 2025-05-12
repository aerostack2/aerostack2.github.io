.. _development_tutorials_gazebo_assets:

========================
Adding new Gazebo Assets
========================

.. contents:: Table of Contents
   :depth: 1
   :local:

.. _development_tutorials_gazebo_assets_overview:

--------
Overview
--------

New worlds and models can be added and configured to work with Aerostack2 in Gazebo simulations.

.. _development_tutorials_gazebo_assets_worlds:

-----------------
Adding New Worlds
-----------------

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

This template includes the necessary physics, lights and sensor plugins for the simulation to propperly run using your world model.

Worlds can also be added from an external source. For Gazebo to find the world file when the simulation is launched, make sure you have your Gazebo sources environment variable
set to look for world files in your external folder. The content of this variable can be checked by running

.. code-block:: bash

  echo $GZ_SIM_RESOURCE_PATH

Add the path to your external ``worlds`` folder using

.. code-block:: bash

  export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:<path/to/directory>/worlds

.. note:: If your are adding your worlds and models from a project which follows the project structure like this `project_gazebo <https://github.com/aerostack2/project_gazebo>`__. example, these environment variables can be set from your launch .bash file so that Gazebo can always find your models. To do so, add the previous ``export`` command to the launch .bash file before the Gazebo simulation launching commands.

.. _development_tutorials_gazebo_assets_objects:

-----------------------------
Adding New Models and Objects
-----------------------------

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

-----------------------
Configuring New Sensors
-----------------------

New sensors to get information from your platform can be added to the Aerostack2 Gazebo simulation.

To configure a new sensor of any of the already supported types in Aerostack2, a sensor model needs to be added following the previous step of this guide. This model must include one of the sensor plugins from the ``gz-sensor`` library that is supported by Aerostack2. These sensor types are:

.. list-table:: Supported Gazebo Sensor Types
   :widths: 50 50 50
   :header-rows: 1

   * - Sensor Type
     - Gazebo Sensor Plugin
     - Existing Sensors
   * - Camera
     - "camera"
     - hd_camera, vga_camera, semantic_camera
   * - Depth Camera
     - "rgbd_camera"
     - rgbd_camera
   * - Lidar
     - "gpu_ray"
     - point_lidar, planar_lidar, lidar_3d

For any application in which you may need a specific camera model with its own instrinsic parameters, you can add it by creating a model that includes the "camera" gazebo sensor plugin with the right
configuration. Such model would look like this example:

.. code-block:: xml

  <?xml version="1.0"?>
  <sdf version="1.9">
      <model name="new_camera">
          <link name="new_camera">
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
                  </camera>
              </sensor>
          </link>
  +
          <frame name="mount_point"/>
      </model>
  </sdf>

Since this new model uses the same sensor type as the rest of the supported cameras, and thus, publishes information in the same expected topics, you would only have to add this new sensor to the corresponding sensor enumerator type in the ``payload.py`` file
at ``as2_simulation_assets/as2_gazebo_assets/src/as2_gazebo_assets/models``:

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

In Aerostack2, sensors are added to drone models as a mounted payload. Sensors of the same type are grouped into a sensor type enumerator that  includes all the valid sensor types that have been implemented. The sensor type enumerator also lists the needed bridges for the sensor Gazebo data topics.

For Aerostack2 to load your sensor model and create the corresponding bridges, the name of your model must be added as a 'Valid camera model type' at the beginning of the class definition.

.. code-block:: python

  class CameraTypeEnum(str, Enum):
    """Valid camera model types."""

    VGA_CAM = 'vga_camera'
    HD_CAM = 'hd_camera'
    SEMANTIC_CAM = 'semantic_camera'
    YOUR_CAMERA = 'your_camera_model'

.. note:: If your are creating a new sensor of an already existing type, this is all you have to do to have your sensor working with Aerostack2. If your sensor is of a different type or is a custom one, check the Develop Guide section for :ref:`Creating New Gazebo Assets <aerial_platform_gazebo>`.

Mounting sensors on Gimbal
--------------------------

Aerostack2 sensors can be mounted on a Gimbal. Configuring your new sensor to be mountable on Gimbal, a Jinja template is required for the new model.
The template renders all the model components on Gazebo so that all topics are propperly formatted and the sensor information is available. For the same "camera" Gazebo sensor used in the previous example, the ``model.sdf.jinja`` file for your sensor would look like the following:

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

Using this template, the name of the model for Gazebo as well as some other information is retrieved from the simulation configuration file, so that you only have to adjust the camera parameters. Parameters can be adjusted to the intrinsics of your own camera.

Just like any other model, the sensor models can be either added to the ``as2_gazebo_assets/models`` folder or to the ``models`` folder in your project. The ``model.sdf.jinja`` file replaces the usual ``model.sdf`` file, although both files can coexist.

To mount it on Gimbal, your new sensor needs a Jinja template to be loaded, you need to add it to the list of sensors with Jinja template of the drone model you are goint to use.
This list is on the ``drone-model.sdf.jinja`` file, and consists on a series of ``if-else`` that checks the name of the sensor in case it has a Jinja template to be loaded.
On the default ``quadrotor-base`` model, the list looks like this:

.. code-block:: sdf

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

so before the line ``{% elif sensor.gimbaled -%}``, add the following lines:

.. code-block:: sdf

    {% elif sensor.model == 'your-sensor' and not sensor.gimbaled -%}
        {% include 'models/your-sensor/your-sensor.sdf.jinja' with context %}

If this lines are not added, the default ``your-sensor.sdf`` model will be loaded.


