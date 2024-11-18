.. _development_guide_create_assets:

------------------------
Create New Gazebo Assets
------------------------

.. contents:: Table of Contents
   :depth: 1
   :local:

.. _development_guide_create_assets_overview:

.. _development_guide_create_assets_add_platform:

Create a New Gazebo Aerial Platform
===================================

Adding new platform models so they can fly using Aerostack2 requires the creation of a Jinja template. Aerostack2 uses Jinja to templatize the Gazebo Platform models so that all the components,
links, joints and topics have the right names and the platform can be built properly.

Adding New Quadrotor Models
---------------------------

To add a new quadrotor model, like a BitCraze Crazyflie for instance, a file ``model-name.sdf.jinja`` must be added to your model folder, with the following content

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

With this and your model added to a directory that has been added to the Gazebo resources path variable (check :ref:`this section <development_tutorials_gazebo_assets_objects>` of the adding Gazebo assets tutorial), your new drone is ready to be loaded by Gazebo and fly using Aerostack2. Make sure you use the name of the model folder as the ``model_type`` field when configuring your new drone in your simulation configuration file.

Although it is not necessary, if you are building Aerostack2 from source, your model can also be added as a valid drone type at ``as2_gazebo_assets/src/as2_gazebo_assets/models/drone.py``. This script declares a class ``DroneTypeEnum`` to which your new drone model can be added.

.. code-block:: python

    class DroneTypeEnum(str, Enum):
        """Valid drone model types."""

        QUADROTOR = 'quadrotor_base'
        HEXROTOR = 'hexrotor_base'
        CRAZYFLIE = 'crazyflie'
        X500 = 'x500'
        PX4 = 'px4vision'
        YOUR_MODEL_TYPE = 'your_model_type'


.. _development_guide_create_assets_add_sensor:

Create a New Gazebo Sensor
==========================

1. Add Your New Sensor Model
----------------------------

For a sensor model to work with Gazebo, your ``model.sdf`` must include a Gazebo plugin that publishes some information. This can be either an existing plugin in the ``gz-sensor`` library or a custom plugin.
Here is an example of a sensor ``model.sdf`` file for a camera:

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

If you want your new type of sensor to be mounted on a Gimbal, a ``model.sdf.jinja`` template is required for your model and its Gazebo and ROS 2 topics to be properly built. Here is the template of a camera for Gazebo. Change the <link> tag name and the sensor name and type to the ones of your new sensor and its parameters:

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
      <frame name="mount_point"/>
  </model>

  {% if sensor.model != 'gimbal_speed' and sensor.model != 'gimbal_position' -%}

  <joint
      name="{{ sensor.name }}_joint" type="fixed">
      <parent>base_link</parent>
      <child>{{ sensor.name }}</child>
  </joint>

  {% endif -%}


Just like any other model, the sensor models can be either added to the ``as2_gazebo_assets/models`` folder or to the ``models`` folder in your project.

2. Adding a bridge for your sensor
----------------------------------

To use your sensor data in ROS 2 nodes, you need Gazebo-ROS 2 bridge. Aerostack2 includes a series of bridges for the already supported sensor types. If your sensor is of a different type, a new bridge is required.
The available bridges can be found in ``as2_simulation_assets/as2_gazebo_assets/src/as2_gazebo_assets_bridges``. There is a bridge for each of the Gazebo topics used by the implemented sensor types from the gz-sensor library.
This is the structure of a bridge in Aerostack2

.. code-block:: python

    def new_image(world_name, drone_model_name, sensor_model_name,
                sensor_model_type, sensor_model_prefix=''):
        """New Image bridge"""
        sensor_prefix = prefix(world_name, drone_model_name, sensor_model_name, sensor_model_type)
        return Bridge(
            gz_topic=f'{sensor_prefix}/camera/image',
            ros_topic=f'sensor_measurements/{sensor_model_prefix}/alternative_topic',
            gz_type='ignition.msgs.Image',
            ros_type='sensor_msgs/msg/Image',
            direction=BridgeDirection.GZ_TO_ROS,
        )

This bridge consists in a node that subscribe to the specified gazebo topic, reformats the message into the ROS 2 message type, and publishes the information again. To know what ROS 2 type you need to connect your Gazebo message to, check
the `Gazebo Transport bridges list <https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge>`_. Make sure you modify the ``gz_topic`` to match the one to which your Gazebo sensor is publishing to, which should be defined in your custom plugin. 

It is important that you use the ``prefix()`` function that formats the core of the topics naming. By using it, you make sure Aerostack2 can find and subscribe and publish to the right topics.

3. Creating a payload type enumerator
-------------------------------------

With your new bridge created, you can finally create the Payload 'TypeEnum' for your new sensor type. Following the structure presented in step 3

.. code-block:: python

    class NewCameraTypeEnum(str, Enum):
        """Valid camera model types."""

        NEW_CAMERA = 'new_camera'

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
                gz_bridges.new_image(world_name, drone_model_name, sensor_model_name,
                                    sensor_model_type, sensor_model_prefix),
                gz_bridges.camera_info(world_name, drone_model_name, sensor_model_name,
                                    sensor_model_type, sensor_model_prefix)
            ]
            return bridges

With your sensor added as a valid type and the bridge (or bridges) created, your sensor is ready to work with the Aerostack2 Gazebo simulation.

4. Register your new type enumerator
------------------------------------

Finally, add the name of your new type enumerator class to the ``Union[]`` memeber of the general Payload class so that Aerostack2 can recognize your sensor type as a valid Payload for the drones.

.. code-block:: python

    class Payload(Entity):
        """
        Gz Payload Entity.

        Use model_type as sensor_type
        """

        model_type: Union[
            CameraTypeEnum, DepthCameraTypeEnum, LidarTypeEnum, GpsTypeEnum, GimbalTypeEnum, NewCameraTypeEnum
        ] = None
        sensor_attached: str = 'None'
        sensor_attached_type: str = 'None'
        payload: Payload = None
        gimbaled: bool = False
        gimbal_name: str = 'None'

        ...

