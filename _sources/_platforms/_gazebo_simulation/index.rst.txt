Ignition Gazebo
===============

For simulation purposes with Ignition Gazebo simulator, Aerostack2 provides with a platform that serves as an entry point for aerial robotics simulated in this environment.

* **Supported Control Modes:**

  - Speed
  - Hover

Aerial robots works in local FLU reference frame. This means that Platform Control Mode should be set in ``BODY_FLU_FRAME``.

No offboard mode is needed, and due to the particularity of this platform beeing fully simulated, this component provides custom ``take off`` and ``land``.

Ros-Gazebo Bridge
#################

Gazebo defines it's own messages in a ROS2 decoupled communication protocol, for instance, in order to get this data to work in ROS2 network, we need to use a intermediate
bridge that publish this data into ROS2. 

Library `ros_gz_bridge <https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge>`_ provides a network bridge which enables the exchange of messages between ROS2 and Gazebo Transport. Messages in Gazebo Transport are published by plugins. As Ignition Gazebo's 
architecture is based on an ECS, plugins provides the functionality for each component, like sensors.
Its support is limited to only certain message types. Aerostack2 uses this component's functionality to provide message conversion. 
The ign messages regarding the robot model that are passed to ROS2 are included in the following table:

.. list-table:: MODEL BRIDGE IGN-ROS
   :widths: 50 50 50 50
   :header-rows: 1

   * - Ign Topic
     - ROS Topic
     - Ign message type
     - ROS message type
   * - /clock
     - /clock
     - ignition.msgs.Clock
     - rosgraph_msgs/msg/Clock
   * - /model/{model_name}/odometry
     - {namespace}/ground_truth/pose
     - ignition.msgs.Odometry
     - geometry_msgs/msg/PoseStamped
   * - /model/{model_name}/odometry
     - {namespace}/ground_truth/speed
     - ignition.msgs.Odometry
     - geometry_msgs/msg/TwistStamped

To communicate ROS2 commands with Ignition Gazebo, ros_gz_bridge can work the other way around, translating messages from ROS2 to Ignition Gazebo 
so we can arm the aerial robot and send speed commands from the controller. This is a unique characteristic of this platform as it is only simulated,
ROS2 communication tools are used to send speed commands: 

.. list-table:: MODEL BRIDGE ROS-IGN
   :widths: 50 50 50 50
   :header-rows: 1

   * - Ign Topic
     - ROS Topic
     - Ign message type
     - ROS message type
   * - /model/{model_name}/velocity_controller/enable
     - /ign/{model_name}/arm
     - ignition.msgs.Boolean
     - std_msgs/msg/Bool
   * - /model/{model_name}/cmd_vel
     - /ign/{model_name}/cmd_vel
     - ignition.msgs.Twist
     - geometry_msgs/msg/Twist

Sensors
#######

Aerostack2 is provided with different types of sensors from `Ignition Gazebo sensor library <https://github.com/gazebosim/gz-sensors>`_. These
sensors can be attached to models in order to simulate the physical model of the sensor in the world and its functionality.
To get the information coming from these sensor's Gazebo plugins, Aerostack2 brings these messages to ROS2 like the following table indicates:

.. list-table:: SENSOR BRIDGE IGN-ROS
   :widths: 50 50 50 50 50
   :header-rows: 1

   * - Sensor
     - Ign Topic
     - ROS Topic
     - Ign message type
     - ROS message type
   * - IMU
     - {sensor_prefix}/imu/imu
     - {namespace}/sensor_measurements/imu
     - ignition.msgs.IMU
     - sensor_msgs/msg/Imu
   * - Magnetometer
     - {sensor_prefix}/magnetometer/magnetometer
     - {namespace}/sensor_measurements/magnetic_field
     - ignition.msgs.Magnetometer
     - sensor_msgs/msg/MagneticField
   * - Air pressure
     - {sensor_prefix}/air_pressure/air_pressure
     - {namespace}/sensor_measurements/air_pressure
     - ignition.msgs.FluidPressure
     - sensor_msgs/msg/FluidPressure
   * - Battery
     - /model/{model_name}/battery/linear_battery/state
     - {namespace}/sensor_measurements/battery
     - ignition.msgs.BatteryState
     - sensor_msgs/msg/BatteryState
   * - Camera
     - {sensor_prefix}/camera/image
     - {namespace}/sensor_measurements/{model_prefix}/image_raw
     - ignition.msgs.Image
     - sensor_msgs/msg/Image
   * - Depth camera
     - {sensor_prefix}/camera/depth_image
     - {namespace}/sensor_measurements/{model_prefix}/depth
     - ignition.msgs.Image
     - sensor_msgs/msg/Image
   * - Camera
     - {sensor_prefix}/camera/camera_info
     - {namespace}/sensor_measurements/{model_prefix}/camera_info
     - ignition.msgs.CameraInfo
     - sensor_msgs/msg/CameraInfo
   * - Lidar
     - {sensor_prefix}/gpu_ray/scan
     - {namespace}/sensor_measurements/{model_prefix}/scan
     - ignition.msgs.LaserScan
     - sensor_msgs/msg/LaserScan
   * - Lidar
     - {sensor_prefix}/gpu_ray/scan/points
     - {namespace}/sensor_measurements/{model_prefix}/points
     - ignition.msgs.PointCloudPacked
     - sensor_msgs/msg/PointCloud2
   * - Camera
     - {sensor_prefix}/camera/points
     - {namespace}/sensor_measurements/{model_prefix}/points
     - ignition.msgs.PointCloudPacked
     - sensor_msgs/msg/PointCloud2

Aerial models
#############

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

In order to add an aerial model with sensors attached to it to the simulated world, modify or create a new configuration file in folder ``simulation_config`` in ``.json`` format with the following:

.. code-block:: json

  {
    "world": "empty",
    "drones": [
        {
            "model": "quadrotor_base",
            "name": "drone_sim_0",
            "xyz": [ 1.0, 1.0, 0.3 ],
            "rpy": [ 0.0, 0.0, 0.0 ],
            "sensors": {
              "gps": {
                "sensor": "gps"
              }
            }
        }
    ]
  }

Where:

* ``world``: name of the defined world in sdf format.
* ``drones``: list of drones to be included in the world.
  
Each of the ``drones`` is defined by:

* ``model``: model of the drone defined in sdf format.
* ``name``: namespace
* ``xyz``: spawn position
* ``rpy``: spawn orientation 
* ``sensors``: list of sensors attached to the model

Each of the ``sensors`` is defined by:

* ``type``: name of the sensor inside de simulation (this case ``gps``)
* ``sensor``: name of the sensor defined in sdf format.

New models, sensors and worlds are defined in the ``as2_ignition_assets`` package. For more information on how to create new assets, go to the `Ignition Fortress tutorial page <https://gazebosim.org/docs/fortress/tutorials>`_.