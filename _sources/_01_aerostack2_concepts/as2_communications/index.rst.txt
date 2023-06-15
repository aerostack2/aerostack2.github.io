.. _as2_concepts_as2_communication:

======================================
Aerostack2 Inter-process Communication
======================================

In the figure below, the communication channels are shown.
In the robot control loop, four modules can be distinguished: the platform, the motion controller, the state estimator and the external agents.
For communication between them, the motion reference, actuator commands, sensor measurement and self localization channels are used.
In addition, there is an extra channel that communicates all the modules of the robot, which is the emergency channel.

.. figure:: ../figures/as2_robot_communications.png
   :scale: 90
   
   Aerostack2 Inter-process Communication Diagram

The objective of each channel is:

* **Motion Reference**: This channel is used to send the motion reference to the motion controller.
* **Actuator Commands**: This channel is used to send the actuator commands to the platform from the motion controller.
* **Sensor Measurements**: This channel is used to send the sensor measurements to the state estimator.
* **Self Localization**: This channel is used to send the robot localization from the state estimator.
* **Emergency**: This channel is used to send the emergency messages to all the modules of the robot.

All of them use `ROS 2 topics <https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html>`__ to communicate in a standard way.
However, Aerostack2 use `ROS 2 Tf2 Library <https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Tf2-Main.html>`__ to set topics frame id.
This feature allow to stablish messages reference frame and each agent of Aerostack2 can use it to convert the messages to the desired frame.



.. _as2_concepts_as2_communication_motion_references:

-----------------
Motion References
-----------------

Aerostack2 supports three different types of motion commands: **position**, **velocity** and **trajectory**. 
With them, each controller mode information is sent to the motion controller.
They are send with a topic structure: `/drone_namespace/motion_reference/reference_type`.

* **Position Control Mode**: The desired position reference is send in the position command. It use the velocity command to set the speed limit.
* **Speed Control Mode**: The desired speed reference is send in the velocity command.
* **Speed in a Plane Control Mode**: The desired speed reference is send in the velocity command and the desired height is send in the position command.
* **Trajectory Control Mode**: The desired trajectory is send in the trajectory command.
* **Yaw Angle Control Mode**: The desired yaw angle is send in the orientation of the position command.
* **Yaw Speed Control Mode**: The desired yaw speed is send in the angular velocity z of the velocity command.

Each motion command must have a frame id and a timestamp. 
Motion Controller use this information to convert from the command frame to the controller desired frame.



.. _as2_concepts_as2_communication_actuator_commands:

-----------------
Actuator Commands
-----------------

Aerostack2 supports three different types of actuator commands: **position**, **velocity** and **thrust**. 
With them, each controller mode information is sent to the platform.
They are send with a topic structure: `/drone_namespace/actuator_commands/actuator_type`.

* **Position Control Mode**: The desired position reference is send in the position command.
* **Speed Control Mode**: The desired speed reference is send in the velocity command.
* **Speed in a Plane Control Mode**: The desired speed reference is send in the velocity command and the desired height is send in the position command.
* **Trajectory Control Mode**: The desired trajectory is send in the position and velocity commands.
* **ATTITUDE Control Mode**: The desired attitude is send in the orientation of the position command, and the thrust in the thrust command.
* **ACRO Control Mode**: The desired attitude rate is send in the angular velocity of the velocity command, and the thrust in the thrust command.
* **Yaw Angle Control Mode**: The desired yaw angle is send in the orientation of the position command.
* **Yaw Speed Control Mode**: The desired yaw speed is send in the angular z of the velocity command.



.. _as2_concepts_as2_communication_sensor_measurements:

-------------------
Sensor Measurements
-------------------

Aerostack2 have a standard way to publish sensor measurements, using topic namespaces. The structure is: `/drone_namespace/sensor_measurements/sensor_name`. 

State Estimator use sensor measurements to fuse them and estimate the robot state. 
Also, external devices such as a motion capture system use this channel to send the robot state to the State Estimator.



.. _as2_concepts_as2_communication_self_localization:

-----------------
Self Localization
-----------------

Aerostack2 have a standard way to publish self localization, given the robot pose and twist. The structure is: `/drone_namespace/self_localization/pose` and `/drone_namespace/self_localization/twist`.

However, the recommended way to get the robot pose is using `ROS 2 Tf2 Library <https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Tf2-Main.html>`__.



.. _as2_concepts_as2_communication_emergency:

---------
Emergency
---------

Aerostack2 have a open channel to publish alerts and warnings. 
All agents of the robot can publish in this channel, and each one must know how to react to each message.
For that, a severity level is used to indicate the importance of the message and which agent must react to it.









