.. _as2_concepts_motion_controller:

=================
Motion Controller
=================

The Motion Controller is the component of Aerostack2 that is responsible for the control of the robot actuators. 
It is in charge of convert :ref:`Motion References <as2_concepts_as2_communication_motion_references>` from the different modules of Aerostack2 and sending the corresponding :ref:`Actuator Commands <as2_concepts_as2_communication_actuator_commands>` to the Aerial Platform.

.. figure:: ../figures/as2_robot_communications.png
   :scale: 90



.. _as2_concepts_motion_reference_handler:

------------------------
Motion Reference Handler
------------------------

Motion Reference Handler is the component of Aerostack2 that is responsible for the generation of the :ref:`Motion References <as2_concepts_as2_communication_motion_references>` that are sent to the Motion Controller.

This utility allows users to encode the desired motion reference of the robot into the three motion reference topics. Also, it manage the different control modes of the Motion Controller.

The Motion Controller will convert the Motion Reference from the input frame to the desired one in the time specified by the reference.



.. _as2_concepts_motion_controller_control_modes:

-------------
Control Modes
-------------

The Motion Controller will convert the Motion Reference into Actuator Commands using the Input Control Mode and Output Control Mode to know how to interpret the Motion Reference.

The input control mode is set by the Motion Reference Handler and the output control mode is set automatically by communicating with the Aerial Platform.




.. _as2_concepts_motion_controller_control_modes_input:

Input Control Modes
===================

Aerostack2 Motion Controller supports the following input control modes:

* **Hover Control**: The robot is commanded to stay in the same position.
* **Position Control**: The robot is commanded to reach a desired 3D position in the desired frame.
* **Speed Control**: The robot is commanded to reach a desired 3D speed in the desired frame.
* **Speed in a Plane Control**: The robot is commanded to reach a desired 2D plane speed in the desired frame with the desired height.
* **Trajectory Control**: The robot is commanded to follow a desired 3D trajectory in the desired frame.

Each one are combined with the desired yaw angle control mode:

* **Yaw Angle Control**: The robot is commanded to reach a desired yaw angle.
* **Yaw Rate Control**: The robot is commanded to reach a desired yaw rate.

Also, the frame of each command is specified in the header of the message, allowing the use of any reference frame.


.. _as2_concepts_motion_controller_control_modes_output:

Output Control Modes
===================

Aerostack2 Motion Controller supports the following ouput control modes:

* **Hover Control**: The robot is commanded to stay in the same position.
* **Position Control**: The robot is commanded to reach a desired 3D position in the desired frame.
* **Speed Control**: The robot is commanded to reach a desired 3D speed in the desired frame.
* **Speed in a Plane Control**: The robot is commanded to reach a desired 2D plane speed in the desired frame with the desired height.
* **Trajectory Control**: The robot is commanded to follow a desired 3D trajectory in the desired frame.
* **Attitude Control**: The robot is commanded to reach a desired attitude angles and thrust in the base link frame.
* **Acro Control**: The robot is commanded to reach a desired angular velocity and thrust in the base link frame.

Except Acro Control, the rest of them are combined with the desired yaw angle control mode:

* **Yaw Angle Control**: The robot is commanded to reach a desired yaw angle.
* **Yaw Rate Control**: The robot is commanded to reach a desired yaw rate.

Also, the frame of each command is specified the following control reference frames:

* **ENU Frame**: The frame is fixed to the Earth frame and its origin is set with the State Estimator. The X axis points to the East, the Y axis points to the North and the Z axis points to the Up.
* **FLU Frame**: The frame is fixed to the Aircraft base link frame. The X axis points to the Front, the Y axis points to the Left and the Z axis points to the Up.
