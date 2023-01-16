Aerostack2 Concepts
===================

In this section we will review high level concepts that Aerostack2 introduces, from the architecture to its approach to attack the problematic of developing and 
deploy aerial robotics systems so the user can familiarize with them.

.. _as2_architecture:

Architecture
############

TBD (new architecture figure)

Platform
########

.. _as2_controller:

Controller
##########

.. _as2_estimator:

State Estimator
###############

Robot behaviors
###############

Aerostack provides a software library to implement robot behaviors that can be used by developers to build a particular robotic system architecture. 
Examples of these components are:

* Feature extractors that read simple states of sensors or implement vision recognition algorithms,

* Motion controllers (e.g., PID controllers or MPC controllers),

* Behaviors that perform self-localization and mapping (SLAM),

* Motion planners that generate obstacle-free paths to reach destination points, and

* Methods for communicating with other agents (other robots or human operators).

The library is open, so new behaviors can be included easily in the future, without changing the core of the system and the rest of the behaviors.

Each behavior in Aerostack is characterized by the following properties:

* Common communication. Each behavior sends and/or receive messages using the common communication channel used by the Aerostack2 architecture. 
  If several behaviors publish data in the same ROS topic (e.g., different motion controllers), additional mechanisms must be used to avoid conflicts.

* Uniform interface. Each behavior has a uniform interface to control its execution (e.g., activate and deactivate). 
  Each behavior provides cognizant failure, i.e., it verifies the correct execution with a reflective process that verifies whether 
  the task is done correctly or not to inform about success or failure.

