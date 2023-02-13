.. Aerostack2 documentation master file, created by
   sphinx-quickstart on Thu Dec  1 13:31:26 2022.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to Aerostack2's documentation!
======================================

Overview
########

Aerostack2 is a software framework that helps developers design and build the control architecture of aerial robotic systems, integrating multiple heterogeneous computational solutions (e.g., computer vision algorithms, motion controllers, self-localization and mapping methods, motion planning algorithms, etc.), built for ROS2 `Humble <https://docs.ros.org/en/humble/index.html>`_ and ROS2 `Galactic <https://docs.ros.org/en/galactic/index.html>`_.

Aerostack2 is useful for building autonomous aerial systems in complex and dynamic environments and it is also a useful research tool for aerial robotics to test new algorithms and architectures.

It was created to be available for communities of researchers and developers and it is currently an active open-source project with periodic software releases. 

Aerostack2 is versatile for building different system configurations with various degrees of autonomy:

* From teleoperation to autonomous flight. Aerostack2 can be used for teleoperation flights (with manual control) but it can also be used for building autonomous robot systems to perform aerial missions without operator assistance.

* Single robots or multi-robot systems. Aerostack2 can be used to fly a swarm of heterogeneous drones to perform multi-aerial-robot missions. It has been validated to operate with up to five drones simultaneously.

* Flexible for different applications. Aerostack2 can be used by system designers to develop their own systems in a wide range of applications. Aerostack2 provides languages and graphical tools to configure specific aerial missions.

* Hardware independent. Aerostack2 runs on conventional laptops and it has also run on onboard computers like Nvidia Jetson NX. Aerostack2 has been used in different aerial platforms, including, but not limited to: DJI platforms (Matrice 210RTKv2, Matrice 300, Ryze Tello), Pixhawk autopilots and Crazyflie drones. The framework can operate in simulation and in a real environment in a similar way, what simplifies the Sim2Real development.

Most important features:

* Natively developed on ROS2
* Complete modularity, allowing elements to be changed or interchanged without affecting the rest of the system. Plugin-based architecture allows to use different implementations for every tasks.
* Independence of the aerial platform.
* Project-oriented, allowing to install and use only the necessary packages and configurations for the application to be developed. 
* Swarming oriented operation.

.. toctree::
   :hidden:

   _getting_started/index.rst
   _aerostack2_concepts/index.rst
   _tutorials/index.rst
   _cli/index.rst
   _behaviors/index.rst
   _platforms/index.rst
   _ros_common_interfaces/index.rst
   _user_interfaces/index.rst
   _python_api/index.rst
   _core/index.rst
   _developer/index.rst
   _projects/index.rst
   _license/index.rst
   _troubleshooting/index.rst
   
