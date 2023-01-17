.. Aerostack2 documentation master file, created by
   sphinx-quickstart on Thu Dec  1 13:31:26 2022.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to AeroStack2's documentation!
======================================

Overview
########

AeroStack2 is a software framework that helps developers design and build the control architecture of aerial robotic systems, integrating multiple heterogeneous computational solutions (e.g., computer vision algorithms, motion controllers, self-localization and mapping methods, motion planning algorithms, etc.), built for ROS2 `Humble <https://docs.ros.org/en/humble/index.html>`_ and ROS2 `Galactic <https://docs.ros.org/en/galactic/index.html>`_.

AeroStack2 is useful for building autonomous aerial systems in complex and dynamic environments and it is also a useful research tool for aerial robotics to test new algorithms and architectures.

It was created to be available for communities of researchers and developers and it is currently an active open-source project with periodic software releases. 

AeroStack2 is versatile for building different system configurations with various degrees of autonomy:

* From teleoperation to autonomous flight. AeroStack2 can be used for teleoperation flights (with manual control) but it can also be used for building autonomous robot systems to perform aerial missions without operator assistance.

* Single robots or multi-robot systems. AeroStack2 can be used to fly a swarm of heterogeneous drones to perform multi-aerial-robot missions. It has been validated to operate with up to five drones simultaneously.

* Flexible for different applications. AeroStack2 can be used by system designers to develop their own systems in a wide range of applications. Aerostack provides languages and graphical tools to configure specific aerial missions.

* Hardware independent. AeroStack2 runs on conventional laptops and it has also run on onboard computers like mastermind or odroid XU3. AeroStack2 has been used in different aerial platforms, including, but not limited to: DJI platforms (Matrice 210, Matrice 100, Ryze Tello), Parrot platforms (Bebop 2, AR Drone 1.0 & 2.0), Pixhawk, Mikrokopter Oktokopter and Asctec Pelican.

Most important features:

* Natively developed on ROS2
* Complete modularity, allowing elements to be changed or interchanged without affecting the rest of the system
* Independence of the aerial platform.
* Project-oriented, allowing to install and use only the necessary packages for the application to be developed. 
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
   _px4_vision/index.rst
   
