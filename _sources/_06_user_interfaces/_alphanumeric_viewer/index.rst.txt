.. _user_interfaces_alphanumeric_viewer:

===================
Alphanumeric Viewer
===================

.. contents:: Table of Contents
   :depth: 3
   :local:

------------
Introduction
------------

Aerostack2’s alphanumeric viewer interface consists on a window with multiple tabs that shows different data regarding the state 
of the uav. 

----------------
Interface launch
----------------

Example of keyboard teleoperation tool launch command:

.. code-block:: bash

    ros2 run as2_alphanumeric_viewer as2_alphanumeric_viewer_node \
    --ros-args -r  __ns:=/drone_sim_0

This will open alphanumeric viewer interface for drone with namespace: ``drone_sim_0``.

------------
Instructions
------------

When the previous launch command is executed, a window like the following will open:

.. figure:: images/summary_window.png
   :scale: 100
   :class: with-shadow
   
   Alphanumeric viewer summary window.

Different tabs can be displayed when pressing the correspondent key. Each of the tabs shows different information for different purposes. 

.. figure:: images/sensor_window.png
   :scale: 100
   :class: with-shadow
   
   Alphanumeric viewer sensors window.

.. figure:: images/navigation_window.png
   :scale: 100
   :class: with-shadow
   
   Alphanumeric viewer navigation window.

.. figure:: images/platform_window.png
   :scale: 100
   :class: with-shadow
   
   Alphanumeric viewer platform window.

Those tabs can be summarized in the following table:

.. list-table:: List of alphanumeric viewer tabs
   :widths: 50 50 50
   :header-rows: 1

   * - Key
     - Name
     - Information displayed
   * - M-m
     - Summary tab
     - | Shows all different but what we believe is the most important information to show
       | most of the times. It displays information about the IMU measurements, localization, 
       | control modes for each the controller and the platform, aswell as the platform status.
   * - S-s
     - Sensors tab
     - Shows information regarding the sensor measurements with a clearer layout.
   * - N-n
     - Navigation tab
     - Shows information regarding self localization with a clearer layout.
   * - P-p
     - Platform tab
     - Shows information regarding platform and controller with a clearer layout.