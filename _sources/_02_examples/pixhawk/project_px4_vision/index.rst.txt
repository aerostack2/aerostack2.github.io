.. _project_px4_vision:

==================
PX4 Vision Example
==================

These examples use the :ref:`Pixhawk Platform <aerial_platform_px4>`. 
You can install it following the instructions in :ref:`installation section <aerial_platform_px4_installation>`. 

To install this project, please clone the repository:

.. code-block:: bash

   git clone https://github.com/aerostack2/project_px4_vision

To start using this project, please go to the root folder of the project.



.. _project_px4_vision_execution:

---------
Execution
---------

The flags for the components launcher are:

- ``-s``: launch the components for the simulated version.
- ``-g``: use GPS localization for outdoor flight.
- ``-t``: launch keyboard teleoperation.
- ``-r``: record rosbag.
- ``-n``: use custom dron namespace.

Arguments for the components launcher are:

- ``-e``: estimator type. Allowed values [``ground_truth``, ``raw_odometry``, ``mocap``]. Default: ``ground_truth``.



.. _project_px4_vision_execution_simulated:

Simulated execution
===================

In order to launch the components for the simulated version, you have to execute:

.. code-block:: bash

    ./launch_as2.bash -s -t

This will open a simulation for a single drone alongside the Aerostack2 components necessary for the mission execution.

A window like the following image should open.

.. figure:: images/single_drone_gz_classic.png
   :scale: 50
   :class: with-shadow
   
   Gazebo Classic simulator

It will also open a keyboard teleoperation (argument ``-t``), which you can use to teleoperate the drone with the :ref:`aerostack2 keyboard teleoperation user interface <user_interfaces_keyboard_teleoperation>`.

A window like the following image should popup:

.. figure:: images/keyboard_teleop_view.png
   :scale: 50
   :class: with-shadow
   
   Keyboard teleoperation

To start the mission, go to a new terminal line and execute:

.. code-block:: bash

    python3 mission.py -s

To do a clean exit of tmux, execute:

.. code-block:: bash

    ./stop.bash drone0



.. _project_px4_vision_execution_real:

Real execution
==============

Before launching the components, remember to set the correct state estimator to use. Currently, Aerostack2 supports two types of state estimators for the PX4, this are:

- **Optitrack**: which uses ``mocap`` plugin. 
- **Odometry**: which uses ``raw_odometry`` plugin.

Also, the second one have two versions, the one to indoor flights and the one to outdoor flights using GPS, that you can select with the flag ``-g``.

In order to launch the components, do:

- For the **odometmocapry** version, do:

.. code-block:: bash

    ./launch_as2.bash -e mocap -t

- For the **odometry** indoor version, do:

.. code-block:: bash

    ./launch_as2.bash -e raw_odometry -t

- For the **odometry** outdoor version, do:

.. code-block:: bash

    ./launch_as2.bash -e raw_odometry -g -t

.. note:: 

    Before launching the components with **mocap**, it is also necessary to set the file ``real_config/mocap.yaml``. This file will be used by the state estimator mocap plugin to 
    get the ground truth pose coming from our motion capture system into the Aerostack2 common interface localization :ref:`topics <ros2_common_interfaces_state_estimator_topics>`.


To start the mission, execute:

.. code-block:: bash

    python3 mission.py

To do a clean exit of tmux, execute the following command:

.. code-block:: bash

    ./stop.bash drone0


