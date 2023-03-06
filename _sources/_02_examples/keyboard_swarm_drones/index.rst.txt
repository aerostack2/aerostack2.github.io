.. _examples_swarm_keyboard_example:

====================================
Swarm Keyboard Teleoperation Example
====================================

In this example, a swarm of three drones is simulated in Ignition Gazebo and user can teleoperate them with the keyboard.

* Run Ignition simulator. The simulation enviroment can be modified in the file ``simulation_config/swarm.json``.

.. code-block:: bash

    ./launch_ignition_swarm.bash

A window like the following image should open.

.. figure:: images/swarm_ign.png
   :scale: 50
   :class: with-shadow
   
   Ignition Gazebo simulator

* In a new terminal, run Aerostack2 nodes:

.. code-block:: bash

    ./launch_swarm.bash

* In a new terminal, run keyboard teleoperation node:

.. code-block:: bash

    ./launch_swarm_keyboard.bash

A window like the following image should popup, where you can teleoperate each drone with the :ref:`aerostack2 keyboard teleopration user interface <keyboard_teleoperation>`.

.. figure:: images/keyboard_teleop_swarm.png
   :scale: 50
   :class: with-shadow
   
   Keyboard teleoperation

* In tmux, you can navigate through the different sessions and windows and see each ROS2 node running.

