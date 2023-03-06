.. _examples_swarm_python_example:

====================
Swarm Python Example
====================

In this example, a swarm of three drones is simulated in Ignition Gazebo and user can define an autonomous mission using the :ref:`Aerostack2 Python API <python_api>`.

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


* Optionally, in a new terminal, you can run keyboard teleoperation node for supervise and modify the mission:

.. code-block:: bash

    ./launch_swarm_keyboard.bash

A window like the following image should popup, where you can supervise and modify the mission with the Behavior control tab of the :ref:`aerostack2 keyboard teleopration user interface <keyboard_teleoperation>`.

.. figure:: images/keyboard_swarm_view.png
   :scale: 50
   :class: with-shadow
   
   Keyboard teleoperation

* Run python mission, where a mission is defined using the :ref:`Aerostack2 Python API <python_api>`.

.. code-block:: bash

    python3 mission_swarm.py

* In tmux, you can navigate through the different sessions and windows and see each ROS2 node running.

