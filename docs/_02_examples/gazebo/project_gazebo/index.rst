.. _project_gazebo:

=====================
Simple Gazebo Example
=====================

To install this project, if you configured our :ref:`CLI <development_cli>`, you can install this project with:

.. code-block:: bash

    as2 project -n project_gazebo

This will clone the project in the directory ``$AEROSTACK2_PATH/projects/``. 
Reopen the terminal to upload the new project:

.. code-block:: bash

    source ~/.bashrc

And move there by executing:

.. code-block:: bash

    as2 cd project_gazebo

Alternatively, you can clone the repository with:

.. code-block:: bash

   git clone https://github.com/aerostack2/project_gazebo

To start using this project, please go to the root folder of the project.



.. _project_gazebo_simulated:

---------
Execution
---------

The execution will open a simulation in Gazebo and the Aerostack2 components will use simulation time.

The flags for the components launcher are:

- ``-m``: launch the components for the swarm multiagent system.
- ``-t``: launch keyboard teleoperation.
- ``-r``: record rosbag.
- ``-b``: use `Behavior Tree <https://www.behaviortree.dev/>`_.
- ``-n``: use custom dron namespace.



.. _project_gazebo_simulated_single_drone:

Single drone
############

In order to launch the components for a **single drone**, do:

.. code-block:: bash

    ./launch_as2.sh -t

This will open a simulation for a single drone alongside the Aerostack2 components necessary for the mission execution.

A window like the following image should open.

.. figure:: images/single_drone_ign.png
   :scale: 50
   :class: with-shadow
   
   Ignition Gazebo simulator

It will also open a keyboard teleoperation (argument ``-t``), which you can use to teleoperate the drone with the :ref:`aerostack2 keyboard teleoperation user interface <user_interfaces_keyboard_teleoperation>`.

A window like the following image should popup:

.. figure:: images/keyboard_teleop_view.png
   :scale: 50
   :class: with-shadow
   
   Keyboard teleoperation

To start the mission, execute:

.. code-block:: bash

    python3 mission.py



Also, you can try the mission planner, which describes the mission in a JSON format:

.. code-block:: bash

    python3 mission_planner.py

To do a clean exit of tmux, execute:

.. code-block:: bash

    ./stop.bash



.. _project_gazebo_simulated_behavior_tree:

Behavior Tree
#############

In order to launch the components for using **behavior tree**, do:

.. code-block:: bash

    ./launch_as2.sh -b

Then, you can start the mission with:

**TBD**



.. _project_gazebo_simulated_swarm_drones:

Swarm drones
############

In order to launch the components for a **swarm of 3 drones**, do:

.. code-block:: bash

    ./launch_as2.sh -m -t

This will open a simulation for a swarm of drones (argument ``-m``) alongside the Aerostack2 components necessary for the mission execution.

A window like the following image should open.

.. figure:: images/swarm_ign.png
   :scale: 50
   :class: with-shadow
   
   Ignition Gazebo simulator

It will also open a keyboard teleoperation (argument ``-t``), which you can use to teleoperate the swarm with the :ref:`aerostack2 keyboard teleoperation user interface <user_interfaces_keyboard_teleoperation>`.

A window like the following image should popup:

.. figure:: images/keyboard_swarm_view.png
   :scale: 50
   :class: with-shadow
   
   Keyboard teleoperation

To start the mission, execute:

.. code-block:: bash

    python3 mission_swarm.py

To do a clean exit of tmux, execute the following command with the list of the used drones:

.. code-block:: bash

    ./stop.bash drone0 drone1 drone2
