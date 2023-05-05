.. _project_gazebo:

==============
Project Gazebo
==============

This project contains the simulated version, which uses :ref:`Gazebo Platform <aerial_platform_ignition_gazebo>`. You can install it by following the instructions in :ref:`Gazebo Platform installation section <aerial_platform_ignition_gazebo_installation>`.
To install this project, you can clone the repository with:

.. code-block:: bash

   git clone https://github.com/aerostack2/project_gazebo

Alternatively, if you configured our :ref:`CLI <development_cli>`, you can install this project with:

.. code-block:: bash

    as2 project -n project_gazebo

This will clone the project in the directory ``$AEROSTACK2_PATH/projects/``. 
Reopen the terminal to upload the new project:

.. code-block:: bash

    source ~/.bashrc

To start using this project, please go to the root folder of the project.

.. _project_gazebo_simulated:

---------
Execution
---------

The execution will open a simulation in Gazebo and the Aerostack2 components will use simulation time.

The flags for the components launcher are:

- ``-w``: launch the components for the swarm.
- ``-k``: launch keyboard teleoperation.
- ``-r``: record rosbag.

.. _project_gazebo_simulated_single_drone:

Single drone
############

In order to launch the components for a **single drone**, do:

.. code-block:: bash

    ./launch_as2.sh -k

This will open a simulation for a single drone alongside the Aerostack2 components necessary for the mission execution.

A window like the following image should open.

.. figure:: images/single_drone_ign.png
   :scale: 50
   :class: with-shadow
   
   Ignition Gazebo simulator

It will also open a keyboard teleoperation (argument ``-k``), which you can use to teleoperate the drone with the :ref:`aerostack2 keyboard teleoperation user interface <user_interfaces_keyboard_teleoperation>`.

A window like the following image should popup:

.. figure:: images/keyboard_teleop_view.png
   :scale: 50
   :class: with-shadow
   
   Keyboard teleoperation

To start the mission, go to a new terminal line and execute:

.. code-block:: bash

    python3 mission.py

.. _project_gazebo_simulated_swarm_drones:

Swarm drones
############

In order to launch the components for a **swarm of 3 drones**, do:

.. code-block:: bash

    ./launch_as2.sh -w -k

This will open a simulation for a swarm of drones alongside the Aerostack2 components necessary for the mission execution.

A window like the following image should open.

.. figure:: images/swarm_ign.png
   :scale: 50
   :class: with-shadow
   
   Ignition Gazebo simulator

It will also open a keyboard teleoperation (argument ``-k``), which you can use to teleoperate the swarm with the :ref:`aerostack2 keyboard teleoperation user interface <user_interfaces_keyboard_teleoperation>`.

A window like the following image should popup:

.. figure:: images/keyboard_swarm_view.png
   :scale: 50
   :class: with-shadow
   
   Keyboard teleoperation

To start the mission, go to a new terminal line and execute:

.. code-block:: bash

    python3 mission_swarm.py
