.. _project_tello:

=====================
Simple Tello Example
=====================

To install this project, clone the repository:

.. code-block:: bash

   git clone https://github.com/aerostack2/project_tello

To start using this project, please go to the root folder of the project.


.. _project_tello_launching:

---------
Launching
---------

The execution on the project is split into two parts: Aerostack2 components and ground station.

Launching Aerostack2
====================

To launch the Aerostack2 components, execute the following command:

.. code-block:: bash

    ./launch_as2.bash


This will open a tmux session with an Aerostack2 instance for each drone launched.

Launcher offers a few options to customize the execution. ``./launch_as2.bash -h`` will show option list. Options can be set with the following flags:

- ``-n``: select drones namespace to launch, values are comma separated. By default, it launches all drones from world description file.
- ``-g``: launch using gnome-terminal. By default not set, uses tmux.

Launching Ground Station
========================

To launch the ground station, execute the following command:

.. code-block:: bash

    ./launch_ground_station.bash

Launcher offers a different pool of options to customize the execution. ``./launch_ground_station.bash -h`` will show option list. Options can be set with the following flags:

- ``-m``: disable launch mocap4ros2. By default set.
- ``-t``: launch keyboard teleoperation. By default not set.
- ``-v``: launch rviz. By default not set.
- ``-r``: record rosbag. By default not set.
- ``-n``: select drones namespace to launch, values are comma separated. By default, it launches all drones from world description file.
- ``-g``: launch using gnome-terminal. By default not set, uses tmux.

Closing
=======

Close all nodes (aerostack2 and ground_station) with the following command:

.. code-block:: bash

    ./stop_tmuxinator.bash

.. note::

    The command ``tmux kill-server`` will have a similar effect but closing all tmux sessions, so be careful if you have other tmux sessions running.

    If launcher was executed with the flag ``-g``, closing should be done manually exiting all gnome-terminal tabs.

Close **only** the Aerostack2 nodes with the following command:

.. code-block:: bash

    ./stop_tmuxinator_as2.bash

Close **only** the ground station nodes with the following command:

.. code-block:: bash

    ./stop_tmuxinator_ground_station.bash


.. _project_tello_mission:

-----------------
Mission execution
-----------------

The project offers a few examples of mission execution.

- **Keyboard Teleoperation control**: Using reactive teleoperation control. For both single and multiagent swarms.
- **Python API missions**: Using Aerostack2 python API for mission definition. For single drone, using GPS and multiple drones.
- **Behavior Tree missions**: Using Behavior Tree as the mission planner. For single drone only.
- **Mission Interpreter**: Using Aerostack2 mission interpreter. For single drone only.


.. _project_tello_keyboard_teleoperation:

Keyboard Teleoperation control
==============================

In order to launch the components for a **single drone**, Aerostack2 launcher does not need any additional flags. Just execute ``./launch_as2.bash``.

Ground station should be launched with ``-t`` flag to enable keyboard teleoperation. Take a look at the :ref:`keyboard teleoperation user guide <user_interfaces_keyboard_teleoperation>` for more information.
A window containing the teleoperation widget should pop up:

.. figure:: images/keyboard_teleop_view.png
   :scale: 50
   :class: with-shadow
   
   Keyboard teleoperation


.. _project_tello_python_api:

Python API missions
===================

In order to launch the components for **multiple drones**, both launchers require the flag ``-n`` indicating drone namespaces to use.

There are three python scripts available for mission execution in the project. For single drone missions, use ``python3 mission.py`` for flying a square.

For multi drone missions, use ``python3 mission_swarm.py`` where a group of drones will fly a swarm coreography. Here it is what the execution looks like:

.. raw:: html

    <div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden; max-width: 100%; height: auto;">
        <iframe src="https://www.youtube.com/embed/BlF6rU9R8Nk" frameborder="0" allowfullscreen style="position: absolute; top: 0; left: 0; width: 100%; height: 100%;"></iframe>
    </div>

.. note::

    To understand how missions are built using the Aerostack2 python API, take a look at the :ref:`development_guide_api_python_api` reference guide.


.. _project_tello_mission_interpreter:

Mission Interpreter
===================

Previous missions were defined using python syntaxis. Aerostask2 offers a mission interpreter that allows mission definition using a JSON format.
Currently, the mission interpreter script at the project is only available for single drone missions.
To launch the mission interpreter, execute ``python3 mission_interpreter.py``.
The execution is similar to the python API mission where the drone will fly a square.


.. _project_tello_behavior_tree:

Behavior Tree
=============

Missions can also be defined using a behavior tree. The project offers a mission example using a behavior tree for a single drone.
To launch the behavior tree mission, execute ``python3 mission_behavior_tree.py``.
The execution is similar to the python API mission where the drone will fly a square.

.. note::

    Trees can be defined using GUIs like `Groot <https://www.behaviortree.dev/groot/>`.