.. _project_multirotor_simulator:

============================
Multirotor Simulator Example
============================

To install this project, clone the repository:

.. code-block:: bash

   git clone https://github.com/aerostack2/project_as2_multirotor_simulator

To start using this project, please go to the root folder of the project.



.. _project_multirotor_simulator_simulated:

---------
Execution
---------

The execution will launch the Aerostack2 components.

The flags for the components launcher are:

* ``-n <namespace>`` - namespace for the drone. Default, not specified and uses ``config/world.yaml`` configuration. If specified, it uses ``config/platform_config_file.yaml`` configuration.
* ``-m`` - multi agent mode. Default is disabled. If specified, it uses ``config/world_swarm.yaml`` configuration.
* ``-d`` - launch rviz visualization. If not specified, it does not launch rviz visualization. If specified, it launches rviz visualization with `config/tf_visualization.rviz` configuration.
* ``-e <estimator type>`` - estimator type. Default is ``ground_truth``. Available options: ``ground_truth``, ``raw_odometry``, ``raw_odometry_gps``. It uses configuration from ``config/state_estimator*.yaml``.
* ``-r`` - record rosbag. Default is disabled. If specified, it records rosbag in ``rosbag`` directory.
* ``-t`` - launch keyboard teleoperation. Default is disabled. If specified, it launches keyboard teleoperation.


.. _project_multirotor_simulator_simulated_single_drone:

Single drone
============

In order to launch the components for a **single drone**, do:

.. code-block:: bash

    ./launch_as2.bash -d -t

This will open a simulation for a single drone alongside the Aerostack2 components necessary for the mission execution. 
A window with RViz (argument ``-d``) should popup.


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



.. _project_multirotor_simulator_simulated_swarm_drones:

Swarm drones
============

In order to launch the components for a **swarm of 3 drones**, do:

.. code-block:: bash

    ./launch_as2.bash -m -t -d

This will open a simulation for a swarm of drones (argument ``-m``) alongside the Aerostack2 components necessary for the mission execution.
A window with RViz (argument ``-d``) should popup.

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
