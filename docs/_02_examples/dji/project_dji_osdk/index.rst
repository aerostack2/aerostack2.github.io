.. _project_dji_osdk:

================
DJI OSDK Example
================

These examples use the :ref:`OSDK DJI Platform <aerial_platform_dji_matrice_osdk_osdk>`.
You can install it following the instructions in :ref:`installation section <aerial_platform_dji_matrice_osdk_installation>`.

To install this project, please clone the repository:

.. code-block:: bash

   git clone https://github.com/aerostack2/project_dji_osdk

To start using this project, please go to the root folder of the project.



.. _project_dji_osdk_state_estimator:

----------------------------
Choosing the state estimator
----------------------------

The state estimator is configured in ``config/config.yaml``, under the ``state_estimator``
block. The project ships with the ``raw_odometry`` plugin and GPS enabled, which is what
outdoor flight with a DJI platform needs:

.. code-block:: yaml

   state_estimator:
     ros__parameters:
       plugin_name: "raw_odometry"
       raw_odometry:
         use_gps: true

``use_gps: true`` anchors the ``earth`` frame at a geodetic datum, which is what lets a
mission be written in GPS coordinates. By default the datum is the first GPS fix received.
The file also carries two commented alternatives for where that datum comes from:

.. code-block:: yaml

   # A fixed datum, useful across flights over the same site
   raw_odometry:
     use_gps: true
     gps_origin:
       set_origin: "manual"
       coordinates:
         latitude: 40.4168
         longitude: -3.7038
         altitude: 667.0

.. code-block:: yaml

   # A datum supplied at run time through the set_origin service
   raw_odometry:
     use_gps: true
     gps_origin:
       set_origin: "service"

.. warning::

    In ``service`` mode the transform tree stays incomplete until ``set_origin`` is called,
    so the estimator looks like it has hung until a ground station or a mission script makes
    that call.

.. note::

    For indoor flight, replace the block with a state estimator that does not depend on
    GPS. The available plugins and all of their parameters are documented in the
    :ref:`State Estimator plugins <as2_plugins_state_estimator>` reference. If you use a
    motion capture system, launch ``mocap4ros2`` with the ground station using the ``-m``
    flag: it is configured in ``config_ground_station/mocap4r2_optitrack.yaml``, and it
    feeds the state estimator, which turns the pose into the Aerostack2 common interface
    localization :ref:`topics <ros2_common_interfaces_state_estimator_topics>`.



.. _project_dji_osdk_execution:

---------
Execution
---------

Aerostack2 nodes run on the onboard computer of the drone. The ground station can run
either on that same computer or on an external one.

Aerostack2 nodes
================

Launch the Aerostack2 nodes once per drone, changing the namespace with ``-n``:

.. code-block:: bash

    ./launch_as2.bash -n drone0

The flags for the components launcher are:

- ``-n``: select the drone namespace to launch. Default is ``drone0``.
- ``-r``: record a rosbag. By default it does not record.
- ``-g``: launch using gnome-terminal instead of tmux. By default it is not set.

Ground station
==============

.. code-block:: bash

    ./launch_ground_station.bash -n drone0

The flags for the ground station launcher are:

- ``-n``: drone namespaces, comma separated. Default is ``drone0``.
- ``-t``: launch the keyboard teleoperation. By default it is not launched.
- ``-v``: open rviz. By default it is not opened.
- ``-m``: launch mocap4ros2. By default it is not launched.
- ``-r``: record a rosbag. By default it does not record.
- ``-g``: launch using gnome-terminal instead of tmux. By default it is not set.

Launching it with ``-t`` opens the
:ref:`keyboard teleoperation <user_interfaces_keyboard_teleoperation>` widget.



.. _project_dji_osdk_mission:

-----------------
Mission execution
-----------------

.. code-block:: bash

    # Aerostack2 Python API
    python3 mission.py -n drone0

    # Aerostack2 Python API, using GPS
    python3 mission_gps.py -n drone0

    # Aerostack2 Mission Interpreter
    python3 mission_interpreter.py -n drone0

    # Aerostack2 Behavior Trees
    python3 mission_behavior_tree.py -n drone0



.. _project_dji_osdk_closing:

-------
Closing
-------

.. code-block:: bash

    # Close the Aerostack2 nodes of a drone
    ./stop_tmuxinator_as2.bash drone0

    # Close the ground station
    ./stop_tmuxinator_ground_station.bash

    # Close both
    ./stop_tmuxinator.bash drone0

.. note::

    ``tmux kill-server`` has a similar effect, but it closes every tmux session, so be
    careful if you have others running. If the launchers were executed with the ``-g``
    flag, close the gnome-terminal tabs manually instead.
