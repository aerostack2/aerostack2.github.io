.. _project_crazyflie:

=================
Project Crazyflie
=================

This project contains the simulated version, which uses :ref:`Gazebo Platform <aerial_platform_ignition_gazebo>`, and 
the real version, which uses :ref:`Crazyflie Platform <aerial_platform_crazyflie>`. You can install them following the instructions in :ref:`Gazebo Platform installation section <aerial_platform_ignition_gazebo_installation>` and :ref:`Crazyflie Platform installation section <aerial_platform_crazyflie_installation>`.
To install this project, you can clone the repository with:

.. code-block:: bash

   git clone https://github.com/aerostack2/project_crazyflie

Alternatively, if you configured our :ref:`CLI <development_cli>`, you can install this project with:

.. code-block:: bash

    as2 project -n project_crazyflie

This will clone the project in the directory ``$AEROSTACK2_PATH/projects/``. 
Reopen the terminal to upload the new project:

.. code-block:: bash

    source ~/.bashrc

To start using this project, please go to the root folder of the project.

.. _project_crazyflie_simulated:

-------------------
Simulated execution
-------------------

We can execute this project in simulation mode. This will open a simulation in Gazebo and the Aerostack2 components will use simulation time.

.. _project_crazyflie_simulated_single_drone:

Single drone
############

In order to launch the components for a **single drone**, do:

.. code-block:: bash

    ./main_launcher.sh -s -k

This will open a simulation for a single drone alongside the Aerostack2 components necessary for the mission execution. It will also open a keyboard teleoperation, which you can use to teleoperate the drone (argument ``-k``).

To start the mission, go to a new terminal line and execute:

.. code-block:: bash

    python3 mission.py -s

.. _project_crazyflie_simulated_swarm_drones:

Swarm drones
############

In order to launch the components for a **swarm of 3 drones**, do:

.. code-block:: bash

    ./main_launcher.sh -s -w -k

This will open a simulation for a swarm of 3 drones alongside the Aerostack2 components necessary for the mission execution. It will also open a keyboard teleoperation, which you can use to teleoperate the drones (argument ``-k``).

To start the mission, go to a new terminal line and execute:

.. code-block:: bash

    python3 mission_swarm.py -s

.. _project_crazyflie_real:

--------------
Real execution
--------------

In order to perform a real execution of this project, it is needed to have the crazyflie(s) :ref:`configured <aerial_platform_crazyflie_installation_prerequisites>` and switched on in its initial position. 

Before launching the components, remember to set the correct address in the file ``real_config/swarm_config_file.yaml``, as indicated in :ref:`aerial_platform_crazyflie_platform_launch`.
Leave only the entries for the drone(s) that will be used, even if it is only one.

It is also important to decide which state estimator to use. Currently, Aerostack2 supports two types of state estimators for the Crazyflie, this are:

- **Optitrack**: which uses ``mocap`` plugin. 
- **Optical Flow**: which uses ``raw_odometry`` plugin.

.. _project_crazyflie_real_single_drone:

Single drone
############

In order to launch the components for a **single drone** with **optical flow**, do:

.. code-block:: bash

    ./main_launcher.sh -e raw_odometry -k

Before launching the components with **mocap**, it is also necessary to set the file ``real_config/swarm_config_file.yaml``. This file will be used by the state estimator mocap plugin to 
get the ground truth pose coming from our motion capture system into the Aerostack2 common interface localization :ref:`topics <ros2_common_interfaces_state_estimator_topics>`.

In order to launch the components for a **single drone** with **mocap**, do:

.. code-block:: bash

    ./main_launcher.sh -e mocap -k

To start the mission for a **single drone**, go to a new terminal line and execute:

.. code-block:: bash

    python3 mission.py

.. _project_crazyflie_real_swarm_drones:

Swarm drones
############

In order to launch the components for a **swarm of 3 drones** with **optical flow**, do:

.. code-block:: bash

    ./main_launcher.sh -w -e raw_odometry -k

In order to launch the components for a **swarm of 3 drones** with **mocap**, do:

.. code-block:: bash

    ./main_launcher.sh -w -e mocap -k

To start the mission for a **swarm of 3 drones**, go to a new terminal line and execute:

.. code-block:: bash

    python3 mission_swarm.py