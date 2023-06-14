.. _project_dji_osdk:

================
DJI OSDK Example
================

For these examples we are going to use :ref:`DJI Platform <aerial_platform_dji_matrice>`. 
You can install it following the instructions in :ref:`installation section <aerial_platform_dji_matrice_installation>`.

To install this project, if you configured our :ref:`CLI <development_cli>`, you can install this project with:

.. code-block:: bash

    as2 project -n dji_osdk

This will clone the project in the directory ``$AEROSTACK2_PATH/projects/``. 
Reopen the terminal to upload the new project:

.. code-block:: bash

    source ~/.bashrc

And move there by executing:

.. code-block:: bash

    as2 cd dji_osdk

Alternatively, you can clone the repository with:

.. code-block:: bash

   git clone https://github.com/aerostack2/project_dji_osdk

To start using this project, please go to the root folder of the project.



.. _project_dji_osdk_execution:

---------
Execution
---------

Available launch arguments are:

- ``-e``: estimator type. Allowed values [``gps_odometry``, ``raw_odometry``, ``mocap``]. Default: ``gps_odometry``.
- ``-t``: launch keyboard teleoperation.
- ``-r``: record rosbag.
- ``-n``: use custom dron namespace.

In order to launch the components, do:

- For the **mocap** version, do:

.. code-block:: bash

    ./launch_as2.bash -e mocap -t

- For the **odometry** indoor version, do:

.. code-block:: bash

    ./launch_as2.bash -e raw_odometry -t

- For the **odometry** outdoor version, do:

.. code-block:: bash

    ./launch_as2.bash -e gps_odometry -t

.. note:: 

    Before launching the components with **mocap**, it is also necessary to set the file ``real_config/mocap.yaml``. This file will be used by the state estimator mocap plugin to 
    get the ground truth pose coming from our motion capture system into the Aerostack2 common interface localization :ref:`topics <ros2_common_interfaces_state_estimator_topics>`.


To start the mission, execute:

.. code-block:: bash

    python3 mission.py

To do a clean exit of tmux, execute the following command:

.. code-block:: bash

    ./stop.bash drone0


