.. _examples_keyboard_example:

==============================
Keyboard Teleoperation Example
==============================

In this example, a single drone is simulated in Ignition Gazebo and user can teleoperate it with the keyboard.

.. note:: 

    Make sure you have installed :ref:`Keyboard Teleoperation dependencies <user_interfaces_keyboard_teleoperation_dependencies>`.


* Run Ignition simulator. The simulation enviroment can be modified in the file ``simulation_config/default.json``.

.. code-block:: bash

    ./launch_ignition.bash

A window like the following image should open.

.. figure:: images/single_drone_ign.png
   :scale: 50
   :class: with-shadow
   
   Ignition Gazebo simulator

* In a new terminal, run Aerostack2 nodes:

.. code-block:: bash

    ./main_launcher.bash

A window like the following image should popup, where you can teleoperate the drone with the :ref:`aerostack2 keyboard teleopration user interface <keyboard_teleoperation>`.

.. figure:: images/keyboard_teleop_view.png
   :scale: 50
   :class: with-shadow
   
   Keyboard teleoperation

* In tmux session drone_sim_0, in window 0, the alphanumeric viewer is shown with some relevant information about the drone. You can navigate through the different windows and see each ROS2 node running.

