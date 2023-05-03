.. _examples_gazebo:

================
Gazebo Simulator
================

For these examples we are going to use :ref:`Ignition Gazebo Platform <aerial_platform_ignition_gazebo>`. 
You can install it following the instructions in :ref:`installation section <aerial_platform_ignition_gazebo>`.

.. figure:: resources/single_drone_ign.png
   :scale: 50
   :class: with-shadow

We are going to use `Aerostack2 Gazebo Project <https://github.com/aerostack2/project_ignition>`_ for running the examples.
Install it by the following command:

.. code-block:: bash

    as2 project -n gazebo

This will clone the project in the directory ``$AEROSTACK2_PATH/projects/``. 
Reopen the terminal to upload the new project:

.. code-block:: bash

    source ~/.bashrc

Go there by:

.. code-block:: bash

    as2 cd gazebo

Here is the list of examples:

.. toctree::
   :maxdepth: 2

   keyboard_single_drone/index.rst
   keyboard_swarm_drones/index.rst
   python_single_drone/index.rst
   python_swarm_drones/index.rst
