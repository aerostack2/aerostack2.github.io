.. _examples_gazebo:

================
Gazebo Simulator
================

For these examples we are going to use :ref:`Ignition Gazebo Platform <aerial_platform_ignition_gazebo>`. 
You can install it following the instructions in :ref:`installation section <aerial_platform_ignition_gazebo>`.

.. figure:: resources/single_drone_ign.png
   :scale: 50
   :class: with-shadow

Every example project is installed by the following command:

.. code-block:: bash

    as2 project -n project_name

This will clone the project in the directory ``$AEROSTACK2_PATH/projects/``. 
Reopen the terminal to upload the new project:

.. code-block:: bash

    source ~/.bashrc

Go there by:

.. code-block:: bash

    as2 cd project_name

Here is the list of examples:

.. toctree::
   :maxdepth: 1

   project_gazebo/index.rst
