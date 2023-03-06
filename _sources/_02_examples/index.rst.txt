.. _examples:

========
Examples
========

In this section, severals examples of how to use Aerostack2 are shown.

.. note::
    Every project in Aerostack2 uses `tmux <https://github.com/tmux/tmux/wiki>`_ to get a more organized execution. Install it with:

    .. code-block:: bash
        
        sudo apt install tmux



.. _examples_prerequisites:

-------------
Prerequisites
-------------

For these examples we are going to use :ref:`Ignition Gazebo Platform <aerial_platform_ignition_gazebo>`. You need to install it following the instructions in :ref:`installation section <aerial_platform_ignition_gazebo>`.

After that, we need to clone the example repository:

* We create a directory where we want to store our projects. It can be any directory:

.. code-block:: bash

    cd $AEROSTACK2_PATH && mkdir projects/

* Clone the project in the directory we just created:

.. code-block:: bash

    cd ${AEROSTACK2_PATH}/projects/ && git clone https://github.com/aerostack2/project_ignition && cd project_ignition/



.. _examples_basics:

---------------
Basics Examples
---------------

Choose an example to run:

.. toctree::
   :maxdepth: 2

   keyboard_single_drone/index.rst
   keyboard_swarm_drones/index.rst
   python_single_drone/index.rst
   python_swarm_drones/index.rst