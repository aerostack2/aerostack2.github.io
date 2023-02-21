Project Ignition
================

This is a basic mission simulated in Ignition.

Installation
############

Every project in Aerostack2 uses `tmux <https://github.com/tmux/tmux/wiki>`_ to get a more organized execution. Install it with:

.. code-block:: bash
    
    sudo apt install tmux

* We create a directory where we want to store our projects. It can be any directory:

.. code-block:: bash

    cd $AEROSTACK2_PATH && mkdir projects/

* Clone the project in the directory we just created:

.. code-block:: bash

    cd ${AEROSTACK2_PATH}/projects/ && git clone https://github.com/aerostack2/project_ignition && cd project_ignition/

Single drone execution
######################

* Run Ignition simulator:

.. code-block:: bash

    ./launch_ignition.bash

A window like the following image should open.

.. figure:: images/ignition_gazebo.png
   :scale: 50
   :class: with-shadow
   
   Ignition Gazebo simulator

* In a new terminal, run Aerostack2 nodes:

.. code-block:: bash

    ./main_launcher.bash

* In a new terminal again, execute mission:

.. code-block:: bash

    python3 mission.py

* Wait for the mission to finish. Once it finished, close the terminal with:

.. code-block:: bash

    ./stop.bash

More information on how to run each project can be found in the README.md located on each project folder.

Swarm execution
###############

* Run Ignition simulator for swarm:

.. code-block:: bash

    ./launch_ignition_swarm.bash

A window like the following image should open.

.. figure:: images/ignition_gazebo_swarm.png
   :scale: 50
   :class: with-shadow
   
   Ignition Gazebo simulator

* In a new terminal, run Aerostack2 nodes:

.. code-block:: bash

    ./launch_swarm.bash

* In a new terminal again, execute mission:

.. code-block:: bash

    python3 mission_swarm.py

* Wait for the mission to finish. Once it finished, close the terminal with:

.. code-block:: bash

    ./stop.bash

More information on how to run each project can be found in the README.md located on each project folder.