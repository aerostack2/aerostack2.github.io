Getting started
===============

This page will help you go through the installation and build process for Aerostack2 packages, as well as providing a simple example project for you to execute.

Install and build
#################

License agreement
-----------------

Before the installation, you must read and accept the conditions defined by the software license
:ref:`License`

Previous dependencies
--------------------

Make sure to have installed ROS2 via Debian Packages `Humble <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html>`_

For simulation install `Ignition Fortress <https://gazebosim.org/docs/fortress/install_ubuntu>`_

Source installation
-------------------

For using and installing Aerostack2, your ROS2 environment must be settled up. If ROS2 is installed using debian packages you should run 

.. code-block:: bash

    source /opt/ros/<your distro>/setup.bash

before running these steps.

* Create a AS2 workspace:

.. code-block:: bash

    mkdir -p ${HOME}/aerostack2_ws/src/ && cd ${HOME}/aerostack2_ws/src/

* Clone AS2 repository:

.. code-block:: bash

    git clone --recursive https://github.com/aerostack2/aerostack2.git
 
* Install core dependencies:

.. code-block:: bash

    sudo apt update
    sudo apt install tmux python3-vcstool python3-rosdep python3-pip python3-colcon-common-extensions -y
    sudo rosdep init

* Set environment vars (or add them to the ~/.bashrc file):

.. code-block:: bash
    
 $ echo 'export AEROSTACK2_PATH=$HOME/aerostack2_ws/src/aerostack2' >> $HOME/.bashrc
 
* Install dependencies:

.. code-block:: bash

    cd ${HOME}/aerostack2_ws
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y

Build
-----

.. code-block:: bash

    colcon build --symlink-install

Build CLI
---------

Aditionally, Aerostack2 provides with a :ref:`CLI` that is able to perform Aerostack2 build operation from any directory:

.. code-block:: bash

    source ~/.bashrc

.. code-block:: bash
    
    as2 build
 
Basic example
#############

For this example we are going to execute a basic mission simulated in Ignition.

* We create a directory where we want to store our projects. It can be any directory:

.. code-block:: bash

    cd $AEROSTACK2_PATH && mkdir projects/

* Clone the project in the directory we just created:

.. code-block:: bash

    cd ${AEROSTACK2_PATH}/projects/ && git clone https://github.com/aerostack2/project_ignition && cd project_ignition/

* Run Ignition simulator:

.. code-block:: bash

    ./launch_ignition.bash

A window like the following image should open.

.. image:: images/ignition_gazebo.png
   :scale: 50
   :class: with-shadow
   :align: left

* In a new terminal, run Aerostack2 nodes:

.. code-block:: bash

    ./as2_launch.bash

* In a new terminal again, execute mission:

.. code-block:: bash

    python3 mission.py

* Wait for the mission to finish. Once it finished, close the terminal with:

.. code-block:: bash

    ./stop.bash

More information on how to run each project can be found in the README.md located on each project folder.
