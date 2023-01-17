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
---------------------

Make sure to have installed ROS2 via Debian Packages `Humble <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html>` or Galactic `Humble <https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html>`

Source installation
-------------------

For using and installing Aerostack2, your ROS2 environment must be settled up. If ROS2 is installed using debian packages you should source the ros2 environment before running these steps:

.. code-block:: bash

    source /opt/ros/<ros2-distro>/setup.bash

A new workspace for Aerostack2 will be created in order to clone the source code and build it. `Rosdep <https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html>`_ will be used to get the binary dependecies for Aerostack2 packages:

.. code-block:: bash

    mkdir -p ~/aerostack2_ws/src/ && cd ~/aerostack2_ws/src/ 
    git clone https://github.com/aerostack2/aerostack2.git
    cd ~/aerostack2_ws
    rosdep install -y -r -q --from-paths src --ignore-src --rosdistro <ros2-distro>

Standard Build
--------------

.. code-block:: bash

    cd ~/aerostack2_ws
    colcon build --symlink-install

Build with as2 CLI
------------------

Aditionally, Aerostack2 provides with a :ref:`CLI` that is able to perform Aerostack2 build operation from any directory:

.. code-block:: bash

    echo 'export AEROSTACK2_PATH=$HOME/aerostack2_ws/src/aerostack2' >> $HOME/.bashrc
    echo 'source $AEROSTACK2_PATH/as2_cli/setup_env.bash' >> $HOME/.bashrc && source $HOME/.bashrc

.. code-block:: bash
    
    as2 build
 
Basic example
#############

Every project in Aerostack2 uses `tmux <https://github.com/tmux/tmux/wiki>`_ to get a more organized execution. Install it with:

.. code-block:: bash
    
    sudo apt install tmux

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
