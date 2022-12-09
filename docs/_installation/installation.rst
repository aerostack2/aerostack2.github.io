Installation
============

For using and installing Aerostack2, your ROS2 environment must be settled up. If ROS2 is installed using debian packages you should run 

.. code-block:: bash

 $ source /opt/ros/<your distro>/setup.bash

before running these steps.

* Create a AS2 workspace:

.. code-block:: bash

 $ mkdir -p ${HOME}/aerostack2_ws/src/ && cd ${HOME}/aerostack2_ws/src/

* Clone AS2 repository:

.. code-block:: bash

 $ git clone https://github.com/aerostack2/aerostack2.git

* Install core dependencies:

.. code-block:: bash

 $ sudo apt update
 $ sudo apt install tmux python3-vcstool python3-rosdep python3-pip python3-colcon-common-extensions -y
 $ sudo rosdep init

* Set environment vars (or add them to the ~/.bashrc file):

.. code-block:: bash
    
 $ echo 'export AEROSTACK2_PATH=$HOME/aerostack2_ws/src/aerostack2' >> $HOME/.bashrc
 $ echo 'source $AEROSTACK2_PATH/scripts/setup_env.bash' >> $HOME/.bashrc