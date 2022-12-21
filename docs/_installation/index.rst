Installation Instructions
=========================

License agreement
#################

Before the installation, you must [read and accept the conditions defined by the software license]
:ref:`License`

Previous dependencies
#####################

Make sure to have installed ROS2 via Debian Packages `Humble <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html>`_

For simulation install `Ignition Fortress <https://gazebosim.org/docs/fortress/install_ubuntu>`_

Installation
############

For using and installing Aerostack2, your ROS2 environment must be settled up. If ROS2 is installed using debian packages you should run 

.. code-block:: bash

 $ source /opt/ros/<your distro>/setup.bash

before running these steps.

* Create a AS2 workspace:

.. code-block:: bash

 $ mkdir -p ${HOME}/aerostack2_ws/src/ && cd ${HOME}/aerostack2_ws/src/

* Clone AS2 repository:

.. code-block:: bash

 $ git clone --recursive https://github.com/aerostack2/aerostack2.git
 
* Install core dependencies:

.. code-block:: bash

 $ sudo apt update
 $ sudo apt install tmux python3-vcstool python3-rosdep python3-pip python3-colcon-common-extensions -y
 $ sudo rosdep init

* Set environment vars (or add them to the ~/.bashrc file):

.. code-block:: bash
    
 $ echo 'export AEROSTACK2_PATH=$HOME/aerostack2_ws/src/aerostack2' >> $HOME/.bashrc
 $ echo 'source $AEROSTACK2_PATH/as2_cli/setup_env.bash' >> $HOME/.bashrc

Release installation
####################

Open a new terminal or update current shell with:

.. code-block:: bash

 $ source ~/.bashrc

* Download repositories (from project or source release):

  * From source release:
  
  	.. code-block:: bash
  	
         $ cd $AEROSTACK2_PATH
         $ vcs import --recursive < ./installers/as2_beta_beetroot.repos
         
  * From project (e.g.):

  	.. code-block:: bash
  	
         $ as2 install mbzirc project
         $ as2 project -n mbzirc

* Install dependencies:

.. code-block:: bash

 $ cd ${HOME}/aerostack2_ws
 $ rosdep update
 $ rosdep install --from-paths src --ignore-src -r -y

* Build:

.. code-block:: bash
    
 $ as2 build

More information on how to run each project can be found in the README.md located on each project folder.
