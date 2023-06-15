.. _getting_started_ubuntu_installation_source:

==============
Source Install
==============



.. _getting_started_ubuntu_installation_prerequisites:

---------------------
Previous dependencies
---------------------

Make sure to have installed ROS 2 via Debian Packages for `Humble <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html>`__ or `Galactic <https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html>`__.

After that, make sure you have installed the following dependencies:

.. code-block:: bash

    sudo apt install python3-rosdep python3-pip python3-colcon-common-extensions -y



.. _getting_started_ubuntu_installation_source_install:

-------------------
Source Installation
-------------------

For using and installing Aerostack2, your ROS 2 environment must be settled up. 
If ROS 2 is installed using debian packages you should source the ros2 environment before running these steps:

.. code-block:: bash

    source /opt/ros/<ros2-distro>/setup.bash

A new workspace for Aerostack2 will be created in order to clone the source code and build it. `Rosdep <https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html>`_ will be used to get the binary dependecies for Aerostack2 packages:

.. code-block:: bash

    mkdir -p ~/aerostack2_ws/src/ && cd ~/aerostack2_ws/src/ 
    git clone https://github.com/aerostack2/aerostack2.git
    cd ~/aerostack2_ws
    rosdep update
    rosdep install -y -r -q --from-paths src --ignore-src --rosdistro <ros2-distro>



.. _getting_started_ubuntu_installation_source_build:

--------------
Standard Build
--------------

.. code-block:: bash

    cd ~/aerostack2_ws
    colcon build --symlink-install



.. _getting_started_ubuntu_installation_source_build_cli:

------------------
Build with AS2 CLI
------------------

Aditionally, Aerostack2 provides with a :ref:`CLI <development_cli>` that is able to perform Aerostack2 build operation from any directory:

.. code-block:: bash

    echo 'export AEROSTACK2_PATH=$HOME/aerostack2_ws/src/aerostack2' >> $HOME/.bashrc
    echo 'source $AEROSTACK2_PATH/as2_cli/setup_env.bash' >> $HOME/.bashrc && source $HOME/.bashrc

.. code-block:: bash
    
    as2 build
