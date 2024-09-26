.. _getting_started_ubuntu_installation_binary:

==============
Binary Install
==============

Currently, we only support binary installation on ROS2 Humble. We are working on binary installation for other ROS2 distributions.
To install all aerostack2 packages, run the following command:

.. code-block:: bash

    sudo apt install ros-humble-aerostack2

For a specific platform, you can install the following packages:

.. code-block:: bash

    sudo apt install ros-humble-as2-platform-crazyflie
    sudo apt install ros-humble-as2-platform-tello
    sudo apt install ros-humble-as2-platform-mavlink
    sudo apt install ros-humble-as2-platform-pixhawk
    sudo apt install ros-humble-as2-platform-dji-osdk
    sudo apt install ros-humble-as2-platform-dji-psdk


You can also install only the packages you need. The available packages can be checked at `ROS Index <https://index.ros.org/search/?term=aerostack2&section=pkgs>`__.

.. warning:: Projects main branch is linked with Aerostack2 main branch. If you want to use a specific Aerostack2 binary version, you should checkout the project tag that matches the Aerostack2 version.
