.. _aerial_platform_px4_installation_guide:

==================
Installation Guide
==================


.. contents:: Table of Contents
   :depth: 3
   :local:

You can see the `PX4 Autopilot ROS 2 Guide <https://docs.px4.io/main/en/ros/ros2_comm.html>`_ for it installation, or follow the steps below.

We will use a colcon workspace for the installation:

  .. code-block:: bash

    mkdir -p ~/px4_ws/src

.. note::
  You can add the following line to your ``~/.bashrc`` file to source the workspace automatically.

  .. code-block:: bash

    echo "source ~/px4_ws/install/setup.bash" >> ~/.bashrc



.. _aerial_platform_px4_installation_dependencies:

----------------
PX4 Dependencies
----------------

Follow the steps below to install the dependencies for PX4:

1. Clone the PX4 Messages repository:

  .. code-block:: bash

    cd ~/px4_ws/src
    git clone https://github.com/PX4/px4_msgs.git

2. Build workspace:

  .. code-block:: bash

    cd ~/px4_ws
    colcon build --symlink-install



.. _aerial_platform_px4_installation_platform:

------------------------
Install platform package
------------------------

Clone the Aerostack2 Pixhawk Platform repository into the workspace created in the section before:

.. code-block:: bash

  cd ~/px4_ws/src
  git clone https://github.com/aerostack2/as2_platform_pixhawk.git

Build it:

.. code-block:: bash

  source ~/px4_ws/install/setup.bash
  cd ~/px4_ws
  colcon build --symlink-install



.. _aerial_platform_px4_installation_xrce_dds:

-----------------------------
XRCE-DDS (PX4-FastDDS Bridge)
-----------------------------

For more information about XRCE-DDS, see `PX4 Autopilot XRCE-DDS bridge <https://docs.px4.io/main/en/middleware/xrce_dds.html>`_.
For its installation, clone it into the workspace and build it using colcon:

.. code-block:: bash

  cd ~/px4_ws/src
  git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git

Build it:

.. code-block:: bash

  cd ~/px4_ws
  colcon build --symlink-install



.. _aerial_platform_px4_installation_px4_autopilot:

-------------
PX4 Autopilot
-------------

For more information about PX4 Autopilot, see `PX4 Autopilot <https://docs.px4.io/main/en/>`_.

1. Clone repository

 .. code-block:: bash

   mkdir -p ~/px4_ws/thirdparties
   cd ~/px4_ws/thirdparties
   git clone -b v1.14.0-beta2 https://github.com/PX4/PX4-Autopilot.git --recursive

2. Add colcon ignore file to avoid building PX4-Autopilot with colcon

 .. code-block:: bash

   cd ~/px4_ws/thirdparties/
   touch COLCON_IGNORE

3. Set PX4_FOLDER environment variable

 .. code-block:: bash

   echo "export PX4_FOLDER=~/px4_ws/thirdparties/PX4-Autopilot" >> ~/.bashrc




.. _aerial_platform_px4_installation_px4_autopilot_px4_vision:

Setting up PX4 Vision Autonomy Development Kit
==============================================

For using `PX4 Vision Autonomy Development Kit <https://docs.px4.io/main/en/complete_vehicles/px4_vision_kit.html>`_ you need to follow the steps below:

1. Build the PX4-Autopilot firmware with the following command:

 .. code-block:: bash

   cd ~/px4_ws/thirdparties/PX4-Autopilot
   make px4_fmu-v6c_default

2. Upload the firmware to the Pixhawk 4 using `QGroundControl <http://qgroundcontrol.com/>`_.

3. Enable XRCE_DDS_0_CFG parameter in QGroundControl on the TELEM port. We use TELEM 2 port for this.

4. Set up baud rate for the TELEM port. We recommend using 921600 baud rate for this.
   