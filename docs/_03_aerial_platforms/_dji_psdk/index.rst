.. _aerial_platform_dji_matrice_psdk_psdk:

=======================
DJI Matrice Series PSDK
=======================

.. contents:: Table of Contents
   :depth: 3
   :local:



.. _aerial_platform_dji_matrice_psdk_introduction:

------------
Introduction
------------

DJI Matrice Series using `DJI Onboard PSDK <https://github.com/dji-sdk/Onboard-SDK>`_ has compatibility with DJI M300, DJI M350 and DJI M30T.

.. figure:: resources/DJI_M300.jpg
   :scale: 15
   :class: with-shadow


.. _aerial_platform_dji_matrice_psdk_installation:

------------
Installation
------------

.. _aerial_platform_matrice_psdk_installation_prerequisites:

Prerequisites to configure any Xavier/Orin NX/AGX board to connect to PSDK, These steps are only done **once**
==============================================================================================================
.. _aerial_platform_dji_matrice_psdk_installation_prerequisites_software_once:


Clone configuration repository to grab the necessary files
------------------------------------------------
.. code-block:: console

   cd ~
   git clone git@github.com:aerostack2/psdk_config_files.git
   cd psdk_config_files

Disable l4t-device-mode auto start
----------------------------------------------

.. code-block:: console

   sudo systemctl disable nv-l4t-usb-device-mode.service

Replace system device-mode script
---------------------------------

Copy your custom startup script into place and make it executable:

.. code-block:: console

   sudo cp ./nv-l4t-usb-device-mode-start.sh /opt/nvidia/l4t-usb-device-mode/nv-l4t-usb-device-mode-start.sh
   chmod +x ./nv-l4t-usb-device-mode-start.sh

Customize the script program
--------------------------------
You need to modify the script to use the correct USB-UDC port.

For the AGX:

.. code-block:: console

   sed -i 's/3550000\.xudc/3550000.usb/g' nv-l4t-usb-device-mode-start.sh

For the NX:

.. code-block:: console

   sed -i 's/3550000\.usb/3550000.xudc/g' nv-l4t-usb-device-mode-start.sh


Grab bulk-mode program folder
------------------------------

Download and unzip the reference package into `~/Desktop/startup_bulk`:

.. code-block:: console

   wget https://terra-1-g.djicdn.com/71a7d383e71a4fb8887a310eb746b47f/psdk/e-port/usb-bulk-configuration-reference.zip \
     && unzip usb-bulk-configuration-reference.zip -d startup_bulk \
     && mv startup_bulk/ ~/Desktop/ \
     && chmod +x ~/Desktop/startup_bulk/*


Reboot
------

Load necessary kernel modules on boot
------------------------------------

Append to `/etc/modules` (ensure the file remains correctly formatted):

.. code-block:: console

   echo -e "configfs\nlibcomposite\nusb_f_fs\ntegra-xudc" | sudo tee -a /etc/modules

Test the setup
--------------

.. code-block:: console

   cd /opt/nvidia/l4t-usb-device-mode/ 
   ./nv-l4t-usb-device-mode-start.sh

If it works, re-enable the service
-----------------------------------

.. code-block:: console

   sudo systemctl enable /opt/nvidia/l4t-usb-device-mode/nv-l4t-usb-device-mode.service

Now both bulk mode and network mode are configured.

Prerequisites to configure any Xavier/Orin NX/AGX board to connect to PSDK, These steps are only done **every time the board boots up**
=======================================================================================================================================
.. _aerial_platform_dji_matrice_psdk_installation_prerequisites_software_every_time:   

M300
----

**Hardware**
Use the Type-C port supporting both bulk and usbnet mode

- Onboard computer = host 
- E-Port = device.

.. image:: resources/device_mode.jpg
   :alt: E-Port in device mode

- On the AGX, connections as shown:

.. image:: resources/agx_connections.jpg
   :alt: AGX USB connections

**Software**

Enable host mode on the onboard computer:

.. code-block:: console

   echo host | sudo tee /sys/class/usb_role/usb2-0-role-switch/role

You should now see `/dev/ttyACM0` when powering on the drone.

M350
----

**Hardware**
Use the Type-C port supporting both bulk and usbnet mode

- Onboard computer = device
- E-Port = host.

.. image:: resources/host_mode.jpg
   :alt: E-Port in host mode

- On the AGX, connections as shown:

.. image:: resources/agx_connections.jpg
   :alt: AGX USB connections (host)

**Software**

Enable device mode on the onboard computer:

.. code-block:: console

   echo device | sudo tee /sys/class/usb_role/usb2-0-role-switch/role

Bring up the `l4tbr0` interface:

.. code-block:: console

   sudo ifconfig usb0 192.168.1.1 netmask 255.255.255.0 up

Then verify with:

.. code-block:: console

   ifconfig

You should see an entry for `l4tbr0`. If not, repeat the above steps.

M30T
----

**Hardware**
Use the Type-C port supporting both bulk and usbnet mode

- Onboard computer = device
- E-Port = host.

.. image:: resources/host_mode.jpg
   :alt: E-Port in host mode

- On the NX, connections as shown:

.. image:: resources/nx_connections.jpg
   :alt: NX USB connections

**Software**

Enable device mode on the onboard computer:

.. code-block:: console

   echo device | sudo tee /sys/class/usb_role/usb2-0-role-switch/role

Bring up the `l4tbr0` interface:

.. code-block:: console

   sudo ifconfig usb0 192.168.1.1 netmask 255.255.255.0 up

Then verify with:

.. code-block:: console

   ifconfig

You should see an entry for `l4tbr0`. If not, repeat the above steps.

.. note::

This leaves both bulk and network mode enabled. You can switch between them by running the appropriate command. In
order to connect to the drone, either in bulk or network mode, you need to run the platform with the following JSON parameter values:

.. code-block:: json

   .
   .
   .
    "uart_config": {
      "uart1_device_name": "/dev/ttyUSB0",
      "uart2_device_enable": "true",
      "uart2_device_name": "/dev/ttyACM0"
    },
    "network_config": {
      "network_device_name": "l4tbr0",
      "network_usb_adapter_vid": "0x0B95",
      "network_usb_adapter_pid": "0x1790"
    }
   

.. _aerial_platform_dji_matrice_psdk_installation_package:

Install platform package
========================

* For binary installation, install by running:

.. code-block:: bash

   sudo apt install ros-humble-as2-platform-dji-psdk


* For source installation, follow the steps below:

.. code-block:: bash

    # If you have installed Aerostack2 from sources we recommend to clone the package in the src folder of your workspace otherwise you can clone it in any ROS 2 workspace you want.
    cd ~/aerostack2_ws/src/aerostack2/as2_aerial_platforms
    git clone git@github.com:aerostack2/as2_platform_dji_psdk.git
    cd ~/aerostack2_ws
    rosdep install as2_platform_dji_psdk --from-paths src --ignore-src -r -y
    colcon build --packages-up-to as2_platform_dji_psdk

.. _aerial_platform_dji_matrice_psdk_as2_common_interface:

---------------------------
Aerostack2 Common Interface
---------------------------

For more details about platform control modes and sensors, see :ref:`Aerostack2 Aerial Platform Concepts <as2_concepts_aerial_platform>`.



.. _aerial_platform_dji_matrice_psdk_as2_common_interface_control_modes:

Control Modes
=============

These are supported control modes:

.. list-table:: Control Modes DJI PSDK Platform
   :widths: 50 50 50
   :header-rows: 1

   * - Control Mode
     - Yaw Mode
     - Reference Frame
   * - Speed
     - Speed
     - ENU


.. _aerial_platform_dji_matrice_psdk_as2_common_interface_sensors:

Sensors
=======

These are supported sensors:
  
.. list-table:: Sensors DJI PSDK Platform
   :widths: 50 50 50
   :header-rows: 1

   * - Sensor
     - Topic
     - Type
   * - Odometry
     - sensor_measurements/odom
     - nav_msgs/Odometry
   * - IMU
     - sensor_measurements/imu
     - sensor_msgs/Imu
   * - Battery
     - sensor_measurements/battery
     - sensor_msgs/BatteryState
   * - GPS
     - sensor_measurements/gps
     - sensor_msgs/NavSatFix
   * - Camera
     - sensor_measurements/camera
     - sensor_msgs/Image


.. _aerial_platform_dji_matrice_psdk_platform_launch:

---------------
Platform Launch
---------------

Aerostack2 provides a launch file for this platform:

.. code-block:: bash

  ros2 launch as2_platform_dji_psdk as2_platform_dji_psdk.launch.py

Also, `ROS 2 PSDK Wrapper <https://github.com/umdlife/psdk_ros2>`_ must be launched before the platform:

.. code-block:: bash

  ros2 launch as2_platform_dji_psdk psdk_wrapper.launch.py

To see all the **available parameters**, use the **'-s'** flag to show the description of each parameter in the launch file.
