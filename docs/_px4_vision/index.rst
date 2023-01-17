PX4 Vision
==========

This is a complete tutorial for installing and running AeroStack2 in the
`PX4-Vision v1.5 <https://docs.px4.io/main/en/complete_vehicles/px4_vision_kit.html>`

Basic Setup
***********

The first step is to install Ubuntu 22.04 on the Carrier Board from a `bootable USB <https://ubuntu.com/tutorials/create-a-usb-stick-on-ubuntu#1-overview>`. Connect a HDMI and a Keyboard to the Carrier Board and follow the Ubuntu install instructions.



Structure Core Setup
********************

In order to use the Structure Core Camera that comes in the drone you must follow the subsequent steps:

1. Download Structure SDK from https://developer.structure.io/sdk . We have tested ``StructureSDK-CrossPlatform-0.9``
2. Unzip content in ``$HOME`` directory 
3. Change directory into the uncompressed folder and run: 

.. code-block:: bash

  chmod +x DriverAndFirmware/Linux/Install-CoreDriver-Udev-Linux.sh
  sudo DriverAndFirmware/Linux/Install-CoreDriver-Udev-Linux.sh # This will allow the device to be detected by the computer
  

4. After that we will build the library with

.. code-block:: bash

  mkdir build
  cd build
  cmake -G'Unix Makefiles' -DCMAKE_BUILD_TYPE=Release ..
  make Samples
