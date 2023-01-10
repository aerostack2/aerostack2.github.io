PX4
===
Aditional dependencies
######################

In order to be able to use a Pixhawk autopilot some additional software must be installed:
This documentation is obtained from ( ADD LINK )


Fast-DDS-Gen
**********************


.. code-block:: bash

  sudo apt install -y openjdk-11-jdk
  
.. code-block:: bash

  git clone --recursive https://github.com/eProsima/Fast-DDS-Gen.git -b v1.0.4 \
      && cd Fast-DDS-Gen \
      && ./gradlew assemble \
      && sudo ./gradlew install

In order to validate the installation in another terminal run:

.. code-block:: bash

  # Check FastRTPSGen version
  fastrtpsgen_version_out=""
  if [[ -z $FASTRTPSGEN_DIR ]]; then
    fastrtpsgen_version_out="$FASTRTPSGEN_DIR/$(fastrtpsgen -version)"
  else
    fastrtpsgen_version_out=$(fastrtpsgen -version)
  fi
  if [[ -z $fastrtpsgen_version_out ]]; then
    echo "FastRTPSGen not found! Please build and install FastRTPSGen..."
    exit 1
  else
    fastrtpsgen_version="${fastrtpsgen_version_out: -5:-2}"
    if ! [[ $fastrtpsgen_version =~ ^[0-9]+([.][0-9]+)?$ ]] ; then
      fastrtpsgen_version="1.0"
      [ ! -v $verbose ] && echo "FastRTPSGen version: ${fastrtpsgen_version}"
    else
      [ ! -v $verbose ] && echo "FastRTPSGen version: ${fastrtpsgen_version_out: -5}"
    fi
  fi

```

PX4-ROS-COM
**********************

.. code-block:: bash
  
  cd ~/aerostack2_ws/src
  git clone https://github.com/aerostack2/px4_ros_com 
  cd ~/aerostack2_ws && colcon build --symlink-install 
  # if using as2 CLI run $ as2 build 




PX4 firmware update
###################

TBD

ADITIONAL PARAMETERS
####################
TBD
