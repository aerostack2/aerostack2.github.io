PX4
===

Since PX4 has a lot of external dependencies we opted to separate it in a diferent repo.
The complete installations step are as follows:


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


The expected output is something similar to:

.. code-block:: bash 

  openjdk version "13.0.7" 2021-04-20
  OpenJDK Runtime Environment (build 13.0.7+5-Ubuntu-0ubuntu120.04)
  OpenJDK 64-Bit Server VM (build 13.0.7+5-Ubuntu-0ubuntu120.04, mixed mode)


pyros-genmsg
**********************
A pyros-genmsg dependency is needed

.. code-block:: bash

  pip3 install pyros-genmsg
  
as2_platform_pixhawk
**********************
We recommend to have this in a diferent colcon_ws since px4_msgs packages spends a lot of time in compiling and we only want to do it once.

.. code-block:: bash

  mkdir -p ~/px4_ws/src && cd ~/px4_ws/src
  git clone git@github.com:aerostack2/as2_platform_pixhawk.git
  vcs import --recursive < as2_platform_pixhawk/dependencies.repos
  cd ~/px4_ws
  colcon build --symlink-install
 
>> Remind to ``$ source ~/aerostack2_ws/install/setup.bash`` before runing ``colcon build``

PX4 firmware update
###################

TBD

ADITIONAL PARAMETERS
####################
TBD
