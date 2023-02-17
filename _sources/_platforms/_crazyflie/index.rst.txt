Bitcraze Crazyflie
==================

First thing to do is to configure your Crazyflie(s). In order to do that, please follow these steps:

* Install the `cfclient <https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/installation/install/>`_.
* Setup `UDEV permissions <https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/installation/usb_permissions/>`_ for using the Crazyradio.
* Perform a `firmware upgrade <https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/userguides/userguide_client/#firmware-upgrade>`_. For this step, if two or more crazyflies are going to be used,
  please use a different radio address for each crazyflie. 

These steps are meant to be done once. 

Platform Setup
##############

In order to map the adresses of the crazyflie(s) we just configured to the input adresses of the platfom, we need to use a configuration ``.yaml`` configuration file with the format as the following example:

.. code-block:: yaml

    cf0:
        uri: radio://0/80/2M/E7E7E7E700
    cf1:
        uri: radio://0/80/2M/E7E7E7E701

In this case, we are mapping two Crazyflies with uri's ``radio://0/80/2M/E7E7E7E700`` and ``radio://0/80/2M/E7E7E7E701`` for namespaces ``cf0`` and ``cf1`` respectively.
As you can see for the radio id part of the uri, the same Crazyradio is going to be used for both. 

This configuration file's path shall serve as an input to the platform launch parameter ``swarm_config_file``. Note that this platform is launched only once, regardless of the number of crazyflies beeing used. 
