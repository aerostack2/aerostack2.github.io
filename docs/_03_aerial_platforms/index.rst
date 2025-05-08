.. _aerial_platforms:

================
Aerial Platforms
================

Aerostack2 is platform-independent, what`s means that you can use any aerial platform that you want for a specific application. That`s allows you to use the same motion controller, state estimator and behaviors for different aerial platforms. For example, you can use the same modules for a quadrotor and a hexarotor aircrafts. However, you need to implement the interface for the new platform. Also, that implies that Sim2Real is very accurate, using same modules for simulation and real platforms.

Currently, the following aerial platforms are supported:

.. toctree::
  :maxdepth: 1

  _gazebo_simulation/index.rst
  _multirotor_simulator/index.rst
  _crazyflie/index.rst
  _ryze_tello/index.rst
  _pixhawk/index.rst
  _mavlink/index.rst
  _dji_osdk/index.rst
  _dji_psdk/index.rst
  _betaflight/index.rst
