.. _ros2_common_interfaces:

=======================
ROS 2 Common Interfaces
=======================

Aerostack2 uses common ROS 2 interfaces like **topics**, **services** and **actions**.

This page describes common ROS 2 interfaces for inter-process communication used by Aerostack2. The goal of this set of interfaces is to facilitate the interoperability of the ROS 2 nodes in Aerostack2.

.. toctree::
   :maxdepth: 1

   aerial_platform/index.rst
   motion_controller/index.rst
   state_estimator/index.rst
   behaviors/index.rst



.. _ros2_common_interfaces_using_as2_interfaces:

--------------------
Using AS2 Interfaces
--------------------

AS2 interfaces names **should not** be typed manually. Names are stored in:

.. code-block:: cpp

 #include "as2_core/names/topics.hpp"
 #include "as2_core/names/services.hpp"
 #include "as2_core/names/actions.hpp"

You can access to a interface name via `as2_names`, e.g. `as2_names::topics::platform::info`.


