.. _ros2_common_interfaces_state_estimator:

=================
State Estimator
=================

.. contents:: Contents
   :depth: 2
   :local:



.. _ros2_common_interfaces_state_estimator_topics:

------
Topics
------

* State Estimator publishes the platform localization.

.. list-table:: Aerostack2 State Estimator Topics
   :widths: 50 50 50 50
   :header-rows: 1

   * - Name
     - Type
     - Quality of Service
     - Description
   * - self_localization/pose
     - geometry_msgs/PoseStamped
     - SensorDataQoS
     - | Localization of the multirotor
       | specifying x, y and z (m: meters) and
       | roll, pitch and yaw (rad: radians)
   * - self_localization/twist
     - geometry_msgs/TwistStamped
     - SensorDataQoS
     - | Localization of the multirotor
       | specifying vx, vy and vz (m/s: meters per seconds) and
       | roll rate, pitch rate and yaw rate (rad/s: radians per seconds)

The pose is expressed in the ``earth`` frame and the twist in the ``base_link`` frame.

Which topics the State Estimator **subscribes** to depends entirely on the plugin that is
loaded. See the :ref:`State Estimator plugins <as2_plugins_state_estimator>`.



.. _ros2_common_interfaces_state_estimator_tf:

--
TF
--

.. list-table:: Aerostack2 State Estimator Transforms
   :widths: 30 70
   :header-rows: 1

   * - Name
     - Description
   * - /tf
     - | The ``map`` to ``odom`` and ``odom`` to ``base_link`` transforms,
       | and ``earth`` to ``map`` when it is not published as a static transform
   * - /tf_static
     - | The ``earth`` to ``map`` transform, which is normally set once at startup

The layout of the chain is described in
:ref:`State Estimator, Frames <as2_concepts_state_estimator_frames>`.



.. _ros2_common_interfaces_state_estimator_debug_topics:

------------
Debug Topics
------------

Every loaded plugin also publishes its own view of the state, before the authority check of
the node. These topics show what a plugin reports, which is not necessarily what the node
ends up publishing.

.. list-table:: Aerostack2 State Estimator Debug Topics
   :widths: 40 30 30
   :header-rows: 1

   * - Name
     - Type
     - Description
   * - state_estimation/<plugin_name>/pose
     - geometry_msgs/PoseStamped
     - The pose reported by that plugin
   * - state_estimation/<plugin_name>/twist
     - geometry_msgs/TwistStamped
     - The twist reported by that plugin

Their rate is set by the ``<plugin_name>.debug_publish_hz`` parameter: ``-1.0`` publishes on
every update, ``0.0`` disables the topics, and a positive value sets a fixed rate.



.. _ros2_common_interfaces_state_estimator_services:

--------
Services
--------

These services are offered by the
:ref:`Raw Odometry <as2_plugins_state_estimator_raw_odometry>` plugin when it runs with
``use_gps: true``. They deal with the geodetic datum that anchors the ``earth`` frame.

.. list-table:: Aerostack2 State Estimator Services
   :widths: 25 30 45
   :header-rows: 1

   * - Name
     - Type
     - Description
   * - set_origin
     - as2_msgs/srv/SetOrigin
     - | Set the geodetic datum of the ``earth`` frame.
       | The datum can only be established once, so a call made
       | after one already exists is answered with ``success: false``
   * - get_origin
     - as2_msgs/srv/GetOrigin
     - | Query the geodetic datum. It answers as soon as a datum
       | exists, whichever way it was established
