.. _behaviors_follow_reference_page:

Follow Reference Behavior
=========================

The ``follow_reference_behavior`` is part of the
``as2_behaviors_motion`` package in Aerostack2.

This behavior keeps the aircraft tracking a 3D reference point given in any TF
frame and stays running until the action client cancels it. It exposes a
**plugin-based architecture** so the actual tracking strategy can be selected
at runtime via the ``plugin_name`` launch argument, while the action server,
goal validation, TF resolution and platform-state monitoring are shared by
every plugin.

Working Principle
-----------------

The behavior is split in two layers:

* **Wrapper** (``FollowReferenceBehavior``): an
  ``as2_behavior::BehaviorServer`` that owns the action lifecycle
  (``on_activate`` / ``on_modify`` / ``on_deactivate`` / ``on_pause`` /
  ``on_resume`` / ``on_run``), validates the incoming goal (the platform
  must be ``FLYING`` and localization must be available), looks the target
  frame up in TF and substitutes any zero-valued ``max_speed_*`` field with
  the matching default parameter before handing the goal over to the plugin.
* **Plugin** (one of the available backends, see below): generates the
  actual reference and pushes it to the controller through the
  ``as2_motion_reference_handlers`` API. The contract is
  ``follow_reference_base::FollowReferenceBase``.

Pipeline per goal:

1. The wrapper resolves the target frame, fills missing speed limits with the
   per-axis defaults and forwards the goal to the plugin.
2. The plugin's ``own_activate`` accepts the goal and starts publishing the
   reference (a position setpoint or a generated trajectory, depending on the
   backend).
3. Every control cycle the wrapper calls ``on_run`` on the plugin, which
   either emits a fresh reference or reports a status transition. In normal
   operation the behavior stays in ``RUNNING`` until the action client
   deactivates it.
4. On cancel / deactivate the plugin commands a hover and the action closes.

The base class also exposes ``state_callback`` and ``platform_info_callback``,
called by the wrapper from the ``self_localization/twist`` and
``platform/info`` subscriptions, so every plugin sees the same localization
and platform state without re-subscribing.

Available Plugins
-----------------

The package ships two plugins that implement the
``FollowReferenceBase`` contract. Choose one at launch time through the
``plugin_name`` argument.

1. **follow_reference_plugin_position**

   Tracks the reference by sending **position setpoints** to the controller
   through the ``as2::motionReferenceHandlers::PositionMotion`` handler. The
   setpoint is always emitted in ``earth``: the live target is re-resolved to
   ``earth`` every control cycle, which keeps the commanded orientation
   decoupled from any rotation in the (possibly moving) goal frame.

   Pause / resume are honoured by sending a hover while paused. Recommended
   when the controller already produces smooth motion and the platform can
   handle hard target switches.

2. **follow_reference_plugin_trajectory**

   Delegates the reference tracking to the
   :ref:`behaviors_trajectory_generation_page` running in
   ``follow_reference_mode``. The plugin sends a single-waypoint goal to the
   trajectory generator and forwards goal modifications through the
   ``motion_reference/modify_waypoint`` topic. This gives smooth, jerk- or
   dynamics-aware tracking at the cost of one extra hop through the
   trajectory generator.

   The plugin also runs a **reactive-modify loop**: every cycle it re-resolves
   the goal target to ``earth`` and republishes a ``modify_waypoint`` message
   when the target has drifted more than ``modify_threshold`` from the last
   published one, rate-limited by ``modify_frequency``. For static target
   frames the converted point stays constant and no extra messages are
   emitted; for moving target frames (e.g. another drone's body frame) this
   keeps the trajectory aligned with the live target without action
   re-issuing.

   Because the trajectory generator accepts a single scalar ``max_speed``,
   the plugin uses ``max(max_speed_x, max_speed_y, max_speed_z)`` and lets
   the generator distribute that bound across the polynomial axes.

Yaw Policy
----------

The yaw goal is taken from the action ``yaw`` field
(``as2_msgs/YawMode``). Supported modes depend on the active plugin:

.. list-table:: Yaw modes supported per plugin
   :widths: 30 35 35
   :header-rows: 1

   * - Mode
     - ``follow_reference_plugin_position``
     - ``follow_reference_plugin_trajectory``
   * - ``KEEP_YAW``
     - Yes — refreshed every control cycle from the current quaternion.
     - Yes — delegated to the trajectory generator.
   * - ``FIXED_YAW``
     - Yes — uses ``yaw.angle`` from the goal.
     - Yes — delegated to the trajectory generator.
   * - ``PATH_FACING``
     - Yes — aims the nose from the current position toward the target in
       the xy plane; falls back to ``KEEP_YAW`` when the planar distance is
       below 0.1 m to avoid jitter at the reference.
     - Yes — delegated to the trajectory generator.
   * - ``YAW_TO_FRAME``
     - Yes — yaws to look at the goal frame origin.
     - Yes — delegated to the trajectory generator.
   * - ``YAW_FROM_TOPIC``
     - **Not supported** — the position plugin rejects the goal.
     - Yes — handled by the trajectory generator.

Action Interface
----------------

The behavior exposes the action ``as2_msgs/action/FollowReference`` on
``<namespace>/FollowReferenceBehavior``.

.. list-table:: Goal
   :widths: 30 70
   :header-rows: 1

   * - Field
     - Description
   * - **yaw** (``as2_msgs/YawMode``)
     - Yaw goal mode (see *Yaw Policy* above).
   * - **target_pose** (``geometry_msgs/PointStamped``)
     - Reference point. The frame is resolved through TF; an empty
       ``frame_id`` is rejected.
   * - **max_speed_x** / **max_speed_y** / **max_speed_z** (``float32``)
     - Maximum tracking speed per earth axis (m/s). A value of ``0`` falls
       back to the corresponding ``follow_reference_max_speed_*`` parameter.

.. list-table:: Result
   :widths: 30 70
   :header-rows: 1

   * - Field
     - Description
   * - **follow_reference_success** (``bool``)
     - ``false`` if the reference could not be tracked to a stable state.

.. list-table:: Feedback
   :widths: 30 70
   :header-rows: 1

   * - Field
     - Description
   * - **actual_speed** (``float32``)
     - Current linear speed norm (m/s).
   * - **actual_distance_to_goal** (``float32``)
     - Euclidean distance from the current position to the target, both
       expressed in ``earth`` (m).

Configuration Parameters
------------------------

The configuration is split in two layers: the **wrapper YAML** with
parameters shared by every plugin, and a **plugin-specific block** for the
trajectory backend.

Wrapper Parameters
^^^^^^^^^^^^^^^^^^

Defaults in ``follow_reference_behavior/config/config_default.yaml``.

.. list-table:: Wrapper Parameters
   :widths: 35 15 50
   :header-rows: 1

   * - Parameter
     - Default
     - Description
   * - **follow_reference_max_speed_x**
     - ``10.0``
     - Default maximum tracking speed along the x axis (m/s). Used when the
       goal's ``max_speed_x`` is ``0``.
   * - **follow_reference_max_speed_y**
     - ``10.0``
     - Default maximum tracking speed along the y axis (m/s). Used when the
       goal's ``max_speed_y`` is ``0``.
   * - **follow_reference_max_speed_z**
     - ``10.0``
     - Default maximum tracking speed along the z axis (m/s). Used when the
       goal's ``max_speed_z`` is ``0``.
   * - **tf_timeout_threshold**
     - ``0.05``
     - Timeout (s) consumed by the internal ``as2::tf::TfHandler`` for every
       TF lookup performed by the behavior (target resolution, state
       conversion).

follow_reference_plugin_trajectory Parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Read only when the trajectory plugin is loaded, under the namespace
``follow_reference_plugin_trajectory.*``.

.. list-table:: follow_reference_plugin_trajectory Parameters
   :widths: 35 15 50
   :header-rows: 1

   * - Parameter
     - Default
     - Description
   * - **modify_threshold**
     - ``0.0``
     - Translation (m) of the target in ``earth`` that triggers a fresh
       ``modify_waypoint`` publication. ``0`` re-publishes on every detected
       change.
   * - **modify_frequency**
     - ``0.0``
     - Maximum rate (Hz) at which the reactive-modify loop re-publishes
       ``modify_waypoint``. ``0`` disables the rate limit.

Launching the Behavior
----------------------

The plugin to load is selected through the ``plugin_name`` launch argument.
The set of valid choices is auto-populated from the registered plugins via
``pluginlib``, so an invalid value is rejected at launch time.

.. code-block:: bash

   ros2 launch as2_behaviors_motion follow_reference_behavior_launch.py \
       namespace:=<drone_namespace> \
       plugin_name:=follow_reference_plugin_position

Launch arguments:

* ``namespace`` — drone namespace (defaults to the
  ``AEROSTACK2_SIMULATION_DRONE_ID`` environment variable).
* ``plugin_name`` — one of ``follow_reference_plugin_position``,
  ``follow_reference_plugin_trajectory``. The list is enforced by
  ``get_available_plugins('as2_behaviors_motion', 'follow_reference')`` at
  launch time.
* ``behavior_config_file`` — path to the behavior YAML (defaults to
  ``config_default.yaml``).
* ``log_level`` — node log level (default ``info``).
* ``use_sim_time`` — use the simulation clock if ``true`` (default
  ``false``).

Sending a goal manually
^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   ros2 action send_goal /<namespace>/FollowReferenceBehavior \
       as2_msgs/action/FollowReference \
       "{
         yaw: {mode: 0, angle: 0.0},
         target_pose: {header: {frame_id: 'earth'},
                       point: {x: 1.0, y: 0.0, z: 1.0}},
         max_speed_x: 1.0,
         max_speed_y: 1.0,
         max_speed_z: 1.0
       }"

The behavior stays in ``RUNNING`` until the client cancels the action. While
running, a fresh goal can be sent through ``on_modify`` (action client's
``modify`` request) to retarget without losing the active reference.

Notes
-----

* The trajectory plugin re-uses the trajectory generator behavior at
  ``<namespace>/TrajectoryGeneratorBehavior``, so both behaviors must be
  available (and launched) when ``plugin_name :=
  follow_reference_plugin_trajectory`` is selected.
* The base class rejects activation if the platform is not ``FLYING`` or if
  no localization has been received yet (no twist message on
  ``self_localization/twist``).
