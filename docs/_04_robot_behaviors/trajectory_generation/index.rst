.. _behaviors_trajectory_generation_page:

Generate Polynomial Trajectory Behavior
=======================================

The ``generate_polynomial_trajectory_behavior`` is the core component of the
``as2_behaviors_trajectory_generation`` package in Aerostack2.

This behavior turns a list of waypoints into a smooth time-parameterised
trajectory and feeds it to the controller through the
``motion_reference/trajectory_setpoints`` topic. It exposes a **plugin-based
architecture** so the actual trajectory-generation algorithm can be selected
at runtime via the ``plugin_name`` launch argument, while the action server,
TF handling, yaw policy, sampling and debug topics are shared by every
plugin.

Working Principle
-----------------

The behavior is split in two layers:

* **Wrapper** (``GeneratePolynomialTrajectoryBehavior``): an
  ``as2_behavior::BehaviorServer`` that owns the action lifecycle
  (``on_activate`` / ``on_modify`` / ``on_deactivate`` / ``on_pause`` /
  ``on_resume`` / ``on_run``), converts the incoming waypoints to the
  configured ``desired_frame_id`` via TF, samples the trajectory at the
  control-cycle rate and forwards the setpoints through the
  ``as2::motionReferenceHandlers::TrajectoryMotion`` handler.
* **Plugin** (one of the available trajectory generators, see below): solves
  the actual generation problem (polynomial fitting, S-curve simulation,
  GCOPTER optimisation, …) behind a stable interface
  (``GeneratePolynomialTrajectoryBase``).

Pipeline per goal:

1. The wrapper validates the incoming waypoints, converts them to
   ``desired_frame_id`` and stores them in the pending mission queue.
2. The plugin is asked to generate (or update) the trajectory through its
   contract (``generateTrajectory`` / ``updateWaypoints``).
3. On every control cycle (``on_run``), the wrapper evaluates the plugin at
   ``trajectory_time_`` for ``sampling_n`` samples spaced by ``sampling_dt``
   and publishes a ``TrajectorySetpoints`` message.
4. When the plugin reports ``isFinished()``, the wrapper hands over to the
   hover handler and closes the action with success.

Yaw Policy
^^^^^^^^^^

The yaw goal mode is taken from the action ``yaw`` field
(``as2_msgs/YawMode``). The wrapper supports the following modes:

* ``KEEP_YAW`` — hold the current yaw measured at goal acceptance.
* ``PATH_FACING`` — point the nose along the trajectory tangent.
* ``FIXED_YAW`` — track the yaw angle provided in the goal.
* ``YAW_FROM_TOPIC`` — track yaw from the ``motion_reference/yaw`` topic.
* ``FACE_REFERENCE`` — point the nose at a reference position; rate-limited
  by ``yaw_speed_threshold``.

The thresholds ``yaw_threshold`` (m) and ``yaw_speed_threshold`` (rad/s)
gate the yaw recomputation to avoid jitter when the drone is stationary or
very close to the reference.

Pause-after-Generate (``start_on_paused``)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

When a goal is sent with ``start_on_paused: true`` the wrapper builds the
trajectory in ``on_activate`` but does **not** emit setpoints. The first
``on_run`` tick reports ``PAUSED`` and the behavior stays in that state
until the client sends ``resume``. After ``resume`` the trajectory is
sampled from ``trajectory_time_ = 0`` exactly, so the planned mission is
preserved without realignment.

Partial-Trajectory Feed (``path_length``)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For long missions the wrapper can deliver waypoints to the plugin in a
sliding window of ``path_length`` waypoints instead of feeding the full
list up front. This trades a CPU spike at start for a steady drip of
smaller re-plans while the drone flies.

* ``path_length: 0`` (default) — feeds the full mission to the plugin
  in one shot (legacy behavior).
* ``path_length: N`` — keeps at most ``N`` waypoints active in the
  plugin; consumed waypoints are replaced by the next pending one through
  ``plugin->updateWaypoints()``. Plugins that override ``updateWaypoints``
  for smooth stitching keep their internal time axis untouched; plugins
  that don't override it regenerate using the live ``vehicle_pose`` /
  ``vehicle_twist`` as boundary conditions.

In both cases ``feedback.remaining_waypoints`` reflects the full mission
queue, not the active window.

Frame-Drift Watchdog
^^^^^^^^^^^^^^^^^^^^

When ``frequency_update_frame > 0`` the wrapper checks the map↔odom
transform at that rate and triggers a regeneration when the translation
drift exceeds ``transform_threshold`` (m). This keeps the trajectory
consistent with the active state-estimation frame after re-localisations.

Degenerate-Hold
^^^^^^^^^^^^^^^

When the active waypoint list collapses to a single waypoint and that
waypoint sits closer than ``kDegenerateDistanceM`` (hard-coded to
``0.05`` m) to the current vehicle pose, the wrapper short-circuits the
plugin: no trajectory is generated, no setpoints are emitted, and the
action reports ``SUCCESS`` immediately. This avoids degenerate generations
(zero-length trajectories) when a client re-sends the current pose as a
goal, while still letting the controller hold the drone via its own
station-keeping.

Available Plugins
-----------------

The package ships four plugins that implement the
``GeneratePolynomialTrajectoryBase`` contract. Choose one at launch time
through the ``plugin_name`` argument.

1. **dynamic_mav_trajectory_generator**

   Wrapper around the
   `dynamic_trajectory_generator <https://github.com/miferco97/dynamic_trajectory_generator>`_
   library (modifiable polynomial generator on top of
   `ETH-ASL mav_trajectory_generation <https://github.com/ethz-asl/mav_trajectory_generation>`_).
   Supports **smooth online stitching**:
   ``updateWaypoints`` re-plans the pending segment without breaking the
   internal time axis, so the boundary velocity is taken from the current
   trajectory and there is no visible discontinuity. Regeneration is
   asynchronous (the new trajectory is computed in a background thread and
   swapped lazily). Recommended for long missions with frequent
   ``on_modify`` requests.

2. **mav_trajectory_generator**

   Wrapper around the
   `mav_trajectory_generation_library <https://github.com/aerostack2/mav_trajectory_generation_library>`_
   (Aerostack2 fork of
   `ETH-ASL mav_trajectory_generation <https://github.com/ethz-asl/mav_trajectory_generation>`_,
   static polynomial generator with linear or NLopt-based nonlinear
   solver). Uses the **regenerate** semantics for
   ``updateWaypoints``: every modify replans from scratch using
   ``vehicle_pose_`` / ``vehicle_twist_`` as boundary state, with
   ``Waypoint::velocity`` injecting the current twist for C\ :sup:`1`
   continuity.

3. **jerk_limited_trajectory_generator**

   Wrapper around the
   `jerk_limited_trajectory_generator_lib <https://github.com/aerostack2/jerk_limited_trajectory_generator_lib>`_
   S-curve simulator (based on the
   `PX4-Autopilot <https://github.com/PX4/PX4-Autopilot>`_ jerk-limited
   trajectory profile). Enforces per-axis jerk, plus 3D-norm acceleration and
   speed bounds. Optional L1 corner-cut smoothing through
   ``acceptance_radius`` lets the drone fly through intermediate waypoints
   without coming to a full stop. Uses **regenerate** semantics on
   ``updateWaypoints``.

4. **gcopter_trajectory_generator**

   Wrapper around the
   `gcopter_trajectory_generator_lib <https://github.com/aerostack2/gcopter_trajectory_generator_lib>`_
   optimiser (Aerostack2 fork of
   `ZJU-FAST-Lab GCOPTER <https://github.com/ZJU-FAST-Lab/GCOPTER>`_,
   with safe-flight-corridor support). Optimises a polynomial
   trajectory under hard dynamic limits (max body rate, tilt, thrust,
   velocity) and an AABB corridor anchored on each user waypoint. Uses
   **regenerate** semantics on ``updateWaypoints`` and honours the live
   ``vehicle_twist_`` as a hard initial-velocity boundary condition.

Action Interface
----------------

The behavior exposes the action ``as2_msgs/action/GeneratePolynomialTrajectory``
on ``<namespace>/TrajectoryGeneratorBehavior``.

.. list-table:: Goal
   :widths: 30 70
   :header-rows: 1

   * - Field
     - Description
   * - **stamp** (``builtin_interfaces/Time``)
     - Request timestamp.
   * - **yaw** (``as2_msgs/YawMode``)
     - Yaw goal mode (see *Yaw Policy* above).
   * - **path** (``as2_msgs/PoseStampedWithID[]``)
     - List of waypoints with id and stamped pose. Empty ids are auto-filled
       as ``waypoint_XXX``.
   * - **max_speed** (``float32``)
     - Maximum desired translation speed along the path (m/s).
   * - **start_on_paused** (``bool``)
     - If ``true``, the trajectory is generated and the behavior stays in
       ``PAUSED`` until the client sends ``resume`` (see *Pause-after-Generate*
       above). Default: ``false`` — start emitting setpoints immediately.

.. list-table:: Result
   :widths: 30 70
   :header-rows: 1

   * - Field
     - Description
   * - **trajectory_generator_success** (``bool``)
     - ``false`` if the generated trajectory could not be followed to
       completion.

.. list-table:: Feedback
   :widths: 30 70
   :header-rows: 1

   * - Field
     - Description
   * - **next_waypoint_id** (``string``)
     - Id of the next pending waypoint in the mission queue.
   * - **remaining_waypoints** (``uint16``)
     - Number of waypoints in the mission queue that have not yet been
       consumed (counts the full mission, not the active window when
       ``path_length`` is enabled).

Configuration Parameters
------------------------

The configuration is split in two layers: the **wrapper YAML** with shared
parameters and one **plugin YAML** per backend with algorithm-specific keys.

Wrapper Parameters
^^^^^^^^^^^^^^^^^^

Defaults in
``generate_polynomial_trajectory_behavior/config/config_default.yaml``.

.. list-table:: Wrapper Parameters
   :widths: 30 15 55
   :header-rows: 1

   * - Parameter
     - Default
     - Description
   * - **desired_frame_id**
     - ``odom``
     - TF frame in which the trajectory is generated and evaluated. Incoming
       waypoints are converted to this frame before being fed to the plugin.
   * - **sampling_n**
     - ``1``
     - Number of trajectory samples emitted per control cycle. Must be ≥ 1.
   * - **sampling_dt**
     - ``0.01``
     - Time step between consecutive samples (s).
   * - **path_length**
     - ``0``
     - Active waypoint window fed to the plugin. ``0`` disables the sliding
       window and feeds the full mission up front. ``> 0`` enables
       incremental feeding (see *Partial-Trajectory Feed* above).
   * - **yaw_threshold**
     - ``0.1``
     - Minimum planar distance (m) before recomputing yaw to avoid jitter
       at low displacement.
   * - **yaw_speed_threshold**
     - ``0.0``
     - Maximum yaw rate (rad/s) for the ``FACE_REFERENCE`` mode. ``0.0``
       disables the rate limit.
   * - **frequency_update_frame**
     - ``0.0``
     - Map↔odom drift watchdog rate (Hz). ``0.0`` disables the watchdog.
   * - **transform_threshold**
     - ``1.0``
     - Translation drift (m) on the watched transform that triggers a
       regeneration. ``0.0`` disables the drift check.
   * - **debug.path_topic**
     - ``debug/behaviors/trajectory_generation/trajectory/generated``
     - Topic where the sampled generated trajectory is republished as
       ``nav_msgs/Path``. Empty disables the publisher.
   * - **debug.reference_setpoint**
     - ``debug/behaviors/trajectory_generation/reference/waypoint``
     - Topic for the current reference point as ``visualization_msgs/Marker``.
   * - **debug.reference_end_waypoint**
     - ``debug/behaviors/trajectory_generation/reference/end_waypoint``
     - Topic for the end-of-horizon point as ``visualization_msgs/Marker``.
   * - **debug.reference_waypoints**
     - ``debug/behaviors/trajectory_generation/trajectory/waypoints``
     - Topic for the full waypoint queue as
       ``visualization_msgs/MarkerArray``.
   * - **debug.generation_time_topic**
     - ``debug/behaviors/trajectory_generation/generation_time``
     - Topic where the wall-clock time (s) spent in the last plugin call to
       ``generateTrajectory`` / ``updateWaypoints`` is published as
       ``std_msgs/Float64``. Empty disables the publisher.

mav_trajectory_generator Parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Defaults in
``plugins/mav_trajectory_generator/config/mav_trajectory_generator.yaml``,
under the namespace ``mav_trajectory_generator.optimization.*``.

.. list-table:: mav_trajectory_generator Parameters
   :widths: 35 15 50
   :header-rows: 1

   * - Parameter
     - Default
     - Description
   * - **derivative_to_optimize**
     - ``2``
     - Derivative order minimised by the cost: ``0`` POS, ``1`` VEL,
       ``2`` ACC, ``3`` JERK, ``4`` SNAP.
   * - **solver**
     - ``nonlinear``
     - ``linear`` (closed-form QP) or ``nonlinear`` (NLopt with hard
       constraints).
   * - **a_max**
     - ``9.81``
     - Maximum acceleration norm enforced by the nonlinear solver
       (m/s\ :sup:`2`).
   * - **nl_max_iterations**
     - ``2000``
     - NLopt iteration cap.
   * - **nl_f_rel**
     - ``0.05``
     - NLopt relative cost tolerance.
   * - **nl_x_rel**
     - ``0.1``
     - NLopt relative parameter tolerance.
   * - **nl_time_penalty**
     - ``1000.0``
     - Penalty weight on total trajectory time.
   * - **nl_initial_stepsize_rel**
     - ``0.1``
     - Initial NLopt step size (relative).
   * - **nl_inequality_constraint_tolerance**
     - ``0.2``
     - Tolerance on the inequality (acceleration) constraint.

dynamic_mav_trajectory_generator Parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The dynamic plugin currently exposes no per-instance ROS parameters; the
underlying ``dynamic_trajectory_generator`` library is configured with its
own internal defaults. The plugin honours all the wrapper parameters
described above.

jerk_limited_trajectory_generator Parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Defaults in
``plugins/jerk_limited_trajectory_generator/config/jerk_limited_trajectory_generator.yaml``,
under the namespace ``jerk_limited_trajectory_generator.{limits,simulator}.*``.

The action goal's ``max_speed`` transiently overrides ``limits.max_speed``
for a single call.

.. list-table:: jerk_limited_trajectory_generator Parameters
   :widths: 35 15 50
   :header-rows: 1

   * - Parameter
     - Default
     - Description
   * - **limits.max_speed**
     - ``1.0``
     - Maximum 3D speed norm (m/s). Set ≤ 0 to disable.
   * - **limits.max_acceleration**
     - ``10.0``
     - Maximum 3D acceleration norm (m/s\ :sup:`2`). Set ≤ 0 to disable.
   * - **limits.max_jerk**
     - ``20.0``
     - Maximum per-axis jerk (m/s\ :sup:`3`). Set ≤ 0 to disable. At least
       one of ``max_speed`` / ``max_acceleration`` / ``max_jerk`` must
       remain > 0.
   * - **limits.acceptance_radius**
     - ``0.01``
     - Radius (m) for the L1 corner-cut at intermediate waypoints. ``0``
       forces a full stop at every intermediate waypoint.
   * - **limits.final_acceptance_radius**
     - ``0.01``
     - Position tolerance at the last waypoint (m). ≤ 0 reuses
       ``acceptance_radius``.
   * - **limits.trajectory_gain**
     - ``0.1``
     - Scales acceleration when computing the corner-speed cap.
   * - **simulator.step_dt**
     - ``0.01``
     - Internal integration step (s).
   * - **simulator.max_simulation_time**
     - ``600.0``
     - Hard horizon (s) for the offline simulation; trajectories that exceed
       it fail to generate.
   * - **simulator.settle_velocity**
     - ``0.05``
     - Velocity norm (m/s) below which the drone is considered at rest.
   * - **simulator.advance_radius_min**
     - ``0.1``
     - Floor (m) for the per-waypoint advance threshold. The effective
       threshold is ``max(acceptance_radius, advance_radius_min)``.
   * - **simulator.polynomial_order**
     - ``16``
     - Number of polynomial coefficients per axis and segment in the
       resulting spline.

gcopter_trajectory_generator Parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Defaults in
``plugins/gcopter_trajectory_generator/config/gcopter_trajectory_generator.yaml``,
under the namespace
``gcopter_trajectory_generator.{drone,limits,optimization,waypoints}.*``.

The action goal's ``max_speed`` transiently overrides ``limits.max_velocity``
for a single call.

.. list-table:: gcopter_trajectory_generator Parameters
   :widths: 35 15 50
   :header-rows: 1

   * - Parameter
     - Default
     - Description
   * - **drone.mass**
     - ``1.0``
     - Vehicle mass (kg) used by the flatness model.
   * - **drone.gravity**
     - ``9.81``
     - Gravity acceleration (m/s\ :sup:`2`).
   * - **drone.horizontal_drag**
     - ``0.1``
     - Horizontal drag coefficient.
   * - **drone.vertical_drag**
     - ``0.1``
     - Vertical drag coefficient.
   * - **drone.parasitic_drag**
     - ``0.01``
     - Parasitic drag coefficient.
   * - **drone.speed_smooth_factor**
     - ``0.01``
     - Smoothing factor on the speed term in the flatness model.
   * - **limits.max_velocity**
     - ``5.0``
     - Maximum velocity norm (m/s).
   * - **limits.max_body_rate**
     - ``6.0``
     - Maximum body rate (rad/s).
   * - **limits.max_tilt_angle**
     - ``0.785``
     - Maximum tilt angle (rad).
   * - **limits.min_thrust**
     - ``0.1``
     - Minimum collective thrust (N).
   * - **limits.max_thrust**
     - ``30.0``
     - Maximum collective thrust (N).
   * - **optimization.time_weight**
     - ``512.0``
     - Weight on the total trajectory time term.
   * - **optimization.position_weight**
     - ``10000.0``
     - Weight on position constraint violations.
   * - **optimization.velocity_weight**
     - ``10000.0``
     - Weight on velocity constraint violations.
   * - **optimization.body_rate_weight**
     - ``10000.0``
     - Weight on body-rate constraint violations.
   * - **optimization.tilt_weight**
     - ``10000.0``
     - Weight on tilt-angle constraint violations.
   * - **optimization.thrust_weight**
     - ``10000.0``
     - Weight on thrust constraint violations.
   * - **optimization.smoothing_eps**
     - ``0.01``
     - Smoothing epsilon for the soft-constraint penalty.
   * - **optimization.integral_resolution**
     - ``16``
     - Number of integration samples per segment for the constraint penalty.
   * - **optimization.rel_cost_tol**
     - ``0.001``
     - L-BFGS relative cost tolerance.
   * - **optimization.corridor_margin**
     - ``0.1``
     - AABB half-margin (m) used between anchors and on segments without
       intermediate waypoints.
   * - **optimization.vertical_perturbation**
     - ``0.05``
     - Initial-state in-plane offset (m) along +x applied as a workaround
       for the L-BFGS rank-deficiency on world-Z aligned segments (every
       waypoint of the active segment sits inside the vertical-alignment
       band defined below). ``0.0`` disables the workaround.
   * - **optimization.vertical_alignment_threshold**
     - ``0.10``
     - Planar distance band (m) under which all waypoints in the active
       segment are considered vertically aligned with the vehicle, which
       triggers the perturbation above.
   * - **waypoints.waypoint_margin**
     - ``0.1``
     - AABB half-margin (m) on the two pinch segments around each
       intermediate waypoint.
   * - **waypoints.waypoint_anchor_radius**
     - ``0.5``
     - Offset (m) of each anchor along the path. ≤ 0 disables the anchoring
       and lets the solver cut corners.

Launching the Behavior
----------------------

The plugin to load is selected through the ``plugin_name`` launch
argument. The set of valid choices is auto-populated from the registered
plugins via ``pluginlib``, so an invalid value is rejected at launch time.

**Standalone node**

.. code-block:: bash

   ros2 launch as2_behaviors_trajectory_generation generate_polynomial_trajectory_behavior_launch.py \
       namespace:=<drone_namespace> \
       plugin_name:=dynamic_mav_trajectory_generator

Launch arguments:

* ``namespace`` — drone namespace (defaults to the
  ``AEROSTACK2_SIMULATION_DRONE_ID`` environment variable).
* ``plugin_name`` — one of
  ``dynamic_mav_trajectory_generator``, ``mav_trajectory_generator``,
  ``jerk_limited_trajectory_generator``, ``gcopter_trajectory_generator``.
  If left empty, the launch tries to read it from ``config_file``; if it
  is also missing there, the launch fails.
* ``config_file`` — path to the wrapper YAML (defaults to
  ``config_default.yaml``). The plugin YAML is auto-located in
  ``plugins/<plugin_name>/config/<plugin_name>.yaml``.
* ``log_level`` — node log level (default ``info``).
* ``use_sim_time`` — use the simulation clock if ``true`` (default
  ``false``).

**Composable node**

.. code-block:: bash

   ros2 launch as2_behaviors_trajectory_generation composable_generate_polynomial_trajectory_behavior.launch.py \
       namespace:=<drone_namespace> \
       plugin_name:=dynamic_mav_trajectory_generator \
       container:=<existing_container_name>

If ``container`` is empty a new container is created; otherwise the
behavior is loaded into the existing one as a
``rclcpp_components`` composable node.

**Sending a goal manually**

.. code-block:: bash

   ros2 action send_goal /<namespace>/TrajectoryGeneratorBehavior \
       as2_msgs/action/GeneratePolynomialTrajectory \
       "{
         yaw: {mode: 0, angle: 0.0},
         path: [
           {id: 'wp_0', pose: {header: {frame_id: 'earth'},
                               pose: {position: {x: 1.0, y: 0.0, z: 1.0}}}},
           {id: 'wp_1', pose: {header: {frame_id: 'earth'},
                               pose: {position: {x: 2.0, y: 1.0, z: 1.5}}}}
         ],
         max_speed: 1.0,
         start_on_paused: false
       }"

Set ``start_on_paused: true`` to generate the trajectory and keep the
behavior in ``PAUSED`` until the action client sends ``resume``.

Debugging Topics
----------------

When the corresponding ``debug.*`` parameter is non-empty the wrapper
publishes:

* ``<namespace>/debug/behaviors/trajectory_generation/trajectory/generated``
  (``nav_msgs/Path``) — sampled trajectory currently held by the plugin.
  Refreshed on every regeneration and, for asynchronous plugins, on the
  cycle where the new backend trajectory is swapped in.
* ``<namespace>/debug/behaviors/trajectory_generation/reference/waypoint``
  (``visualization_msgs/Marker``) — current reference point being commanded.
* ``<namespace>/debug/behaviors/trajectory_generation/reference/end_waypoint``
  (``visualization_msgs/Marker``) — end-of-horizon point of the active
  sampling window.
* ``<namespace>/debug/behaviors/trajectory_generation/trajectory/waypoints``
  (``visualization_msgs/MarkerArray``) — pending mission waypoints.
* ``<namespace>/debug/behaviors/trajectory_generation/generation_time``
  (``std_msgs/Float64``) — wall-clock time (s) spent in the last plugin call
  to ``generateTrajectory`` / ``updateWaypoints``.

See also
--------

* :ref:`development_tutorials_trajectory_generation_plugin` — step-by-step
  guide to writing a new trajectory-generation plugin against the
  ``GeneratePolynomialTrajectoryBase`` contract.
