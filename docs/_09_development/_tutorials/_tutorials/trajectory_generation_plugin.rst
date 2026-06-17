.. _development_tutorials_trajectory_generation_plugin:

================================================
Writing a New Trajectory Generation Plugin
================================================

.. contents:: Table of Contents
   :depth: 1
   :local:



.. _development_tutorials_trajectory_generation_plugin_overview:

--------
Overview
--------

This tutorial shows how to add your own trajectory-generation algorithm to
Aerostack2 as a new plugin of the
:ref:`Generate Polynomial Trajectory Behavior <behaviors_trajectory_generation_page>`.

The behavior follows a wrapper + plugin pattern: the
``GeneratePolynomialTrajectoryBehavior`` server owns the action lifecycle,
TF, yaw policy, sampling, debug topics and the partial-trajectory
machinery; the plugin only owns the trajectory math behind a stable
interface (``GeneratePolynomialTrajectoryBase``). Adding a new generator
therefore boils down to implementing that interface, registering the
plugin with ``pluginlib`` and providing a YAML with the backend
parameters — the wrapper, the launch files and the action server are
reused as-is.

The reference implementations shipped with Aerostack2 live in
``as2_behaviors_trajectory_generation/generate_polynomial_trajectory_behavior/plugins/``
on `GitHub
<https://github.com/aerostack2/aerostack2/tree/main/as2_behaviors/as2_behaviors_trajectory_generation>`_:

* ``dynamic_mav_trajectory_generator`` — smooth-stitching async backend.
* ``mav_trajectory_generator`` — ETH-ASL static polynomial.
* ``jerk_limited_trajectory_generator`` — per-axis S-curve simulator.
* ``gcopter_trajectory_generator`` — GCOPTER with safe-flight-corridor.

Use them as templates while following the steps below.



.. _development_tutorials_trajectory_generation_plugin_requirements:

------------
Requirements
------------

- ROS 2 Humble
- Aerostack2
- ``as2_behaviors_trajectory_generation`` available in your workspace
- ``pluginlib``



.. _development_tutorials_trajectory_generation_plugin_steps:

--------------
Tutorial Steps
--------------



.. _development_tutorials_trajectory_generation_plugin_steps_base:

1. Plugin Base Class
====================

All plugins inherit from
``generate_polynomial_trajectory_behavior_plugin_base::GeneratePolynomialTrajectoryBase``.
The base class provides the following virtual methods:

.. list-table:: List of Virtual Methods
   :header-rows: 1

   * - Virtual method
     - Method description
     - Requires override?
   * - ownInitialize()
     - Plugin-specific init hook. Declare and read parameters with the
       protected ``getParameter<T>()`` helper, build internal state.
       Called from ``initialize()`` after the host node and the plugin
       namespace have been cached.
     - No (default is empty)
   * - generateTrajectory(waypoints, max_speed, t_trajectory_now)
     - Generate a fresh trajectory through the mission waypoints. Read
       ``vehicle_pose_`` / ``vehicle_twist_`` from the protected base
       members and inject them into your backend (``buildCurrentWaypoint()``
       is provided as a helper). Anchor your private offset so the host's
       ``t_trajectory_now`` maps to the start of the new backend trajectory.
     - Yes
   * - updateWaypoints(waypoints, max_speed, t_trajectory_now)
     - Update the active trajectory with a new pending waypoint list.
       Default implementation calls ``reset()`` + ``generateTrajectory()``
       (full re-plan, plugin re-anchors against ``t_trajectory_now``).
       Override only if your backend supports smooth online stitching
       and you want to preserve its internal time origin.
     - No (default re-plans)
   * - evaluate(t_trajectory, out, is_horizon_sample)
     - Sample the trajectory at host time ``t_trajectory``. Map it to your
       backend axis with the private offset and clamp to the valid range.
     - Yes
   * - isFinished(t_trajectory)
     - ``true`` when ``t_trajectory`` has reached or exceeded the end of
       the currently held trajectory in the host time axis.
     - Yes
   * - reset()
     - Clear internal state. The wrapper calls it on deactivate and on
       the default ``updateWaypoints()`` re-plan path.
     - Yes
   * - getNextWaypointId()
     - Id of the next pending mission waypoint (empty when none). Used by
       the wrapper to build feedback and to drive the partial-trajectory
       window.
     - Yes
   * - isTrajectoryGenerated()
     - Whether the plugin currently holds a valid trajectory.
     - Yes
   * - consumeRegeneratedFlag()
     - Consume and return whether the underlying backend trajectory has
       been swapped since the previous call. Default ``false`` for
       synchronous plugins. Async plugins override it to expose the
       deferred swap event so the wrapper can refresh
       ``debug/traj_generated`` once the backend has actually swapped its
       internal trajectory.
     - No (default is false)



.. _development_tutorials_trajectory_generation_plugin_steps_initial_state:

2. Initial-State Contract
=========================

The trajectory start state (position **and** velocity) is **not** passed
as an argument to ``generateTrajectory()`` / ``updateWaypoints()``. It
flows through the protected base members:

.. code-block:: cpp

    geometry_msgs::msg::PoseStamped vehicle_pose_;
    geometry_msgs::msg::TwistStamped vehicle_twist_;

The host refreshes them once per incoming state message via
``setVehicleState(pose, twist)`` directly from its odom/twist subscription
callback, so plugin entry points always see the latest state. The base
class exposes them on the read side too via ``getVehiclePose()`` /
``getVehicleTwist()`` — the host uses those for its yaw policies, so the
plugin remains the single source of truth for the live state.

Each plugin decides how to inject the state into its backend:

* Backends that take the start state as a "first waypoint with a fixed
  pose" (gcopter_lib, mav_trajectory_generation_lib) can prepend a
  synthetic entry built from ``vehicle_pose_`` and tag the optional
  velocity boundary condition with ``vehicle_twist_.linear``. The base
  helper ``buildCurrentWaypoint()`` returns a ready-to-use
  ``PoseStampedWithID{id="current", pose=vehicle_pose_}`` for the
  waypoint side; the velocity has to be propagated separately because
  ``PoseStampedWithID`` does not carry twist.
* Backends that derive the boundary state from a previously generated
  trajectory (dynamic_trajectory_generator) can ignore the live values on
  ``updateWaypoints()`` and rely on the lib's stitching logic.



.. _development_tutorials_trajectory_generation_plugin_steps_time_axis:

3. Time-Axis Contract
=====================

The wrapper owns a single logical trajectory time axis,
``trajectory_time_`` (seconds, ≥ 0). It starts at ``0`` on every fresh
``generateTrajectory()`` call (including the regeneration triggered by
external ``on_resume``), advances on every ``on_run`` tick, and is
preserved across ``updateWaypoints()``.

Each plugin keeps a private offset that maps ``t_trajectory`` to its
backend axis:

.. code-block:: cpp

    t_backend = t_trajectory + internal_offset_;

The wrapper never observes that mapping. The contract is:

* ``generateTrajectory(t_trajectory_now)`` re-anchors ``internal_offset_``
  so ``t_trajectory_now`` maps to the start of the new backend
  trajectory.
* ``updateWaypoints(t_trajectory_now)`` lets the plugin choose:

  * **Smooth stitching** — keep ``internal_offset_`` untouched. The
    backend axis continues uninterrupted, and the host's ``t_trajectory``
    keeps mapping to the same backend trajectory without realignment.
  * **Regenerate** (base default) — re-anchor ``internal_offset_``
    against ``t_trajectory_now`` so the freshly generated backend
    trajectory aligns with the host's current time.

* ``evaluate()`` and ``isFinished()`` apply ``internal_offset_`` and
  clamp internally to the valid backend range.



.. _development_tutorials_trajectory_generation_plugin_steps_layout:

4. Folder Layout
================

Create the following layout under
``as2_behaviors_trajectory_generation/generate_polynomial_trajectory_behavior/plugins/``:

.. code-block:: text

    plugins/<my_plugin>/
    ├── CMakeLists.txt
    ├── include/<my_plugin>/<my_plugin>.hpp
    ├── src/<my_plugin>.cpp
    ├── config/<my_plugin>.yaml
    ├── tests/
    │   ├── CMakeLists.txt
    │   └── <my_plugin>_gtest.cpp
    └── thirdparties/        # optional, gitignored, vendored deps

The folder name is the canonical plugin name and is reused for the
library target, the namespace, the YAML filename and the launch
``plugin_name`` choice. Use ``snake_case`` consistently.



.. _development_tutorials_trajectory_generation_plugin_steps_implement:

5. Implementing the Plugin
==========================

Header skeleton (``include/<my_plugin>/<my_plugin>.hpp``):

.. code-block:: cpp

    #pragma once

    #include "generate_polynomial_trajectory_behavior/generate_polynomial_trajectory_base.hpp"

    namespace my_plugin {

    class Plugin
    : public generate_polynomial_trajectory_behavior_plugin_base::
            GeneratePolynomialTrajectoryBase {
    public:
      Plugin() = default;
      ~Plugin() override = default;

      bool generateTrajectory(
          const std::vector<as2_msgs::msg::PoseStampedWithID> & waypoints,
          double max_speed,
          double t_trajectory_now) override;

      bool evaluate(
          double t_trajectory,
          dynamic_traj_generator::References & out,
          bool is_horizon_sample) override;

      bool isFinished(double t_trajectory) override;
      bool isTrajectoryGenerated() override;
      void reset() override;
      std::string getNextWaypointId() override;

    protected:
      void ownInitialize() override;

    private:
      double internal_offset_{0.0};
      // backend handles, cached parameters, etc.
    };

    }  // namespace my_plugin

Source skeleton (``src/<my_plugin>.cpp``):

.. code-block:: cpp

    #include "<my_plugin>/<my_plugin>.hpp"
    #include <pluginlib/class_list_macros.hpp>

    namespace my_plugin {

    void Plugin::ownInitialize() {
      // Read backend parameters using the auto-prefixed helper:
      //   getParameter<double>("limits.max_speed", max_speed_, /*use_default=*/true);
      //
      // Build any backend objects you need.
    }

    bool Plugin::generateTrajectory(
        const std::vector<as2_msgs::msg::PoseStampedWithID> & waypoints,
        double max_speed,
        double t_trajectory_now) {
      // 1. Inject vehicle_pose_ / vehicle_twist_ into the backend.
      // 2. Solve the trajectory.
      // 3. Anchor internal_offset_ so that t_trajectory_now maps to the
      //    start of the new backend trajectory.
      return true;
    }

    bool Plugin::evaluate(
        double t_trajectory,
        dynamic_traj_generator::References & out,
        bool is_horizon_sample) {
      const double t_backend = t_trajectory + internal_offset_;
      // Sample your backend at t_backend, fill `out`. Clamp internally.
      return true;
    }

    bool Plugin::isFinished(double t_trajectory) {
      const double t_backend = t_trajectory + internal_offset_;
      return t_backend >= backendDuration();
    }

    bool Plugin::isTrajectoryGenerated() {
      // True iff a backend trajectory is currently held.
      return false;
    }

    void Plugin::reset() {
      // Clear backend state and reset internal_offset_.
      internal_offset_ = 0.0;
    }

    std::string Plugin::getNextWaypointId() {
      // Return the id of the next pending mission waypoint, or "" if none.
      return {};
    }

    }  // namespace my_plugin

    PLUGINLIB_EXPORT_CLASS(
        my_plugin::Plugin,
        generate_polynomial_trajectory_behavior_plugin_base::
            GeneratePolynomialTrajectoryBase)



.. _development_tutorials_trajectory_generation_plugin_steps_parameters:

6. Parameter Helper
===================

The base class exposes a templated helper:

.. code-block:: cpp

    template <typename T>
    bool getParameter(const std::string & name, T & value, bool use_default);

It **automatically prefixes** the parameter name with the plugin
namespace (resolved by ``qualifyParameterName()``). Plugins only deal
with short keys — there is no risk of collision with the wrapper
parameters or with another plugin loaded in the same node, because the
plugin name is enforced as a prefix.

Supported scalar/array types are dispatched via ``if constexpr``:
``double``, ``int``, ``bool``, ``std::string``, ``std::vector<double>``.
Other types fall back to the generic accessor with a warning.



.. _development_tutorials_trajectory_generation_plugin_steps_register:

7. Registering the Plugin
=========================

Two files have to be touched in the ``generate_polynomial_trajectory_behavior``
directory:

1. **CMake** — append the plugin folder to the ``PLUGIN_LIST`` in
   ``generate_polynomial_trajectory_behavior/CMakeLists.txt``:

   .. code-block:: cmake

       set(PLUGIN_LIST
         dynamic_mav_trajectory_generator
         mav_trajectory_generator
         jerk_limited_trajectory_generator
         gcopter_trajectory_generator
         my_plugin
       )

   The loop in the parent CMake will pick the new folder up
   automatically (``add_subdirectory``, install of the YAML, etc.).

2. **pluginlib registry** — add an entry to
   ``generate_polynomial_trajectory_behavior/plugins.xml``:

   .. code-block:: xml

       <library path="my_plugin">
         <class type="my_plugin::Plugin"
                base_class_type="generate_polynomial_trajectory_behavior_plugin_base::GeneratePolynomialTrajectoryBase">
           <description>Short description of the new generator.</description>
         </class>
       </library>

The plugin's own ``CMakeLists.txt`` is responsible for exposing exactly
**one** SHARED library named after the plugin folder, exporting the
symbol ``<plugin_namespace>::Plugin`` — which ``plugins.xml`` references
through ``pluginlib::ClassLoader``.



.. _development_tutorials_trajectory_generation_plugin_steps_yaml:

8. Configuration YAML
=====================

Provide ``config/<my_plugin>.yaml`` with the backend defaults under the
plugin namespace:

.. code-block:: yaml

    /**:
      ros__parameters:
        my_plugin:
          limits:
            max_speed: 1.0
            max_acceleration: 5.0
          # ... your backend keys here ...

The launch files locate the plugin YAML automatically as
``plugins/<my_plugin>/config/<my_plugin>.yaml`` once the plugin folder is
discovered.



.. _development_tutorials_trajectory_generation_plugin_steps_test:

9. Testing
==========

Add a minimal gtest in ``tests/<my_plugin>_gtest.cpp`` that exercises
the contract end-to-end:

1. ``setVehicleState(pose, twist)`` to seed the initial state.
2. ``generateTrajectory(waypoints, max_speed, /*t_trajectory_now=*/0.0)``.
3. ``evaluate(t_trajectory, out, false)`` at several points along the
   trajectory.
4. ``isFinished(0.0)`` is ``false`` right after generation; ``isFinished(t)``
   becomes ``true`` after walking past the end.
5. ``reset()`` clears the state.

For plugins that use the regenerate path, also verify that a second
``generateTrajectory(t_trajectory_now=t)`` (or the default
``updateWaypoints(t)``) re-anchors the offset so subsequent
``evaluate()`` calls stay valid for the host's progressing
``trajectory_time_``.

Build and run with:

.. code-block:: bash

   colcon build --packages-select as2_behaviors_trajectory_generation
   colcon test  --packages-select as2_behaviors_trajectory_generation



.. _development_tutorials_trajectory_generation_plugin_steps_launch:

10. Launching with the New Plugin
=================================

Once the workspace is rebuilt and sourced, the new plugin name is
auto-registered as a valid ``plugin_name`` choice on the launch:

.. code-block:: bash

   ros2 launch as2_behaviors_trajectory_generation generate_polynomial_trajectory_behavior_launch.py \
       namespace:=<drone_namespace> \
       plugin_name:=my_plugin

The wrapper, the action server, the partial-trajectory window and the
debug topics work without any change.

See the :ref:`Generate Polynomial Trajectory Behavior <behaviors_trajectory_generation_page>`
page for details on the action interface and the wrapper-level
parameters that your plugin will inherit for free.
