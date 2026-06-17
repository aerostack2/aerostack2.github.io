.. _development_tutorials_follow_reference_plugin:

=======================================
Writing a New Follow Reference Plugin
=======================================

.. contents:: Table of Contents
   :depth: 2
   :local:

Overview
--------

The :ref:`behaviors_follow_reference_page` ships as a plugin-based behavior
under the ``as2_behaviors_motion`` package. The wrapper
``FollowReferenceBehavior`` owns the action lifecycle, validates the goal,
resolves the target frame via TF and feeds the active plugin with the
validated state and platform info; the plugin only has to implement the
**tracking strategy** (position setpoints, trajectory delegation, …).

This tutorial walks through the contract exposed by
``follow_reference_base::FollowReferenceBase`` and how to build a plugin
against it. The reference plugins live in
`as2_behaviors_motion/follow_reference_behavior/plugins <https://github.com/aerostack2/aerostack2/tree/main/as2_behaviors/as2_behaviors_motion/follow_reference_behavior/plugins>`_:

* `follow_reference_plugin_position <https://github.com/aerostack2/aerostack2/blob/main/as2_behaviors/as2_behaviors_motion/follow_reference_behavior/plugins/follow_reference_plugin_position.cpp>`_
  — direct position-setpoint plugin used as the running example below.
* `follow_reference_plugin_trajectory <https://github.com/aerostack2/aerostack2/blob/main/as2_behaviors/as2_behaviors_motion/follow_reference_behavior/plugins/follow_reference_plugin_trajectory.cpp>`_
  — delegates tracking to ``TrajectoryGeneratorBehavior`` in
  ``follow_reference_mode``.

It is worth reading :ref:`behaviors_follow_reference_page` first for the
external semantics (action interface, yaw modes, configuration) before
diving into the plugin side.

Requirements
------------

* ROS 2 Humble
* Aerostack2 (with ``as2_behaviors_motion`` built and sourced)

Architecture
------------

``FollowReferenceBase`` owns the wiring with the wrapper and exposes a small
set of hooks the plugin implements:

* **Lifecycle** — ``FollowReferenceBehavior`` resolves
  ``plugin_name + "::Plugin"`` via ``pluginlib``, then calls
  ``initialize(node, tf_handler)`` on the loaded plugin. Inside ``initialize``
  the base reads the per-axis defaults ``follow_reference_max_speed_x/y/z``
  from the node, creates the shared ``HoverMotion`` handler and invokes the
  plugin's ``ownInit()``.
* **State and platform info** — the wrapper subscribes to
  ``self_localization/twist`` and ``platform/info`` and forwards each message
  to ``state_callback`` and ``platform_info_callback`` on the base. The base
  caches the pose in ``actual_pose_`` (already in ``earth``), updates the
  feedback ``actual_distance_to_goal`` from the target converted to earth,
  and flips ``localization_flag_`` once the first message arrives.
* **Goal handling** — the wrapper calls ``on_activate(goal)`` /
  ``on_modify(goal)``. The base validates that the platform is ``FLYING``
  and that localization has been received (``processGoal``), and only then
  hands the goal to the plugin via ``own_activate`` / ``own_modify``. The
  base also routes ``on_deactivate``, ``on_pause``, ``on_resume``,
  ``on_execution_end`` and ``on_run`` to their ``own_*`` counterparts.

The full header is in
`follow_reference_base.hpp <https://github.com/aerostack2/aerostack2/blob/main/as2_behaviors/as2_behaviors_motion/follow_reference_behavior/include/follow_reference_behavior/follow_reference_base.hpp>`_.

Plugin Contract
^^^^^^^^^^^^^^^

.. list-table:: Methods the plugin overrides
   :widths: 30 50 20
   :header-rows: 1

   * - Method
     - Purpose
     - Required
   * - ``ownInit()``
     - Allocate motion reference handlers, action / topic clients, plugin
       publishers. Read any plugin-specific parameters from the node.
     - No
   * - ``own_activate(goal)``
     - Accept the validated goal and prepare to track the reference. Return
       ``true`` if the plugin can proceed.
     - **Yes**
   * - ``own_modify(goal)``
     - Apply a goal modification while running. Default returns ``false``
       (unsupported).
     - No
   * - ``own_deactivate(message)``
     - Stop tracking, leave the drone in a safe state. The default base
       behavior is unsupported; the position plugin sends a hover.
     - **Yes**
   * - ``own_pause(message)`` / ``own_resume(message)``
     - Pause and resume hooks. Defaults return ``false``; implement only
       when the plugin can be paused safely.
     - No
   * - ``own_execution_end(state)``
     - Finalize when the action ends (success / failure / cancel). Release
       resources, send hover if needed.
     - **Yes**
   * - ``own_run()``
     - Per-tick work: emit the next reference and report a status. Returns
       an ``as2_behavior::ExecutionStatus`` (typically ``RUNNING`` until
       cancelled).
     - **Yes**

.. list-table:: Base members available to the plugin
   :widths: 35 65
   :header-rows: 1

   * - Member
     - Description
   * - ``node_ptr_``
     - Non-owning ``as2::Node *`` for logging, parameters, publishers.
   * - ``tf_handler_``
     - ``std::shared_ptr<as2::tf::TfHandler>`` for frame conversions.
   * - ``goal_``
     - Last goal accepted by ``own_activate`` / ``own_modify``.
   * - ``feedback_`` / ``result_``
     - Action feedback / result instances the wrapper forwards to the
       client at the end of each tick.
   * - ``actual_pose_``
     - Last validated pose state in ``earth``.
   * - ``platform_state_``
     - Last ``as2_msgs::msg::PlatformStatus`` value seen on
       ``platform/info``.
   * - ``localization_flag_``
     - ``true`` once at least one state message has been received.
   * - ``params_``
     - Struct with the per-axis defaults
       (``follow_reference_max_speed_x/y/z``).
   * - ``sendHover()``
     - Helper that publishes a hover through the shared ``HoverMotion``
       handler.

Tutorial Steps
--------------

The snippets below are simplified excerpts from
``follow_reference_plugin_position``. Refer to the full source for the
complete implementation.

1. Plugin skeleton
^^^^^^^^^^^^^^^^^^

Create a class that inherits from
``follow_reference_base::FollowReferenceBase`` and declares the overrides.

.. code-block:: cpp

   #include "follow_reference_behavior/follow_reference_base.hpp"
   #include "as2_motion_reference_handlers/position_motion.hpp"

   namespace follow_reference_plugin_position
   {

   class Plugin : public follow_reference_base::FollowReferenceBase
   {
   public:
     void ownInit() override;

     bool own_activate(as2_msgs::action::FollowReference::Goal & goal) override;
     bool own_modify(as2_msgs::action::FollowReference::Goal & goal) override;
     bool own_deactivate(const std::shared_ptr<std::string> & message) override;
     bool own_pause(const std::shared_ptr<std::string> & message) override;
     bool own_resume(const std::shared_ptr<std::string> & message) override;
     void own_execution_end(const as2_behavior::ExecutionStatus & state) override;
     as2_behavior::ExecutionStatus own_run() override;

   private:
     std::shared_ptr<as2::motionReferenceHandlers::PositionMotion> position_motion_handler_;
   };

   }  // namespace follow_reference_plugin_position

2. Initialization
^^^^^^^^^^^^^^^^^

``ownInit`` runs after the base has injected the node, the TF handler and
the shared ``HoverMotion`` handler. Allocate plugin-specific motion
handlers and read plugin parameters here. Use
``follow_reference_base::FollowReferenceBase::params_`` to access the
per-axis defaults that the base has already loaded.

.. code-block:: cpp

   void Plugin::ownInit()
   {
     position_motion_handler_ =
       std::make_shared<as2::motionReferenceHandlers::PositionMotion>(node_ptr_);
   }

For plugins that need extra parameters, declare and read them on the node
inside ``ownInit`` (use a ``<plugin_name>.<key>`` namespace to avoid
collisions with the wrapper). See ``follow_reference_plugin_trajectory`` for
an example reading ``follow_reference_plugin_trajectory.modify_threshold``
and ``modify_frequency``.

3. Goal activation
^^^^^^^^^^^^^^^^^^

``own_activate`` only sees goals that the base has already validated
(platform ``FLYING`` and localization received). The wrapper has also
replaced any zero ``max_speed_*`` field with the matching
``follow_reference_max_speed_*`` default. Validate any plugin-specific
constraint (e.g. yaw mode supported), prepare internal state and return
``true`` to accept.

.. code-block:: cpp

   bool Plugin::own_activate(as2_msgs::action::FollowReference::Goal & goal)
   {
     if (!computeYaw(goal.yaw.mode,
                     goal.target_pose.point,
                     actual_pose_.pose.position,
                     goal.yaw.angle))
     {
       return false;
     }
     RCLCPP_INFO(
       node_ptr_->get_logger(),
       "FollowReference target: %f, %f, %f",
       goal.target_pose.point.x,
       goal.target_pose.point.y,
       goal.target_pose.point.z);
     return true;
   }

The goal stays accessible to the rest of the lifecycle through ``goal_``
(filled by the base only after ``own_activate`` returns ``true``).

4. Goal modification (optional)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Implement ``own_modify`` only when the plugin can update the active goal
without re-activating. The default returns ``false``, so the wrapper
rejects modifications. ``follow_reference_plugin_trajectory`` publishes to
``motion_reference/modify_waypoint`` and accepts.

.. code-block:: cpp

   bool Plugin::own_modify(as2_msgs::action::FollowReference::Goal & goal)
   {
     if (!computeYaw(goal.yaw.mode, goal.target_pose.point,
                     actual_pose_.pose.position, goal.yaw.angle)) {
       return false;
     }
     return true;
   }

5. Deactivation, pause and resume
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``own_deactivate`` runs on a clean cancel. Leave the drone in a safe state
— the position plugin sends a hover and clears the cached target frame so
the next ``own_run`` ticks become no-ops.

.. code-block:: cpp

   bool Plugin::own_deactivate(const std::shared_ptr<std::string> & /*message*/)
   {
     goal_.target_pose.header.frame_id = "";
     sendHover();
     return true;
   }

Implement ``own_pause`` / ``own_resume`` only when the plugin can be safely
suspended:

.. code-block:: cpp

   bool Plugin::own_pause(const std::shared_ptr<std::string> & /*message*/)
   {
     sendHover();
     return true;
   }

   bool Plugin::own_resume(const std::shared_ptr<std::string> & /*message*/)
   {
     return true;
   }

6. Execution end
^^^^^^^^^^^^^^^^

``own_execution_end`` is invoked whenever the action terminates (success,
failure or cancel). The base has already cleared ``localization_flag_``;
release any plugin-specific resource and leave the drone safe.

.. code-block:: cpp

   void Plugin::own_execution_end(const as2_behavior::ExecutionStatus & /*state*/)
   {
     sendHover();
   }

7. Per-tick run
^^^^^^^^^^^^^^^

``own_run`` is the per-tick entry point. Read the live target through
``goal_``, convert it to whatever frame the underlying handler expects
(``earth`` is the canonical one), and publish the reference. Return
``RUNNING`` while tracking — Follow Reference is open-ended, the wrapper
expects the action to be cancelled by the client.

.. code-block:: cpp

   as2_behavior::ExecutionStatus Plugin::own_run()
   {
     // Re-resolve the live target to "earth" so moving target frames work
     // transparently (the position handler publishes in a static frame).
     geometry_msgs::msg::PointStamped target_stamped = goal_.target_pose;
     target_stamped.header.stamp = node_ptr_->now();
     geometry_msgs::msg::PointStamped target_in_earth;
     try {
       target_in_earth = tf_handler_->convert(target_stamped, "earth");
     } catch (const tf2::TransformException & ex) {
       RCLCPP_WARN_THROTTLE(
         node_ptr_->get_logger(), *node_ptr_->get_clock(), 2000,
         "TF lookup '%s' -> 'earth' failed: %s",
         goal_.target_pose.header.frame_id.c_str(), ex.what());
       return as2_behavior::ExecutionStatus::RUNNING;
     }

     if (!position_motion_handler_->sendPositionCommandWithYawAngle(
         "earth",
         static_cast<float>(target_in_earth.point.x),
         static_cast<float>(target_in_earth.point.y),
         static_cast<float>(target_in_earth.point.z),
         goal_.yaw.angle,
         "earth",
         goal_.max_speed_x, goal_.max_speed_y, goal_.max_speed_z))
     {
       result_.follow_reference_success = false;
       return as2_behavior::ExecutionStatus::FAILURE;
     }
     result_.follow_reference_success = true;
     return as2_behavior::ExecutionStatus::RUNNING;
   }

8. Exporting the plugin
^^^^^^^^^^^^^^^^^^^^^^^

Register the class with ``pluginlib`` at the bottom of the source file:

.. code-block:: cpp

   #include <pluginlib/class_list_macros.hpp>
   PLUGINLIB_EXPORT_CLASS(
     follow_reference_plugin_position::Plugin,
     follow_reference_base::FollowReferenceBase)

The plugin manifest lives at the **root of the ``as2_behaviors_motion``
package** (``plugins.xml``) and is shared by every motion behavior plugin.
Append a ``<class>`` block for the new plugin to the existing
``<library path="as2_behaviors_motion">``:

.. code-block:: xml

   <class type="follow_reference_plugin_position::Plugin"
          base_class_type="follow_reference_base::FollowReferenceBase">
     <description>Follow Reference done with position commands.</description>
   </class>

The package ``CMakeLists.txt`` already exports the manifest with
``pluginlib_export_plugin_description_file(as2_behaviors_motion plugins.xml)``;
nothing extra is needed there for a new plugin under the same library.

9. Configuration files
^^^^^^^^^^^^^^^^^^^^^^

The wrapper YAML lives in
``as2_behaviors_motion/follow_reference_behavior/config/config_default.yaml``.
Plugin-specific parameters use the ``<plugin_name>.*`` namespace inside the
same file. Plugin authors only add the keys their plugin needs; the wrapper
keys (max speed defaults, ``tf_timeout_threshold``) stay shared.

.. code-block:: yaml

   /**:
     ros__parameters:
       follow_reference_max_speed_x: 10.0
       follow_reference_max_speed_y: 10.0
       follow_reference_max_speed_z: 10.0
       tf_timeout_threshold: 0.05

       # Plugin-specific defaults. Only loaded when the matching plugin is
       # selected via plugin_name.
       follow_reference_plugin_trajectory:
         modify_threshold: 0.0
         modify_frequency: 0.0

10. Launching
^^^^^^^^^^^^^

The behavior exposes a standard launch with a ``plugin_name`` argument
validated against the registered plugins. If ``plugin_name`` is empty the
launch reads it from the config file.

.. code-block:: bash

   ros2 launch as2_behaviors_motion follow_reference_behavior_launch.py \
       namespace:=<drone_namespace> \
       plugin_name:=follow_reference_plugin_position

Useful arguments:

* ``namespace`` — drone namespace (defaults to
  ``AEROSTACK2_SIMULATION_DRONE_ID``).
* ``plugin_name`` — one of
  ``follow_reference_plugin_position``,
  ``follow_reference_plugin_trajectory``. The list is enforced by
  ``get_available_plugins('as2_behaviors_motion', 'follow_reference')``.
* ``behavior_config_file`` — path to the behavior YAML (defaults to
  ``config_default.yaml``).
* ``log_level`` / ``use_sim_time`` — node log level and clock source.

Where to look next
------------------

* `follow_reference_behavior plugins/ <https://github.com/aerostack2/aerostack2/tree/main/as2_behaviors/as2_behaviors_motion/follow_reference_behavior/plugins>`_
  — full source of ``follow_reference_plugin_position`` and
  ``follow_reference_plugin_trajectory``.
* `follow_reference_base.hpp <https://github.com/aerostack2/aerostack2/blob/main/as2_behaviors/as2_behaviors_motion/follow_reference_behavior/include/follow_reference_behavior/follow_reference_base.hpp>`_
  — authoritative documentation of every base method and helper.
* :ref:`behaviors_follow_reference_page` — external view of the behavior
  (action interface, yaw policy, configuration).
