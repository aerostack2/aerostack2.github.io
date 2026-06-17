.. _development_tutorials_motion_controller:

======================================
Writing a New Motion Controller Plugin
======================================

.. contents:: Table of Contents
   :depth: 2
   :local:

Overview
--------

Aerostack2 loads motion controllers as ``pluginlib`` plugins under the
``as2_motion_controller`` package. The controller node (``ControllerManager``
and ``ControllerHandler``) owns the ROS lifecycle, the TF lookups and the
publishers; the plugin only has to implement the **control law** —
state caching, reference handling, mode validation and the per-tick
``computeOutput``.

This tutorial walks through the contract exposed by
``as2_motion_controller_plugin_base::ControllerBase`` and how to build a
plugin against it. The reference implementations live in
`as2_motion_controller/plugins <https://github.com/aerostack2/aerostack2/tree/main/as2_motion_controller/plugins>`_:

* `pid_speed_controller <https://github.com/aerostack2/aerostack2/tree/main/as2_motion_controller/plugins/pid_speed_controller>`_
  — PID-based velocity controller used as the running example below.
* `differential_flatness_controller <https://github.com/aerostack2/aerostack2/tree/main/as2_motion_controller/plugins/differential_flatness_controller>`_
  — flatness-based controller for ACRO / ATTITUDE outputs.

Before reading this tutorial it is worth remembering the
:ref:`Aerostack2 architecture <as2_concepts_architecture>` and the
:ref:`motion controller <as2_concepts_motion_controller>` role.

Requirements
------------

* ROS 2 Humble
* Aerostack2 (with ``as2_motion_controller`` built and sourced)

Architecture
------------

``ControllerBase`` owns the heavy lifting and exposes a small set of hooks
the plugin must implement:

* **Lifecycle** — ``ControllerManager`` calls ``setTfHandler()``,
  ``setBaseLinkFrameId()``, ``setPluginParamNamespace()`` and finally
  ``initialize(node)``. Inside ``initialize()`` the base declares the
  ``desired_pose_frame`` and ``desired_twist_frame`` parameters, invokes the
  plugin's ``ownInitialize()`` and seeds the pending-essential-parameter
  set from ``getEssentialParameters()``.
* **State and references** — incoming messages are converted by
  ``ControllerHandler`` to the frames returned by ``getDesiredPoseFrameId()``
  / ``getDesiredTwistFrameId()`` before reaching the base. ``updateState``
  validates the frame, caches the state, consumes any pending hover latch
  and forwards to ``onUpdateState``. The four ``updateReference`` overloads
  forward to the matching ``onUpdateReference`` overload.
* **Parameters** — every parameter under ``<plugin_namespace>.`` is routed
  to ``updateParameter()``. The base tracks the names returned by
  ``getEssentialParameters()`` and fires ``onAllParametersRead()`` exactly
  once when every essential has been applied; ``setMode`` can gate on
  ``essentialParamsReady()``.
* **Hover** — ``ControllerHandler`` calls ``requestHoverLatch()`` after a
  successful ``setMode(HOVER)`` and the base materialises a hover reference
  inside the next ``updateState`` tick via ``latchHoverReference()``. The
  default latch synthesises a one-point trajectory at the cached pose; the
  plugin can override it.

The full header is in
`controller_base.hpp <https://github.com/aerostack2/aerostack2/blob/main/as2_motion_controller/include/as2_motion_controller/controller_base.hpp>`_.

Plugin Contract
^^^^^^^^^^^^^^^

.. list-table:: Methods the plugin overrides
   :widths: 30 50 20
   :header-rows: 1

   * - Method
     - Purpose
     - Required
   * - ``ownInitialize()``
     - Allocate handlers, create plugin-specific publishers, finish setup
       after the base has injected node / TF / param namespace.
     - No
   * - ``onUpdateState(pose, twist)``
     - Cache or transform the validated state used by the control law.
     - **Yes**
   * - ``onUpdateReference(PoseStamped)``,
       ``onUpdateReference(TwistStamped)``,
       ``onUpdateReference(TrajectorySetpoints)``,
       ``onUpdateReference(Thrust)``
     - Per-reference-type hooks. Implement only the ones the plugin
       consumes.
     - No (defaults are no-ops)
   * - ``setMode(in, out)``
     - Validate the in/out control-mode pair, configure the output frame,
       reset integrators when the mode actually changes.
     - **Yes**
   * - ``getEssentialParameters()``
     - Fully-qualified names of the parameters that must be present before
       the plugin accepts ``setMode``.
     - **Yes**
   * - ``updateParameter(parameter)``
     - Apply one parameter under the plugin namespace. Called both for the
       initial batch and for runtime changes.
     - **Yes**
   * - ``onAllParametersRead()``
     - One-shot hook fired the first time every essential parameter has
       been delivered. Good place for first-time solver configuration.
     - No
   * - ``computeOutput(dt, pose, twist, thrust)``
     - Per-tick controller evaluation.
     - **Yes**
   * - ``reset()``
     - Clear cached state / commands / integrators. Must call
       ``ControllerBase::reset()`` to clear base-owned flags.
     - No
   * - ``latchHoverReference(pose, twist)``
     - Produce the hover reference when ``HOVER`` is requested. Override if
       the default trajectory latch is not compatible with the plugin.
     - No

.. list-table:: Base helpers available to the plugin
   :widths: 35 65
   :header-rows: 1

   * - Helper
     - Description
   * - ``getNodePtr()``
     - Non-owning ``as2::Node *`` for logging, parameters, publishers.
   * - ``getTfHandler()``
     - Shared ``as2::tf::TfHandler`` for additional frame conversions.
   * - ``getBaseLinkFrameId()``
     - Namespaced ``base_link`` frame.
   * - ``getDesiredPoseFrameId()`` / ``getDesiredTwistFrameId()``
     - Frames the plugin currently expects for state and references. Update
       them from ``setMode()`` with ``setDesiredPoseFrameId`` /
       ``setDesiredTwistFrameId`` when the mode demands it.
   * - ``getPluginParamNamespace()`` / ``param(tail)``
     - Plugin namespace (e.g. ``"pid_speed_controller"``) and helper that
       returns ``"<namespace>.<tail>"`` ready for ``declare_parameter`` /
       ``get_parameter``.
   * - ``getStatePose()`` / ``getStateTwist()``
     - Last validated state cached by the base.
   * - ``isStateReceived()`` / ``isReferenceReceived()`` /
       ``isHoverPending()`` / ``essentialParamsReady()``
     - Status flags maintained by the base.

Tutorial Steps
--------------

The snippets below are simplified excerpts from
``pid_speed_controller``. Refer to the full source for the complete
implementation.

1. Plugin skeleton
^^^^^^^^^^^^^^^^^^

Create a class that inherits from
``as2_motion_controller_plugin_base::ControllerBase`` and declares the
required overrides.

.. code-block:: cpp

   #include "as2_motion_controller/controller_base.hpp"

   namespace pid_speed_controller
   {

   class Plugin : public as2_motion_controller_plugin_base::ControllerBase
   {
   public:
     Plugin() = default;
     ~Plugin() override = default;

     void ownInitialize() override;

     std::vector<std::string> getEssentialParameters() const override;
     void updateParameter(const rclcpp::Parameter & parameter) override;
     void onAllParametersRead() override;  // optional

     void onUpdateState(
       const geometry_msgs::msg::PoseStamped & pose_msg,
       const geometry_msgs::msg::TwistStamped & twist_msg) override;
     void onUpdateReference(const geometry_msgs::msg::PoseStamped & ref) override;
     void onUpdateReference(const geometry_msgs::msg::TwistStamped & ref) override;
     void onUpdateReference(const as2_msgs::msg::TrajectorySetpoints & ref) override;

     bool setMode(
       const as2_msgs::msg::ControlMode & mode_in,
       const as2_msgs::msg::ControlMode & mode_out) override;

     bool computeOutput(
       double dt,
       geometry_msgs::msg::PoseStamped & pose,
       geometry_msgs::msg::TwistStamped & twist,
       as2_msgs::msg::Thrust & thrust) override;

     void reset() override;
   };

   }  // namespace pid_speed_controller

2. Initialization
^^^^^^^^^^^^^^^^^

``ownInitialize`` runs after the base has injected the node, TF handler and
plugin namespace and after ``desired_pose_frame`` / ``desired_twist_frame``
have been read. Allocate handlers, create optional debug publishers and pull
the initial frame ids from the helpers.

.. code-block:: cpp

   void Plugin::ownInitialize()
   {
     speed_limits_ = Eigen::Vector3d::Zero();

     // Default the output twist frame to the configured pose frame until
     // setMode() picks a body-frame mode.
     output_twist_frame_id_ = getDesiredPoseFrameId();

     // Optional plugin-specific debug publisher under the plugin namespace.
     const std::string desired_velocity_topic =
       declareOptionalTopic(getNodePtr(), param("debug.desired_velocity_topic"));
     if (!desired_velocity_topic.empty()) {
       debug_desired_velocity_pub_ =
         getNodePtr()->create_publisher<geometry_msgs::msg::TwistStamped>(
         desired_velocity_topic, rclcpp::SensorDataQoS());
     }

     reset();
   }

3. Essential parameters and parameter callbacks
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``getEssentialParameters`` returns the fully-qualified names that **must
arrive** before the plugin accepts ``setMode``. Use ``param("…")`` to keep
the names anchored to the plugin namespace.

.. code-block:: cpp

   std::vector<std::string> Plugin::getEssentialParameters() const
   {
     std::vector<std::string> out;
     for (const auto & tail : plugin_parameters_tail_)        {out.push_back(param(tail));}
     for (const auto & tail : position_control_parameters_tail_) {out.push_back(param(tail));}
     for (const auto & tail : yaw_control_parameters_tail_)   {out.push_back(param(tail));}
     return out;
   }

``updateParameter`` is called for every parameter under ``<plugin_ns>.``
both at startup and at runtime. Dispatch the value to the matching gain
group or flag:

.. code-block:: cpp

   void Plugin::updateParameter(const rclcpp::Parameter & parameter)
   {
     const std::string & full_name = parameter.get_name();
     // … strip the plugin namespace and route to the right handler.
     // Update the params_read_ flags for optional groups (e.g. trajectory_control)
     // so setMode can refuse modes whose gains have not arrived yet.
   }

If first-time configuration depends on the full parameter set, override
``onAllParametersRead`` — it fires exactly once, when the last essential
parameter is delivered, with ``essentialParamsReady() == true``.

4. State and reference hooks
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``onUpdateState`` only sees messages whose frames match the desired ones
(the base already filtered them and consumed pending hover latches).
Implement only the ``onUpdateReference`` overloads the plugin actually
consumes — the rest default to no-op.

.. code-block:: cpp

   void Plugin::onUpdateState(
     const geometry_msgs::msg::PoseStamped & pose_msg,
     const geometry_msgs::msg::TwistStamped & twist_msg)
   {
     uav_state_.position = {
       pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z};
     uav_state_.velocity = {
       twist_msg.twist.linear.x, twist_msg.twist.linear.y, twist_msg.twist.linear.z};
     uav_state_.yaw.x() = as2::frame::getYawFromQuaternion(pose_msg.pose.orientation);
   }

5. Mode handling
^^^^^^^^^^^^^^^^

``setMode`` validates the requested in/out pair and, when the mode demands
it, calls ``setDesiredPoseFrameId`` / ``setDesiredTwistFrameId`` so the
``ControllerHandler`` converts subsequent state and reference messages to
the right frames. Gate on ``essentialParamsReady()`` before validating
mode-specific parameter groups.

.. code-block:: cpp

   bool Plugin::setMode(
     const as2_msgs::msg::ControlMode & in_mode,
     const as2_msgs::msg::ControlMode & out_mode)
   {
     if (!essentialParamsReady()) {
       RCLCPP_WARN(
         getNodePtr()->get_logger(),
         "Essential parameters not read yet, can not set mode");
       return false;
     }
     // … check mode-specific gain groups, configure output_twist_frame_id_,
     // reset integrators if the mode changed.
     control_mode_in_  = in_mode;
     control_mode_out_ = out_mode;
     return true;
   }

6. Compute output
^^^^^^^^^^^^^^^^^

The wrapper calls ``computeOutput`` at ``cmd_freq``. Pack the command in
the frame announced through the previous ``setDesiredPoseFrameId`` /
``setDesiredTwistFrameId`` calls.

.. code-block:: cpp

   bool Plugin::computeOutput(
     double dt,
     geometry_msgs::msg::PoseStamped & pose,
     geometry_msgs::msg::TwistStamped & twist,
     as2_msgs::msg::Thrust & thrust)
   {
     if (!isStateReceived() || !isReferenceReceived()) {return false;}
     // run the active PID handler on (uav_state_, control_ref_) …
     twist.header.frame_id = output_twist_frame_id_;
     twist.header.stamp    = getNodePtr()->now();
     // fill twist.twist from the PID output
     return true;
   }

7. Reset and hover
^^^^^^^^^^^^^^^^^^

When overriding ``reset``, call the base implementation so the
``state_received_`` / ``reference_received_`` / ``hover_pending_`` flags are
cleared too. The ``essentialParamsReady()`` latch is intentionally
monotonic and is **not** cleared by ``reset()``.

.. code-block:: cpp

   void Plugin::reset()
   {
     ControllerBase::reset();
     resetReferences();
     resetState();
     resetCommands();
     pid_yaw_handler_.reset_controller();
     pid_3D_position_handler_.reset_controller();
     // … other handlers
   }

If the default hover latch (single-point trajectory at the cached pose) does
not fit the plugin (e.g. the plugin only consumes pose / twist references
gated by mode), override ``latchHoverReference``:

.. code-block:: cpp

   void Plugin::latchHoverReference(
     const geometry_msgs::msg::PoseStamped & pose,
     const geometry_msgs::msg::TwistStamped & /*twist*/)
   {
     control_ref_.position = {pose.pose.position.x, pose.pose.position.y, pose.pose.position.z};
     control_ref_.velocity.setZero();
     control_ref_.yaw.x() = as2::frame::getYawFromQuaternion(pose.pose.orientation);
   }

8. Exporting the plugin
^^^^^^^^^^^^^^^^^^^^^^^

Register the class with ``pluginlib`` at the bottom of the source file:

.. code-block:: cpp

   #include <pluginlib/class_list_macros.hpp>
   PLUGINLIB_EXPORT_CLASS(
     pid_speed_controller::Plugin,
     as2_motion_controller_plugin_base::ControllerBase)

The plugin manifest lives at the **root of the ``as2_motion_controller``
package** (``plugins.xml``). Append a ``<library>`` block for the new plugin:

.. code-block:: xml

   <library path="pid_speed_controller">
     <class type="pid_speed_controller::Plugin"
            base_class_type="as2_motion_controller_plugin_base::ControllerBase">
       <description>Controller plugin for PID speed control.</description>
     </class>
   </library>

Then declare the manifest in the package ``CMakeLists.txt`` so it is
installed and discoverable:

.. code-block:: cmake

   pluginlib_export_plugin_description_file(as2_motion_controller plugins.xml)

9. Configuration files
^^^^^^^^^^^^^^^^^^^^^^

Two YAML files cooperate at launch time.

**Wrapper config** — defaults in
``as2_motion_controller/config/motion_controller_default.yaml``. The
relevant keys for plugin authors are:

.. code-block:: yaml

   /**:
     ros__parameters:
       cmd_freq: 100.0
       info_freq: 10.0
       use_bypass: true
       tf_timeout_threshold: 0.05

       # Frames the active plugin expects for state and references. The
       # ControllerHandler converts incoming pose/twist/trajectory messages
       # to these frames before delivering them to the plugin.
       desired_pose_frame: "odom"
       desired_twist_frame: "base_link"

       # Debug topics published by ControllerHandler. Empty = disabled.
       # Plugin-specific debug topics live under <plugin_name>.debug.*
       # in each plugin's controller_default.yaml.
       debug:
         state_pose_topic: "debug/controller/state/pose"
         state_twist_topic: "debug/controller/state/twist"
         reference_pose_topic: "debug/controller/reference/pose"
         reference_twist_topic: "debug/controller/reference/twist"
         reference_trajectory_topic: "debug/controller/reference/trajectory"
         reference_thrust_topic: "debug/controller/reference/thrust"
         compute_output_time_topic: "debug/controller/compute_output_time"

**Plugin config** — defaults in
``plugins/<plugin_name>/config/controller_default.yaml``, namespaced under
the plugin name. Every key the plugin reads via
``param("<sub>.<key>")`` must live under ``<plugin_name>.``. Example:

.. code-block:: yaml

   /**:
     ros__parameters:
       pid_speed_controller:
         proportional_limitation: true
         use_bypass: true
         position_control:
           reset_integral: false
           antiwindup_cte: 0.0
           alpha: 0.0
           kp: {x: 0.0, y: 0.0, z: 0.0}
           kd: {x: 0.0, y: 0.0, z: 0.0}
           ki: {x: 0.0, y: 0.0, z: 0.0}
         yaw_control:
           reset_integral: false
           antiwindup_cte: 0.0
           alpha: 0.0
           kp: 0.0
           kd: 0.0
           ki: 0.0
         # optional groups gated by setMode (trajectory_control, …) …

**Control modes** — declare the input/output control modes the plugin
supports in a separate YAML loaded by the controller launch. The plugin
itself does not parse this file; ``ControllerManager`` uses it to negotiate
the active mode with the platform.

.. code-block:: yaml

   /**:
     ros__parameters:
       input_control_modes:
         - 0b00010000  # HOVER
         - 0b01000000  # SPEED with yaw ANGLE in LOCAL_FLU
         - 0b01000001  # SPEED with yaw ANGLE in GLOBAL_ENU
         - 0b01110001  # TRAJECTORY with yaw ANGLE in GLOBAL_ENU
       output_control_modes:
         - 0b01000100  # SPEED with yaw SPEED in LOCAL_FLU
         - 0b01000101  # SPEED with yaw SPEED in GLOBAL_ENU

10. Launching
^^^^^^^^^^^^^

The controller manager exposes a standard launch file with a ``plugin_name``
argument validated against the registered plugins. If ``plugin_name`` is
empty the launch reads it from the config file.

.. code-block:: bash

   ros2 launch as2_motion_controller controller_launch.py \
       namespace:=<drone_namespace> \
       plugin_name:=pid_speed_controller

Useful arguments:

* ``namespace`` — drone namespace (defaults to
  ``AEROSTACK2_SIMULATION_DRONE_ID``).
* ``plugin_name`` — ``pid_speed_controller`` |
  ``differential_flatness_controller`` | … any registered plugin.
* ``config_file`` — wrapper YAML (defaults to
  ``motion_controller_default.yaml``).
* ``plugin_config_file`` — plugin YAML (defaults to the plugin's
  ``controller_default.yaml``).
* ``log_level`` / ``use_sim_time`` — node log level and clock source.

Where to look next
------------------

* `as2_motion_controller plugins/ <https://github.com/aerostack2/aerostack2/tree/main/as2_motion_controller/plugins>`_
  — full source of ``pid_speed_controller`` and
  ``differential_flatness_controller``.
* `controller_base.hpp <https://github.com/aerostack2/aerostack2/blob/main/as2_motion_controller/include/as2_motion_controller/controller_base.hpp>`_
  — authoritative documentation of every base method and helper.
* :ref:`as2_concepts_motion_controller` — high-level role of the controller
  inside Aerostack2.
