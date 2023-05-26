.. _development_tutorials_motion_controller:

======================================
Writing a New Motion Controller Plugin
======================================

.. contents:: Table of Contents
   :depth: 1
   :local:



.. _development_tutorials_motion_controller_overview:

--------
Overview
--------

This tutorial shows how to create your own Controller within the AeroStack2 framework.
You may want to remember AeroStack2 :ref:`Architecture <as2_concepts_architecture>` and its :ref:`Controller <as2_concepts_motion_controller>`
operation.

This tutorial will be a walktrough of one existing controller to easily understand its creation.
The one explained will be a simplified version of the PID controller (:ref:`speed_controller`).
The code used in this tutorial can be found in `Github 
<https://github.com/aerostack2/aerostack2/tree/main/as2_controller/as2_controller_plugin_speed_controller>`_.



.. _development_tutorials_motion_controller_requirements:

------------
Requirements
------------

- ROS 2 Humble
- AeroStack2



.. _development_tutorials_motion_controller_steps:

--------------
Tutorial Steps
--------------



.. _development_tutorials_motion_controller_steps_class:

1. Controller Plugin Base
=========================

Following the plugin structure, all the controllers should inherit from a base class
``controller_plugin_base::ControllerBase``. The base class provides a set of virtual methods
to override with expected controller functionality. The list of methods is presented in the
table below:

.. list-table:: List of Virtual Methods
   :header-rows: 1

   * - Virtual method
     - Method description
     - Requires override?
   * - ownInitialize()
     - Controller plugin own initialize method.
     - No
   * - updateState()
     - State update with the information received from the current pose and twist.
     - Yes
   * - updateReference()
     - Update the pose, twist, trajectory and thrust references. Use only the ones you need.
     - No
   * - computeOutput()
     - Compute the controller output signal.
     - Yes
   * - setMode()
     - Update the controller input and output control modes.
     - Yes
   * - updateParams()
     - Update the controller parameters.
     - Yes
   * - reset()
     - Reset the controller inner state.
     - Yes
   * - getDesiredPoseFrameId()
     - Return the desired pose state and reference frame_id. ``odom`` by default.
     - No
   * - getDesiredTwistFrameId()
     - Return the desired twist state and reference frame_id. ``base_link`` by default.
     - No



.. _development_tutorials_motion_controller_steps_methods:

2. Overriden methods
====================



.. _development_tutorials_motion_controller_steps_methods_init:

Initialization
--------------

**TDB**

.. code-block:: c++

    void Plugin::ownInitialize() {
        speed_limits_ = Eigen::Vector3d::Zero();

        pid_yaw_handler_                 = std::make_shared<pid_controller::PIDController>();
        pid_3D_position_handler_         = std::make_shared<pid_controller::PIDController3D>();
        pid_3D_velocity_handler_         = std::make_shared<pid_controller::PIDController3D>();
        pid_1D_speed_in_a_plane_handler_ = std::make_shared<pid_controller::PIDController>();
        pid_3D_speed_in_a_plane_handler_ = std::make_shared<pid_controller::PIDController3D>();
        pid_3D_trajectory_handler_       = std::make_shared<pid_controller::PIDController3D>();

        tf_handler_ = std::make_shared<as2::tf::TfHandler>(node_ptr_);

        enu_frame_id_ = as2::tf::generateTfName(node_ptr_, enu_frame_id_);
        flu_frame_id_ = as2::tf::generateTfName(node_ptr_, flu_frame_id_);

        input_pose_frame_id_  = as2::tf::generateTfName(node_ptr_, input_pose_frame_id_);
        input_twist_frame_id_ = as2::tf::generateTfName(node_ptr_, input_twist_frame_id_);

        output_twist_frame_id_ = as2::tf::generateTfName(node_ptr_, output_twist_frame_id_);

        reset();
        return;
    }



.. _development_tutorials_motion_controller_steps_methods_state:

Update State
------------

**TBD**

.. code-block:: c++

    void Plugin::updateState(const geometry_msgs::msg::PoseStamped &pose_msg,
                            const geometry_msgs::msg::TwistStamped &twist_msg) {
        uav_state_.position = Eigen::Vector3d(pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z);
        uav_state_.velocity = Eigen::Vector3d(twist_msg.twist.linear.x, twist_msg.twist.linear.y, twist_msg.twist.linear.z);
        uav_state_.yaw.x() = as2::frame::getYawFromQuaternion(pose_msg.pose.orientation);

        if (hover_flag_) {
            resetReferences();
            flags_.ref_received = true;
            hover_flag_         = false;
        }

        flags_.state_received = true;
        return;
    }



.. _development_tutorials_motion_controller_steps_methods_reference:

Update Reference
----------------

**TBD**

.. code-block:: c++

    void Plugin::updateReference(const geometry_msgs::msg::PoseStamped &pose_msg) {
        if (control_mode_in_.control_mode == as2_msgs::msg::ControlMode::POSITION ||
            control_mode_in_.control_mode == as2_msgs::msg::ControlMode::SPEED_IN_A_PLANE) {
            control_ref_.position = Eigen::Vector3d(pose_msg.pose.position.x, pose_msg.pose.position.y,
                                                    pose_msg.pose.position.z);
            flags_.ref_received   = true;
        }

        if ((control_mode_in_.control_mode == as2_msgs::msg::ControlMode::SPEED ||
            control_mode_in_.control_mode == as2_msgs::msg::ControlMode::POSITION ||
            control_mode_in_.control_mode == as2_msgs::msg::ControlMode::SPEED_IN_A_PLANE) &&
            control_mode_in_.yaw_mode == as2_msgs::msg::ControlMode::YAW_ANGLE) {
            control_ref_.yaw.x() = as2::frame::getYawFromQuaternion(pose_msg.pose.orientation);
        }

        return;
    }

    void Plugin::updateReference(const geometry_msgs::msg::TwistStamped &twist_msg) {
        if (control_mode_in_.control_mode == as2_msgs::msg::ControlMode::POSITION) {
            speed_limits_ = Eigen::Vector3d(twist_msg.twist.linear.x, twist_msg.twist.linear.y,
                                            twist_msg.twist.linear.z);
            pid_3D_position_handler_->setOutputSaturation(speed_limits_);
            pid_3D_velocity_handler_->setOutputSaturation(speed_limits_);
            pid_3D_trajectory_handler_->setOutputSaturation(speed_limits_);
            return;
        }

        if (control_mode_in_.control_mode != as2_msgs::msg::ControlMode::SPEED &&
            control_mode_in_.control_mode != as2_msgs::msg::ControlMode::SPEED_IN_A_PLANE) {
            return;
        }

        control_ref_.velocity =
            Eigen::Vector3d(twist_msg.twist.linear.x, twist_msg.twist.linear.y, twist_msg.twist.linear.z);

        if (control_mode_in_.yaw_mode == as2_msgs::msg::ControlMode::YAW_SPEED) {
            control_ref_.yaw.y() = twist_msg.twist.angular.z;
        }

        flags_.ref_received = true;
        return;
    }

    void Plugin::updateReference(const as2_msgs::msg::TrajectoryPoint &traj_msg) {
        if (control_mode_in_.control_mode != as2_msgs::msg::ControlMode::TRAJECTORY) {
            return;
        }

        control_ref_.position =
            Eigen::Vector3d(traj_msg.position.x, traj_msg.position.y, traj_msg.position.z);

        control_ref_.velocity = Eigen::Vector3d(traj_msg.twist.x, traj_msg.twist.y, traj_msg.twist.z);

        control_ref_.yaw.x() = traj_msg.yaw_angle;

        flags_.ref_received = true;
        return;
    }

Thrust is not overriden since is not needed.



.. _development_tutorials_motion_controller_steps_methods_output:

Compute Output
--------------

**TBD**

.. code-block:: c++

    bool Plugin::computeOutput(double dt,
                            geometry_msgs::msg::PoseStamped &pose,
                            geometry_msgs::msg::TwistStamped &twist,
                            as2_msgs::msg::Thrust &thrust) {
        if (!flags_.state_received) {
            auto &clk = *node_ptr_->get_clock();
            RCLCPP_WARN_THROTTLE(node_ptr_->get_logger(), clk, 5000, "State not received yet");
            return false;
        }

        if (!flags_.ref_received) {
            auto &clk = *node_ptr_->get_clock();
            RCLCPP_WARN_THROTTLE(node_ptr_->get_logger(), clk, 5000,
                                "State changed, but ref not recived yet");
            return false;
        }

        resetCommands();

        switch (control_mode_in_.control_mode) {
            case as2_msgs::msg::ControlMode::HOVER:
            case as2_msgs::msg::ControlMode::POSITION:
            control_command_.velocity =
                pid_3D_position_handler_->computeControl(dt, uav_state_.position, control_ref_.position);

            control_command_.velocity = pid_3D_position_handler_->saturateOutput(
                control_command_.velocity, speed_limits_, proportional_limitation_);
            break;
            case as2_msgs::msg::ControlMode::SPEED: {
            if (use_bypass_) {
                control_command_.velocity = control_ref_.velocity;
            } else {
                control_command_.velocity = pid_3D_velocity_handler_->computeControl(
                    dt, uav_state_.velocity, control_ref_.velocity);
            }
            break;
            }
            case as2_msgs::msg::ControlMode::SPEED_IN_A_PLANE: {
            if (use_bypass_) {
                control_command_.velocity = control_ref_.velocity;
            } else {
                control_command_.velocity = pid_3D_speed_in_a_plane_handler_->computeControl(
                    dt, uav_state_.velocity, control_ref_.velocity);
            }

            control_command_.velocity.z() = pid_1D_speed_in_a_plane_handler_->computeControl(
                dt, uav_state_.position.z(), control_ref_.position.z());

            break;
            }
            case as2_msgs::msg::ControlMode::TRAJECTORY: {
            control_command_.velocity =
                pid_3D_trajectory_handler_->computeControl(dt, uav_state_.position, control_ref_.position,
                                                            uav_state_.velocity, control_ref_.velocity);

            control_command_.velocity = pid_3D_trajectory_handler_->saturateOutput(
                control_command_.velocity, speed_limits_, proportional_limitation_);
            break;
            }
            default:
            auto &clk = *node_ptr_->get_clock();
            RCLCPP_ERROR_THROTTLE(node_ptr_->get_logger(), clk, 5000, "Unknown control mode");
            return false;
            break;
        }

        switch (control_mode_in_.yaw_mode) {
            case as2_msgs::msg::ControlMode::YAW_ANGLE: {
            double yaw_error = as2::frame::angleMinError(control_ref_.yaw.x(), uav_state_.yaw.x());
            control_command_.yaw_speed = pid_yaw_handler_->computeControl(dt, yaw_error);
            break;
            }
            case as2_msgs::msg::ControlMode::YAW_SPEED: {
            control_command_.yaw_speed = control_ref_.yaw.y();
            break;
            }
            default:
            auto &clk = *node_ptr_->get_clock();
            RCLCPP_ERROR_THROTTLE(node_ptr_->get_logger(), clk, 5000, "Unknown yaw mode");
            return false;
            break;
        }

        return getOutput(twist);
    }



.. _development_tutorials_motion_controller_steps_methods_mode:

Set Mode
--------

**TBD**

.. code-block:: c++


    bool Plugin::setMode(const as2_msgs::msg::ControlMode &in_mode,
                        const as2_msgs::msg::ControlMode &out_mode) {
        if (!flags_.plugin_parameters_read) {
            RCLCPP_WARN(node_ptr_->get_logger(), "Plugin parameters not read yet, can not set mode");
            return false;
        }

        if (!flags_.position_controller_parameters_read) {
            RCLCPP_WARN(node_ptr_->get_logger(),
                        "Position controller parameters not read, can not set mode");
            for (auto &param : position_control_parameters_to_read_) {
            RCLCPP_WARN(node_ptr_->get_logger(), "Parameter %s not read", param.c_str());
            }
            return false;
        }

        if (in_mode.control_mode == as2_msgs::msg::ControlMode::TRAJECTORY &&
            !flags_.trajectory_controller_parameters_read) {
            RCLCPP_WARN(node_ptr_->get_logger(),
                        "Trajectory controller parameters not read yet, can not set mode to TRAJECTORY");
            for (auto &param : trajectory_control_parameters_to_read_) {
            RCLCPP_WARN(node_ptr_->get_logger(), "Parameter %s not read", param.c_str());
            }
            return false;
        } else if ((in_mode.control_mode == as2_msgs::msg::ControlMode::SPEED ||
                    in_mode.control_mode == as2_msgs::msg::ControlMode::SPEED_IN_A_PLANE) &&
                    (!flags_.velocity_controller_parameters_read && !use_bypass_)) {
            RCLCPP_WARN(node_ptr_->get_logger(),
                        "Velocity controller parameters not read yet and bypass is not used, can not set "
                        "mode to SPEED or SPEED_IN_A_PLANE");
            if (in_mode.control_mode == as2_msgs::msg::ControlMode::SPEED) {
            for (auto &param : velocity_control_parameters_to_read_) {
                RCLCPP_WARN(node_ptr_->get_logger(), "Parameter %s not read", param.c_str());
            }
            } else {
            for (auto &param : speed_in_a_plane_control_parameters_to_read_) {
                RCLCPP_WARN(node_ptr_->get_logger(), "Parameter %s not read", param.c_str());
            }
            }
            return false;
        }

        if (in_mode.yaw_mode == as2_msgs::msg::ControlMode::YAW_ANGLE &&
            !flags_.yaw_controller_parameters_read) {
            RCLCPP_WARN(node_ptr_->get_logger(),
                        "Yaw controller parameters not read yet, can not set mode to YAW_ANGLE");
            return false;
        }

        if (in_mode.control_mode == as2_msgs::msg::ControlMode::HOVER) {
            control_mode_in_.control_mode    = in_mode.control_mode;
            control_mode_in_.yaw_mode        = as2_msgs::msg::ControlMode::YAW_ANGLE;
            control_mode_in_.reference_frame = as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME;
            hover_flag_                      = true;
        } else {
            control_mode_in_ = in_mode;
        }

        flags_.state_received = false;
        flags_.ref_received   = false;
        control_mode_out_     = out_mode;

        if (control_mode_in_.control_mode == as2_msgs::msg::ControlMode::HOVER ||
            control_mode_in_.control_mode == as2_msgs::msg::ControlMode::POSITION ||
            control_mode_in_.control_mode == as2_msgs::msg::ControlMode::TRAJECTORY) {
            input_pose_frame_id_   = enu_frame_id_;
            output_twist_frame_id_ = enu_frame_id_;
        } else if (control_mode_in_.control_mode == as2_msgs::msg::ControlMode::SPEED ||
                    control_mode_in_.control_mode == as2_msgs::msg::ControlMode::SPEED_IN_A_PLANE) {
            input_pose_frame_id_ = enu_frame_id_;
            switch (control_mode_out_.reference_frame) {
            case as2_msgs::msg::ControlMode::BODY_FLU_FRAME:
                input_twist_frame_id_  = flu_frame_id_;
                output_twist_frame_id_ = flu_frame_id_;
                break;
            case as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME:
            default:
                input_twist_frame_id_  = enu_frame_id_;
                output_twist_frame_id_ = enu_frame_id_;
                break;
            }
        }

        return true;
    }



.. _development_tutorials_motion_controller_steps_methods_params:

Parameters Update
-----------------

**TBD**

.. code-block:: c++

    bool Plugin::updateParams(const std::vector<std::string> &_params_list) {
        auto result = parametersCallback(node_ptr_->get_parameters(_params_list));
        return result.successful;
    };



.. _development_tutorials_motion_controller_steps_methods_reset:

Reset
-----

**TBD**

.. code-block:: c++

    void Plugin::reset() {
        resetReferences();
        resetState();
        resetCommands();
        pid_yaw_handler_->resetController();
        pid_3D_position_handler_->resetController();
        pid_3D_velocity_handler_->resetController();
        pid_3D_trajectory_handler_->resetController();
    }



.. _development_tutorials_motion_controller_steps_export:

3. Exporting the plugin
=======================

Now that we have created our new controller, we need to export it so that it would be visible
to the Controller Manager. Plugins are loaded at runtime and if they are not visible, then our
controller manager won't be able to load it. In ROS 2, exporting and loading plugins is handled
by ``pluginlib``.

To achieve that, class ``controller_plugin_speed_controller::Plugin`` is loaded dynamically from
``controller_plugin_base::ControllerBase`` which is the base class.

1. To export the controller, we need to provide two lines

.. code-block:: c++

    #include <pluginlib/class_list_macros.hpp>
    PLUGINLIB_EXPORT_CLASS(controller_plugin_speed_controller::Plugin, controller_plugin_base::ControllerBase)

It is good practice to place these lines at the end of the file but technically, you can also
write at the top.

2. Next step would be to create plugin's description file in the root directory of the package.
For example, ``plugins.xml`` file in our tutorial package. This file contains following 
information:

- `library path`: Plugin's library name and it's location.
- `class name`: Name of the class.
- `class type`: Type of class.
- `base class`: Name of the base class.
- `description`: Description of the plugin.

.. code-block:: xml

    <library path="as2_controller_plugin_speed_controller">
        <class type="controller_plugin_speed_controller::Plugin" base_class_type="controller_plugin_base::ControllerBase">
            <description>Controller plugin for speed controller.</description>
        </class>
    </library>

3. Next step would be to export plugin using ``CMakeLists.txt`` by using cmake function 
``pluginlib_export_plugin_description_file()``. This function installs plugin description 
file to share directory and sets ament indexes to make it discoverable.

.. code-block:: cmake

    pluginlib_export_plugin_description_file(as2_controller_plugin_base plugins.xml)

4. Compile and it should be registered. Next, we'll use this plugin.



.. _development_tutorials_motion_controller_steps_config:

4. Controller manager and configuration files
=============================================

To use the plugin, we should create two configuration files.

**TBD**

.. code-block:: yaml

    input_control_modes:
     - 0b00000000 # UNSET
     - 0b00010000 # HOVER
     # - 0b00100100 # ACRO (p,q,r, Thrust)
     # - 0b00110001 # ATTITUDE with yaw ANGLE ( r,p,y , Thrust) 
     # - 0b00110101 # ATTITUDE with yaw SPEED ( r,p, dy , Thrust) 
     - 0b01000000 # SPEED with yaw ANGLE in the LOCAL_FLU_FRAME
     - 0b01000001 # SPEED with yaw ANGLE in the GLOBAL_ENU_FRAME
     - 0b01000100 # SPEED with yaw SPEED in the LOCAL_FLU_FRAME
     - 0b01000101 # SPEED with yaw SPEED in the GLOBAL_ENU_FRAME
     - 0b01010000 # SPEED_IN_A_PLANE with yaw ANGLE in the LOCAL_FLU_FRAME
     - 0b01010001 # SPEED_IN_A_PLANE with yaw ANGLE in the GLOBAL_ENU_FRAME
     - 0b01010100 # SPEED_IN_A_PLANE with yaw SPEED in the LOCAL_FLU_FRAME
     - 0b01010101 # SPEED_IN_A_PLANE with yaw SPEED in the GLOBAL_ENU_FRAME
     - 0b01100001 # POSITION with yaw ANGLE in the GLOBAL_ENU_FRAME
     - 0b01100101 # POSITION with yaw SPEED in the GLOBAL_ENU_FRAME
     - 0b01110001 # TRAJECTORY with yaw ANGLE in the GLOBAL_ENU_FRAME
     - 0b01110101 # TRAJECTORY with yaw SPEED in the GLOBAL_ENU_FRAME

   output_control_modes:
     - 0b00000000 # UNSET
     # - 0b00010000 # HOVER
     # - 0b00100100 # ACRO (p,q,r, Thrust)
     # - 0b00110001 # ATTITUDE with yaw ANGLE ( r,p,y , Thrust) 
     # - 0b00110101 # ATTITUDE with yaw SPEED ( r,p, dy , Thrust) 
     # - 0b01000000 # SPEED with yaw ANGLE in the LOCAL_FLU_FRAME
     # - 0b01000001 # SPEED with yaw ANGLE in the GLOBAL_ENU_FRAME
     - 0b01000100 # SPEED with yaw SPEED in the LOCAL_FLU_FRAME
     - 0b01000101 # SPEED with yaw SPEED in the GLOBAL_ENU_FRAME
     # - 0b01010000 # SPEED_IN_A_PLANE with yaw ANGLE in the LOCAL_FLU_FRAME
     # - 0b01010001 # SPEED_IN_A_PLANE with yaw ANGLE in the GLOBAL_ENU_FRAME
     # - 0b01010100 # SPEED_IN_A_PLANE with yaw SPEED in the LOCAL_FLU_FRAME
     # - 0b01010101 # SPEED_IN_A_PLANE with yaw SPEED in the GLOBAL_ENU_FRAME
     # - 0b01100001 # POSITION with yaw ANGLE in the GLOBAL_ENU_FRAME
     # - 0b01100101 # POSITION with yaw SPEED in the GLOBAL_ENU_FRAME
     # - 0b01110001 # TRAJECTORY with yaw ANGLE in the GLOBAL_ENU_FRAME
     # - 0b01110101 # TRAJECTORY with yaw SPEED in the GLOBAL_ENU_FRAME

**TBD**

.. code-block:: yaml

    /**:
        ros__parameters:
            proportional_limitation: true
            position_control:
                reset_integral: false
                antiwindup_cte: 0.0
                alpha: 0.0
                kp:
                    x: 0.0
                    y: 0.0
                    z: 0.0
                kd:
                    x: 0.0
                    y: 0.0
                    z: 0.0
                ki:
                    x: 0.0
                    y: 0.0
                    z: 0.0
            speed_control:
                reset_integral: false
                antiwindup_cte: 0.0
                alpha: 0.0
                kp:
                    x: 0.0
                    y: 0.0
                    z: 0.0
                kd:
                    x: 0.0
                    y: 0.0
                    z: 0.0
                ki:
                    x: 0.0
                    y: 0.0
                    z: 0.0
            speed_in_a_plane_control:
                reset_integral: false
                antiwindup_cte: 0.0
                alpha: 0.0
                height:
                    kp: 0.0
                    kd: 0.0
                    ki: 0.0
                speed:
                    kp:
                    x: 0.0
                    y: 0.0
                    kd:
                    x: 0.0
                    y: 0.0
                    ki:
                    x: 0.0
                    y: 0.0
            trajectory_control:
                reset_integral: false
                antiwindup_cte: 0.0
                alpha: 0.0
                kp:
                    x: 0.0
                    y: 0.0
                    z: 0.0
                kd:
                    x: 0.0
                    y: 0.0
                    z: 0.0
                ki:
                    x: 0.0
                    y: 0.0
                    z: 0.0
            yaw_control:
                reset_integral: false
                antiwindup_cte: 0.0
                alpha: 0.0
                kp: 0.0
                kd: 0.0
                ki: 0.0



.. _development_tutorials_motion_controller_steps_launch:

5. Launching
============

¿**TBD**?
