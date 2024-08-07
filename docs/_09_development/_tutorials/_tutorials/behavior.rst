.. _development_tutorials_behavior:

======================
Writing a New Behavior
======================

.. contents:: Table of Contents
   :depth: 1
   :local:



.. _development_tutorials_behavior_overview:

--------
Overview
--------

This tutorial shows how to create your brand new Behavior. To understand what is an
AeroStack2 Behavior, please revisit :ref:`behaviors` section.

For better understanding, this tutorial will be a walktrough of one existing behavior (Takeoff).
As you already now, a behavior is formed by a Server and a Client. The code used in this tutorial 
can be found in Github (`Server 
<https://github.com/aerostack2/aerostack2/tree/main/as2_behaviors/as2_movement_behaviors/take_off>`_, 
`Client <https://github.com/aerostack2/aerostack2/tree/main/as2_python_api/as2_python_api/behavior_actions>`_).



.. _development_tutorials_behavior_requirements:

------------
Requirements
------------

- ROS 2 Humble
- AeroStack2



.. _development_tutorials_behavior_server:

---------------
Behavior Server
---------------

All behavior servers should inherit from a common base class, called ``as2_behavior::BehaviorServer``.
This class has a set of virtual methods to override to fulfill with your behavior implementation.
These methods are shown in the table below:

.. list-table:: List of Virtual Methods
   :header-rows: 1

   * - Virtual method
     - Method description
     - Requires override?
   * - on_activate()
     - Tasks to do when starting the behavior.
     - Yes
   * - on_modify()
     - Tasks to do when modifying the behavior.
     - Yes
   * - on_deactivate()
     - Tasks to do when canceling the behavior.
     - Yes
   * - on_pause()
     - Tasks to do when pausing the behavior.
     - Yes
   * - on_resume()
     - Tasks to do when resuming the behavior.
     - Yes
   * - on_run()
     - Tasks to do on each execution cycle.
     - Yes
   * - on_execution_end()
     - Tasks to do after finishing the behavior.
     - No

For the example, the Takeoff Behavior would be something similar to:

.. code-block:: cpp

    TakeOffBehavior() : as2_behavior::BehaviorServer<as2_msgs::action::TakeOff>(as2_names::actions::behaviors::takeoff) {
        platform_takeoff_cli_ = node_ptr_->create_client<std_srvs::srv::SetBool>(as2_names::services::platform::takeoff);
        platform_takeoff_request_ = std::make_shared<std_srvs::srv::SetBool::Request>();
        platform_takeoff_request_->data = true;
        return;
    }

    bool on_activate(std::shared_ptr<const as2_msgs::action::TakeOff::Goal> goal) override {
        platform_takeoff_future_ = platform_takeoff_cli_->async_send_request(platform_takeoff_request_);

        if (!platform_takeoff_future_.valid()) {
            RCLCPP_ERROR(node_ptr_->get_logger(), "Request could not be sent");
            return false;
        }
        return true;
    }

    bool on_modify(
        std::shared_ptr<const as2_msgs::action::TrajectoryGenerator::Goal> goal)
        override;

    bool on_deactivate(const std::shared_ptr<std::string> &message) override {
        RCLCPP_INFO(node_ptr_->get_logger(), "Takeoff can not be cancelled");
        return false;
    }

    bool on_pause(const std::shared_ptr<std::string> &message) override {
        RCLCPP_INFO(node_ptr_->get_logger(), "Takeoff can not be paused");
        return false;
    }

    bool on_resume(const std::shared_ptr<std::string> &message) override {
        RCLCPP_INFO(node_ptr_->get_logger(), "Takeoff can not be resumed");
        return false;
    }

    as2_behavior::ExecutionStatus on_run(const std::shared_ptr<const as2_msgs::action::TrajectoryGenerator::Goal> &goal,
                                        std::shared_ptr<as2_msgs::action::TrajectoryGenerator::Feedback> &feedback_msg,
                                        std::shared_ptr<as2_msgs::action::TrajectoryGenerator::Result> &result_msg) override {
        if (platform_takeoff_future_.valid() && platform_takeoff_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            auto result = platform_takeoff_future_.get();
            if (result->success) {
                result_.takeoff_success = true;
                return as2_behavior::ExecutionStatus::SUCCESS;
            } else {
                result_.takeoff_success = false;
                return as2_behavior::ExecutionStatus::FAILURE;
            }
        }
        return as2_behavior::ExecutionStatus::RUNNING;
    }

    void on_execution_end(const as2_behavior::ExecutionStatus &state) override {
        RCLCPP_INFO(node_ptr_->get_logger(), "Takeoff end");
        return;
    }



.. _development_tutorials_behavior_client:

---------------
Behavior Client
---------------

The client side will explained with an example in Python. However, feel free to code your Behavior
in C++ or any other ROS 2 supported language.

The Behavior class inherits from a common behavior handler which implements all common features to
the AeroStack2 behaviors. Methods like ``pause``, ``resume`` or ``cancel`` are equal between behaviors,
so the user doesn't need to code them. Other aspects like topics, services or action management is also
solved by the parent class.

So what do you need to do? Just implement the methods that vary from one behavior to another,
the ``start`` and ``modify`` methods.

.. code-block:: python

    from as2_msgs.action import TakeOff

    from ..behavior_actions.behavior_handler import BehaviorHandler

    class TakeoffBehavior(BehaviorHandler):
        """Takeoff Behavior"""

        def __init__(self, drone) -> None:
            self.__drone = drone

            try:
                super().__init__(drone, TakeOff, 'TakeOffBehaviour')
            except self.BehaviorNotAvailable as err:
                self.__drone.get_logger().warn(str(err))

        def start(self, height: float, speed: float, wait_result: bool = True) -> bool:
            goal_msg = TakeOff.Goal()
            goal_msg.takeoff_height = float(height)
            goal_msg.takeoff_speed = float(speed)

            try:
                return super().start(goal_msg, wait_result)
            except self.GoalRejected as err:
                self.__drone.get_logger().warn(str(err))
            return False

        def modify(self, height: float, speed: float) -> bool:
            goal_msg = TakeOff.Goal()
            goal_msg.takeoff_height = height
            goal_msg.takeoff_speed = speed

            return super().modify(goal_msg)

.. note::

    Don't forget to call the parent methods using ``super()``.
