.. _writing_new_aerial_platform:

Writing a New Aerial Platform
*****************************

- `Overview`_
- `Requirements`_
- `Tutorial Steps`_

Overview
========

This tutorial shows how to create your own Aerial Platform. To understand what is an
AeroStack2 Aerial Platform, please revisit :ref:`aerial_platforms`.

For better understanding, this tutorial will be a walktrough of one existing platform.
Due to accesibility, :ref:`ign_gazebo_platform` will be the chosen one. The code used in
this tutorial can be found in `Github 
<https://github.com/aerostack2/aerostack2/tree/main/as2_aerial_platforms/as2_ignition_platform>`_.

.. note::

    Just as a reminder, an aerial platform has two sides. One who deals with AeroStack2
    communication (ROS2) and another who deals with the specific hardware platform.
    Since all platforms inherit from a common parent class, the user should only focus on
    how to communicate (send commands and receive data) with the platform hardware. ROS2
    communication will be done by the base class, who has all the subscribers and publishers
    needed in AeroStack2.

Requirements
============

- ROS 2 Humble
- AeroStack2
- Gazebo Ignition Fortress
- Quadrotor Gazebo model (as2_gazebo_assets, already included within AeroStack2)

Tutorial Steps
==============

1. Abstract class, Aerial Platform
----------------------------------

All Aerial Platforms should inherit from a common base class ``as2::AerialPlatform``, an 
extension of a ``rclcpp::Node``. It provides the basic functionality for the platform and 
is responsible for handling the platform state machine and the platform status. It also 
handles the command subscriptions and the basic platform services.

The base class provides a set of virtual methods to implement a platform. It also offers 
a group of protected attributes where the data received from the ROS2 side is received.
The list of methods is presented in the table below:

.. list-table:: List of Virtual Methods
   :header-rows: 1

   * - Virtual method
     - Method description
     - Requires override?
   * - configureSensors()
     - Configures the platform sensors.
     - Yes
   * - ownSendCommand()
     - Handles how a command must be sended in a specific platform.
     - Yes
   * - ownSetArmingState()
     - Handles how arming state has to be settled in a specific platform.
     - Yes
   * - ownSetOffboardControl()
     - Handles how offboard mode has to be settled in a specific platform.
     - Yes
   * - ownSetPlatformControlMode()
     - Handles how the control mode has to be settled in a specific platform.
     - Yes
   * - ownKillSwitch()
     - Handles the platform emergency kill switch command (stop the motors inmediately).
       Can not be reversed. USE WITH CAUTION.
     - Yes
   * - ownStopPlatform()
     - Handles the platform emergency stop command. STOP means to hover as best as possible.
       This hover is different from the hover in the platform control mode. And when it is
       activated the platform will stop hearing commands from AS2. USE WITH CAUTION.
     - Yes
   * - ownTakeoff()
     - Handles the platform takeoff command.
     - No
   * - ownLand()
     - Handles the platform landing command.
     - No

Besides, the group of protected attributes are:

.. list-table:: List of Virtual Methods
   :header-rows: 1

   * - Attribute
     - Type
     - Description
   * - ``command_pose_msg_``
     - geometry_msgs::msg::PoseStamped
     - Pose msg received from ``actuator_command/pose`` topic interface.
   * - ``command_twist_msg_``
     - geometry_msgs::msg::TwistStamped
     - Twist msg received from ``actuator_command/twist`` topic interface.
   * - ``command_thrust_msg_``
     - as2_msgs::msg::Thrust
     - Pose msg received from ``actuator_command/thrust`` topic interface.
   * - ``platform_info_msg_``
     - as2_msgs::msg::PlatformInfo
     - Platform information data container. Readable only.

2. Overriden methods
--------------------

Class constuctor
################

Besides the overriden methods, it is necessary to create all the interface communication with the
hardware specific hardware. In this case, only two publishers are needed to arm and send commands
to de platform.

.. code-block:: c++

    IgnitionPlatform::IgnitionPlatform() : as2::AerialPlatform() {
        this->declare_parameter<std::string>("cmd_vel_topic");  // Reading topic from launch argument
        std::string cmd_vel_topic_param = this->get_parameter("cmd_vel_topic").as_string();

        this->declare_parameter<std::string>("arm_topic");  // Reading topic from launch argument
        std::string arm_topic_param = this->get_parameter("arm_topic").as_string();

        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_param, rclcpp::QoS(1));
        arm_pub_ = this->create_publisher<std_msgs::msg::Bool>(arm_topic_param, rclcpp::QoS(1));
    }

Sensor configuration
####################

There is no need to call this method manually. Base class will call it after instantiation. 

.. code-block:: c++

    void IgnitionPlatform::configureSensors() {
        // Not sensors used within the platform
    }

In this specific platform, there are no need of configure any sensors due to ros-gazebo bridge,
which is publishing directly the sensors in ROS topics.
You can find more information about this at :ref:`ign_gazebo_platform`.

.. note::

    **TDB**: Reference how to use Sensors

Control mode
############

When a control mode reach this method, it can be assumed that the control mode is supported by the
platform (see `3. Control modes configuration`_).

.. code-block:: c++

    bool IgnitionPlatform::ownSetPlatformControlMode(const as2_msgs::msg::ControlMode &control_in) {
        RCLCPP_INFO(this->get_logger(), "Control mode: [%s]",
                    as2::control_mode::controlModeToString(control_in).c_str());
        control_in_ = control_in;  // storing control mode
        return true;
    }

.. note::

    **TDB**: Reference how to control modes work

Send command
############

Twist msg received from ``actuator_command/twist`` topic is sent to the simulated drone. Before,
current control mode is checked to send null speed if HOVER mode is active.

.. code-block:: c++

    bool IgnitionPlatform::ownSendCommand() {
        if (control_in_.control_mode == as2_msgs::msg::ControlMode::HOVER) {
            geometry_msgs::msg::Twist twist_msg;
            twist_msg.linear.x  = 0;
            twist_msg.linear.y  = 0;
            twist_msg.linear.z  = 0;
            twist_msg.angular.x = 0;
            twist_msg.angular.y = 0;
            twist_msg.angular.z = 0;
            twist_pub_->publish(twist_msg);
            return true;
        }

        twist_pub_->publish(command_twist_msg_.twist);
        return true;
    }

.. warning::

    Notice that the sending command msg might need some transformations from the one stored in 
    ``command_twist_msg_`` attribute. In this particular example, we are not performing any 
    transformation since the control mode has the same reference frame that the expected by
    the platform. Remember that the controller publish the command in the reference frame 
    specified in the platform control mode.


Arming
######

Send arming message to Gazebo simulator.

.. code-block:: c++

    bool IgnitionPlatform::ownSetArmingState(bool state) {
        std_msgs::msg::Bool arm_msg;
        arm_msg.data = state;
        arm_pub_->publish(arm_msg);
        resetCommandTwistMsg();
        return true;
    }

Offboard
########

Setting offboard mode before fly is not needed on Gazebo simulator.

.. code-block:: c++

    bool IgnitionPlatform::ownSetOffboardControl(bool offboard) {
        return true;  // Offboard not needed
    }

Emergency kill switch
#####################

Disarms the drone.

.. code-block:: c++

    void IgnitionPlatform::ownKillSwitch() {
        ownSetArmingState(false);
        return;
    }



Emergency stop
##############

Sends null speed to the drone.

.. code-block:: c++

    void IgnitionPlatform::ownStopPlatform() {
        geometry_msgs::msg::Twist twist_msg_hover;
        twist_msg_hover.linear.x  = 0.0;
        twist_msg_hover.linear.y  = 0.0;
        twist_msg_hover.linear.z  = 0.0;
        twist_msg_hover.angular.x = 0.0;
        twist_msg_hover.angular.y = 0.0;
        twist_msg_hover.angular.z = 0.0;

        twist_pub_->publish(twist_msg_hover);
        return;
    }


Takeoff
#######

Gazebo simulator doesn't not have a platform takeoff. Call platform takeoff here if supported.

.. code-block:: c++

    bool IgnitionPlatform::ownTakeoff() {
        RCLCPP_WARN(this->get_logger(), "Takeoff platform not enabled");
        return false;
    }

Land
####

Gazebo simulator doesn't not have a platform land. Call platform land here if supported.

.. code-block:: c++

    bool IgnitionPlatform::ownLand() {
        RCLCPP_WARN(this->get_logger(), "Land platform not enabled");
        return false;
    }


3. Control modes configuration
------------------------------

Valid control modes are defined in a configuration file ``control_modes.yaml``. In the case of
Gazebo simulator, only two control modes are valid:

.. code-block:: yaml

    available_modes:
        # - 0b00000000 # UNSET
        - 0b00010000 # HOVER
        # - 0b00100100 # ACRO (p,q,r, Thrust)
        # - 0b00110001 # ATTITUDE with yaw ANGLE ( r,p,y , Thrust) 
        # - 0b00110101 # ATTITUDE with yaw SPEED ( r,p, dy , Thrust) 
        # - 0b01000000 # SPEED with yaw ANGLE in the LOCAL_FLU_FRAME
        # - 0b01000001 # SPEED with yaw ANGLE in the GLOBAL_ENU_FRAME
        - 0b01000100 # SPEED with yaw SPEED in the LOCAL_FLU_FRAME
        # - 0b01000101 # SPEED with yaw SPEED in the GLOBAL_ENU_FRAME
        # - 0b01010000 # SPEED_IN_A_PLANE with yaw ANGLE in the LOCAL_FLU_FRAME
        # - 0b01010001 # SPEED_IN_A_PLANE with yaw ANGLE in the GLOBAL_ENU_FRAME
        # - 0b01010100 # SPEED_IN_A_PLANE with yaw SPEED in the LOCAL_FLU_FRAME
        # - 0b01010101 # SPEED_IN_A_PLANE with yaw SPEED in the GLOBAL_ENU_FRAME
        # - 0b01100001 # POSITION with yaw ANGLE in the GLOBAL_ENU_FRAME
        # - 0b01100101 # POSITION with yaw SPEED in the GLOBAL_ENU_FRAME
        # - 0b01110001 # TRAJECTORY with yaw ANGLE in the GLOBAL_ENU_FRAME
        # - 0b01110101 # TRAJECTORY with yaw SPEED in the GLOBAL_ENU_FRAME

This configuration explains why no extra comprobation is performed in 
``ownSetPlatformControlMode()`` and why before sending commands only HOVER mode is
checked (``ownSendCommand()``).

.. note::

    **TDB**: Reference how to control modes work