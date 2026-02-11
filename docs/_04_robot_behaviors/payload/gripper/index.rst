.. _gripper:

Gripper Behavior
================

The ``gripper_behavior`` is a core component of the 
``as2_behaviors_payload`` package in Aerostack2.

This behavior provides a **plugin-based interface** to control different types of grippers mounted on aerial platforms. It allows drones to pick up and release objects during flight by interfacing with various gripper hardware implementations.

Working Principle
-----------------

The behavior operates through a plugin architecture that abstracts the specific gripper hardware:

* **Plugin Base Interface** (``GripperBehaviorPluginBase``): Defines the common interface that all gripper plugins must implement.
* **Behavior Node** (``GripperBehavior``): Manages the ROS 2 action server and coordinates plugin execution.
* **Hardware Plugins**: Specific implementations for different gripper types.

Available Plugins
^^^^^^^^^^^^^^^^^

The package includes two gripper plugin implementations:

1. **DC Servo Gripper**
   
   Controls a gripper using a DC servo motor via PWM signals.
   
   * Configuration: ``plugins/dc_servo/config/plugin_default.yaml``
   * Launch: ``dc_servo_gripper.launch.py``

2. **Two Fingers Gripper**
   
   Controls a two-finger parallel gripper mechanism to use with Gazebo.
   
   * Configuration: ``plugins/two_fingers/config/plugin_default.yaml``
   * Launch: ``two_fingers.launch.py``

Behavior Actions
----------------

- **Modify**: To open and close the gripper finges it must be use the modify service.
  
  - If you modify the action to True, the fingers will  **close**.
  - If you modify the action to False, the fingers will **open**.


Configuration Parameters
------------------------

The behavior can be configured via the ``plugins/<plugin_name>/config/plugin_default.yaml`` file. Each plugin has its own specific configuration.

1. **DC Servo Gripper Plugin Parameters**

.. list-table:: Configuration Parameters
   :widths: 30 70
   :header-rows: 1

   * - Parameter
     - Description
   * - **angle_to_open**
     - The angle in degrees to which the gripper opens.
   * - **angle_to_close**
     - The angle in degrees to which the gripper closes.
   * - **max_angle**
     - Maximum angle in degrees to limit for the gripper.
   * - **min_angle**
     - Minimum angle in degrees to limit for the gripper.
   * - **duty_min**
     - The minimum duty cycle for the PWM signal controlling the servo.
   * - **duty_max**
     - The maximum duty cycle for the PWM signal controlling the servo.
   * - **topic_pwm**
     - The ROS topic where the PWM commands are published.
  
2. **Two Fingers Gripper Plugin Parameters**

.. list-table:: Configuration Parameters
   :widths: 30 70
   :header-rows: 1

   * - Parameter
     - Description
   * - **l_finger_open**
     - Message to open the left finger.
   * - **r_finger_open**
     - Message to open the right finger.
   * - **l_finger_close**
     - Message to close the left finger.
   * - **r_finger_close**
     - Message to close the right finger.
   * - **topic_l_finger**
     - The ROS topic to send to Gazebo for the left finger.
   * - **topic_r_finger**
     - The ROS topic to send to Gazebo for the right finger.
Launching the Behavior
----------------------

You can launch this behavior with different plugin configurations:

**1. DC Servo Gripper**

.. code-block:: bash

   ros2 launch as2_behaviors_payload dc_servo_gripper.launch.py namespace:=<drone_namespace>

**Sending the Goal manually:**

.. code-block:: bash

   ros2 action send_goal /<namespace>/GripperHandlerBehavior as2_msgs/action/GripperHandler "{request_gripper: <bool>}"

**2. Two Fingers Gripper**

.. code-block:: bash

   ros2 launch as2_behaviors_payload two_fingers.launch.py namespace:=<drone_namespace>

**Sending the Goal manually:**

.. code-block:: bash
    
   ros2 action send_goal /<namespace>/GripperHandlerBehavior as2_msgs/action/GripperHandler "{request_gripper: <bool>}"

