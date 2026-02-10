.. _behaviors_force_estimation:

Force Estimation Behavior
=========================

The ``force_estimation_behavior`` is a core component of the 
``as2_behaviors_param_estimation`` package in Aerostack2.

**Purpose**

This behavior estimates the **external force error** acting on the aerial platform in real-time during flight. It achieves this by comparing the expected force (commanded thrust) against the actual force measured by the onboard IMU.

Working Principle
-----------------

The behavior operates by subscribing to two main data sources:

* **IMU Data** (``/sensor_measurements/imu``): To obtain linear acceleration, specifically along the Z-axis.
* **Thrust Command** (``/actuator_command/thrust``): To obtain the current force commanded to the motors.

Estimation Algorithm
^^^^^^^^^^^^^^^^^^^^

The estimation process is executed as follows:

1.  **Data Synchronization**: 
    When a new thrust command is received, the behavior defines a time window (based on ``threshold_time_sync``). IMU data received within this window is stored and synchronized with the command.

2.  **Acceleration Averaging**: 
    The behavior computes the mean value of the Z-axis acceleration (:math:`{a}_z`) from the synchronized IMU buffer.

3.  **Force Measurement**: 
    The "measured force" is computed using Newton's second law, assuming the current mass is known:
   
    .. math::

       F_{meas} = m \cdot {a}_z

    Where:
    
    * :math:`m` is the current mass of the platform (kg).
    * :math:`{a}_z` is the mean Z-axis acceleration (:math:`m/s^2`).

4.  **Error Calculation**: 
    The estimated ``force_error`` is computed as the difference between the commanded thrust and the measured force. This value is pushed to a buffer stack.

    .. math::

       F_{error} = F_{cmd} - F_{meas}

5.  **Filtering**: 
    At a frequency defined by ``fz_filtered_error``, the mean value of the stack is computed. Then, a **Low-Pass Filter (LPF)** (defined by ``alpha``) is applied to smooth the estimation.

6.  **Validation**: 
    The result is only accepted if it falls within the [``minimum_error``, ``maximum_error``] range.

Configuration Parameters
------------------------

The behavior can be configured via the ``config/config_default.yaml`` file.

.. list-table:: Configuration Parameters
   :widths: 30 70
   :header-rows: 1

   * - Parameter
     - Description
   * - **controller_node**
     - Name of the controller manager node to update the parameters.
   * - **force_param_name**
     - Name of the force parameter in the controller to be updated.
   * - **mass_param_name**
     - Name of the mass parameter in the controller (used to read the current mass).
   * - **default_mass**
     - Default mass value to use if the parameter server is not available. (kg)
   * - **alpha**
     - Low-pass filter coefficient for the force error estimation. Range [0.0 - 1.0].
   * - **n_samples**
     - Number of samples to average the force error estimation.
   * - **initial_force_error**
     - Initial force error value to start the estimation. (N)
   * - **threshold_time_sync**
     - Time threshold for synchronizing IMU and force command messages. (s)
   * - **fz_filtered_error**
     - Frequency to process the stack and update the internal filtered force error. (Hz)
   * - **fz_update_error**
     - Frequency to update the external controller parameter with the current calculated error. (Hz)
   * - **minimum_error**
     - Minimum force error value to be considered valid. (N)
   * - **maximum_error**
     - Maximum force error value to be considered valid. (N)

Launching the Behavior
----------------------

You can launch this behavior in two modes depending on your needs:

**1. Autostart Mode**
    
Use this mode to launch the node and automatically start the force estimation behavior immediately.

.. code-block:: bash

   ros2 launch as2_behaviors_param_estimation force_estimation_behavior_auto_launch.py

**2. Manual Mode**

Use this mode to launch the node in an idle state. You must manually trigger the start via the ROS 2 Action interface.

.. code-block:: bash

   ros2 launch as2_behaviors_param_estimation force_estimation_behavior_launch.py

**Sending the Goal manually:**

.. code-block:: bash

   ros2 action send_goal /<namespace>/ForceEstimationBehavior as2_msgs/action/ForceEstimation {}

Debugging & Interfaces
----------------------

The behavior publishes internal states to the following topics for debugging purposes:

* ``debug/param_estimation/force_error``: Publishes the raw instantaneous thrust error.
* ``debug/param_estimation/force_filtered``: Publishes the error after applying the low-pass filter.
* ``debug/param_estimation/force_limited_error``: Publishes the force error after clamping it within the min/max validity range.
* ``debug/param_estimation/force_update``: Publishes the actual value sent to the controller parameter server.
