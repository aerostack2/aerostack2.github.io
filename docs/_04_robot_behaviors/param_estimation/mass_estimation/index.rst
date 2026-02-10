.. _behaviors_mass_estimation:

Mass Estimation Behavior
========================

The ``mass_estimation_behavior`` is a core component of the 
``as2_behaviors_param_estimation`` package in Aerostack2.


This behavior estimates the **mass of the aerial platform** in real-time during flight. It achieves this by fusing sensor data from the onboard IMU with the thrust commanded by the controller.

Working Principle
-----------------

The behavior operates by subscribing to two main data sources:

* **IMU Data** (``/sensor_measurements/imu``): To obtain linear acceleration, specifically along the Z-axis.
* **Thrust Command** (``/actuator_command/thrust``): To obtain the current force applied by the motors.

Estimation Algorithm
^^^^^^^^^^^^^^^^^^^^

The estimation process is executed iteratively:

1.  **Data Synchronization**: When a new thrust command is received, the behavior computes the mean value of the Z-axis acceleration from the IMU buffer.
2.  **Instantaneous Estimation**: A raw mass value is computed using Newton's second law:

    .. math::

       m_{inst} = \frac{T}{{a_z}}

    Where:
    
    * :math:`T` is the commanded thrust (N).
    * :math:`{a_z}` is the mean Z-axis acceleration :math:`(m/ s^2)`.

3.  **Validation & Filtering**:
    
    The raw estimation goes through a validation pipeline before being accepted:
    
    * **Thrust Check**: The calculation is only performed if the thrust :math:`T` is greater than ``thrust_threshold``.
    * **Range Check**: The result is discarded if it falls outside the [``minimum_mass``, ``maximum_mass``] range.
    * **Change Detection**: The value is added to the processing stack only if the difference compared to the previous estimate exceeds ``mass_threshold``.

4.  **Final Output**:
    
    The final ``estimated_mass`` is derived by taking the average of the last ``n_samples`` stored in the stack and applying a **Low-Pass Filter** defined by the coefficient ``alpha``.

Configuration Parameters
------------------------

The behavior can be configured via the ``config/config_default.yaml`` file.

.. list-table:: Configuration Parameters
   :widths: 30 70
   :header-rows: 1

   * - Parameter
     - Description
   * - **mass_threshold**
     - Minimum change required between the new estimate and the last computed mass to update the stack. (kg)
   * - **thrust_threshold**
     - Minimum thrust value required to perform the mass computation. Prevents division by zero or noise at low thrust. (N)
   * - **alpha**
     - Coefficient for the Low-Pass Filter (LPF). Range: [0.0 - 1.0]. Lower values result in smoother but slower updates.
   * - **n_samples**
     - Size of the buffer (number of samples) used to average the mass estimation before filtering.
   * - **minimum_mass**
     - Lower bound for a valid mass estimation. Values below this are discarded. (kg)
   * - **maximum_mass**
     - Upper bound for a valid mass estimation. Values above this are discarded. (kg)
   * - **mass_fz**
     - Frequency at which the estimated mass parameter is updated in the controller node. (Hz)
   * - **controller_node**
     - Name of the controller manager node that will receive the mass update.
   * - **mass_param_name**
     - Name of the specific parameter within the controller node to be updated.
   * - **initial_mass**
     - Fallback mass value used if the controller node is unavailable or before the first estimation. (kg)

Launching the Behavior
----------------------

You can launch this behavior in two modes depending on your needs:

**1. Autostart Mode**
    
Use this mode to launch the node and automatically start the mass estimation behavior immediately.

.. code-block:: bash

   ros2 launch as2_behaviors_param_estimation mass_estimation_behavior_auto_launch.py

**2. Manual Mode**

Use this mode to launch the node in an idle state. You must manually trigger the start via the ROS 2 Action interface.

.. code-block:: bash

   ros2 launch as2_behaviors_param_estimation mass_estimation_behavior_launch.py

**Sending the Goal manually:**

.. code-block:: bash

   ros2 action send_goal /<namespace>/MassEstimationBehavior as2_msgs/action/MassEstimation {}

Debugging & Interfaces
----------------------

The behavior publishes internal states to the following topics for debugging purposes:

* ``debug/param_estimation/mass_estimation``: Publishes all raw estimated mass values (pre-filter).
* ``debug/param_estimation/mass_filtered``: Publishes the final mass value after averaging and low-pass filtering.
* ``debug/param_estimation/mass_update``: Publishes the actual value sent to the controller parameter server.