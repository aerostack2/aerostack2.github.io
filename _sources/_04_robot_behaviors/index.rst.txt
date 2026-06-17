.. _behaviors:

===============
Robot Behaviors
===============

This catalog shows the list of behaviors according to the tasks that they perform.

.. note:: For more information about what a behavior is, please refer to the :ref:`as2_concepts_behaviors` section, or how they interact with other Aerostack2 components at :ref:`as2_concepts_architecture` section.

.. TDB: Reference msgs in parameters column to as2_msgs section.
.. TDB: Add description to request, result and feedback msgs.



.. _behaviors_motion:

----------------
Motion Behaviors
----------------

.. list-table:: Motion Behaviors
   :widths: 50 50 50
   :header-rows: 1

   * - Name
     - Description
     - Parameters
   * - Take Off
     - Change the state of the aircraft from landed to flying. For that, the aircraft must be armed and in offboard mode
     - | Height: desired take off height from the starting point, in meters (m)
       | Speed: desired take off speed movement, in meters per second (m/s)
   * - Land
     - Change the state of the aircraft from flying to landed. Also disarm it
     - Speed: desired landing speed movement, in meters per second (m/s)
   * - Go To
     - Move the aircraft to a given position in the desired frame
     - | Yaw: desired mode
       | Target pose: desired position in the desired frame, in meters (m)
       | Max speed: desired speed movement, in meters per second (m/s)
   * - Follow Path
     - Move the aircraft along a given path in the desired frame
     - | Header: desired frame of the path
       | Yaw: desired mode
       | Path: desired path with a list of poses with id.
       | Max speed: desired speed movement, in meters per second (m/s)
   * - :ref:`behaviors_follow_reference_page`
     - Keep the aircraft at given pose reference in the desired frame through a runtime-selectable plugin (position or trajectory).
     - | Yaw: desired yaw mode.
       | Target pose: desired reference point in the given frame (m).
       | Max speed: desired speed per axis (m/s).

.. toctree::
   :hidden:
   :caption: Motion Behaviors
   :maxdepth: 1

   follow_reference/index



.. _behaviors_perception:

--------------------
Perception Behaviors
--------------------

.. list-table:: Perception Behaviors
   :widths: 50 50 50
   :header-rows: 1

   * - Name
     - Description
     - Parameters
   * - Detect ArUco Markers
     - Detect ArUco Markers in a image and publish the result.
     - Target ids: list of id of the markers to detect



.. _behaviors_trajectory_generation:

-------------------------------
Trajectory Generation Behaviors
-------------------------------

.. list-table:: Trajectory Generation Behaviors
   :widths: 50 50 50
   :header-rows: 1

   * - Name
     - Description
     - Parameters
   * - :ref:`behaviors_trajectory_generation_page`
     - Convert a list of waypoints into a smooth trajectory through a runtime-selectable plugin (dynamic_mav, mav, jerk_limited or gcopter).
     - | Yaw: desired yaw mode.
       | Path: list of waypoints with id.
       | Max speed: desired speed along the path (m/s).
       | Start on paused: if true, generate the trajectory and stay paused until the client resumes.

.. toctree::
   :hidden:
   :caption: Trajectory Generation Behaviors
   :maxdepth: 1

   trajectory_generation/index

.. _behaviors_param_estimation:

-------------------------------
Parameter Estimation Behaviors
-------------------------------

.. list-table:: Parameter Estimation Behaviors
   :widths: 50 50 50
   :header-rows: 1

   * - Name
     - Description
     - Parameters
   * - :ref:`behaviors_mass_estimation`
     - Estimate the mass of the platform using the aceleracion in the z axis and the thrust command to the motors.
     - 
   * - :ref:`behaviors_force_estimation`
     - Estimate the difference between the commanded thrust and the thrust being executed by the platform.
     -

.. toctree::
   :hidden:
   :caption: Parameter Estimation Behaviors
   :maxdepth: 1

   param_estimation/mass_estimation/index
   param_estimation/force_estimation/index

.. _behaviors_swarm_flocking:

-------------------------------
Swarm Flocking Behaviors
-------------------------------

.. list-table:: Swarm Flocking Behaviors
   :widths: 50 50 50
   :header-rows: 1

   * - Name
     - Description
     - Parameters
   * - :ref:`swarm_flocking`
     - Generates a dynamic Swarm structure around a centroid in order to move the drones in formation.
     - | Virtual Centroid: Desired pose offset of the virtual centroid with respect to the frame it must follow.
       | Swarm Formation: List of desired poses, including the ID of each drone in the swarm, with respect to the virtual centroid.
       | Drone Namespace: List of the drone namespaces in the swarm. It must be in the same order as the swarm formation list.

.. toctree::
   :hidden:
   :caption: Swarm Flocking Behaviors
   :maxdepth: 1

   swarm_flocking/index

.. _behaviors_payload:

-------------------------------
Payload Behaviors
-------------------------------

.. list-table:: Payload Behaviors
   :widths: 50 50 50
   :header-rows: 1

   * - Name
     - Description
     - Parameters
   * - :ref:`gripper`
     - Control a gripper to pick up and release objects.
     - Gripper request: It is a bool msg where true means to start the closing action and false means to start the opening action.

.. toctree::
   :hidden:
   :caption: Payload Behaviors
   :maxdepth: 1

   payload/gripper/index