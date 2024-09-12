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
   * - Follow Reference
     - Keep the aircraft at given pose reference in the desired frame
     - | Frame id: desired frame of the reference
       | Yaw: desired mode
       | Target reference: desired pose reference in the given frame.
       | Max speed: desired speed movement in each of the axis, in meters per second (m/s)



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
   * - Generate Polynomial Trajectory
     - Convert a list of waypoints to a trajectory
     - | Header: desired frame of the path
       | Yaw: desired mode
       | Path: desired path with a list of poses with id.
       | Max speed: desired speed movement, in meters per second (m/s)