.. _behaviors_object_perception:

Object Perception Behavior
==========================

The ``object_perception_behavior`` is a core component of the
``as2_behaviors_object_perception`` package in Aerostack2.

This behavior provides a **plugin-based perception pipeline** that processes
camera images and publishes the detected objects (class, image location and,
when possible, their 3D pose). It is designed so that different detection
algorithms can be plugged in without changing the behavior itself, and so that
several detection stages can be chained together to build more complex
perception pipelines.

Working Principle
-----------------

The behavior centralizes the image acquisition and preprocessing, and delegates
the actual detection to one or more plugins:

* **Behavior Node** (``PerceptionBehavior``): Manages the ROS 2 action server,
  acquires the camera images, preprocesses them and runs the detection pipeline.
* **Plugin Base Interface** (``detection_plugin_base::DetectionBase``): Defines
  the common interface that every detection plugin must implement. Plugins
  receive an already decompressed (and optionally rectified) ``cv::Mat`` frame
  and output a list of detections.
* **Detection Plugins**: Specific detection implementations (e.g. ArUco marker
  detection).

Image Acquisition
^^^^^^^^^^^^^^^^^

The behavior can obtain images from two different sources, selected with the
``use_embedded_camera`` parameter:

* **Embedded camera driver** (``use_embedded_camera: true``): The behavior loads
  the ``as2_usb_camera_interface`` driver internally and reads frames directly
  from a USB/CSI camera device. A camera calibration file is used to provide the
  camera intrinsics.
* **Topic mode** (``use_embedded_camera: false``): The behavior subscribes to an
  external image topic (``camera_image_topic``) and, optionally, to a camera
  info topic (``camera_info_topic``). When no camera info is provided, detection
  still runs but 3D pose estimation is not available.

In both cases the behavior decompresses the image and, if
``enable_rectification`` is set, rectifies it using the camera intrinsics before
handing it to the plugins.

Detection Pipeline
^^^^^^^^^^^^^^^^^^

The detection is organized as a **pipeline of stages**, configured in the
``pipeline`` section of the configuration file. Each stage loads one detection
plugin and defines where its input comes from and where its output is published.
A stage can take its input from:

* ``image``: the live preprocessed camera frame.
* ``internal``: the output of a previous stage in the same pipeline (either the
  immediately previous stage, or an explicitly named ``input_stage``).
* ``external``: an ``ObjectPerceptionArray`` received on an external topic
  (``input_topic``).

This allows building pipelines where, for example, one stage detects objects and
a later stage tracks or filters those detections. The output of the last stage
is returned through the action feedback and result.

Available Plugins
^^^^^^^^^^^^^^^^^

The package currently includes one detection plugin:

1. **ArUco Marker Detector** (``aruco::Plugin``)

   Detects ArUco markers in the image and, when the camera intrinsics are known,
   estimates each marker's 6-DoF pose in the camera frame. For every detected
   marker it reports the four corners as keypoints, an axis-aligned bounding box
   and (optionally) the marker pose.

   * Configuration: ``plugins/aruco/config/plugin_default_config.yaml``

Action Interface
----------------

The behavior exposes a ROS 2 action server using the
``as2_msgs/action/DetectObjects`` action:

.. list-table:: DetectObjects action
   :widths: 20 25 55
   :header-rows: 1

   * - Field
     - Part
     - Description
   * - **threshold**
     - Goal
     - Minimum confidence score in the range [0.0, 1.0] required for a detection
       to be accepted.
   * - **target_classes**
     - Goal
     - List of class ids to detect (e.g. ArUco marker ids). If empty, every
       detected object is reported.
   * - **perceptions**
     - Feedback
     - Current detections produced by the pipeline
       (``as2_msgs/ObjectPerceptionArray``).
   * - **perceptions**
     - Result
     - Final detections produced by the pipeline
       (``as2_msgs/ObjectPerceptionArray``).

When the behavior is configured as ``persistent``, it keeps running and
reporting detections through the feedback until it is explicitly cancelled.

Output Message
^^^^^^^^^^^^^^

Each detection is described by an ``as2_msgs/ObjectPerception`` message:

.. list-table:: ObjectPerception
   :widths: 30 70
   :header-rows: 1

   * - Field
     - Description
   * - **header**
     - Frame of the detection.
   * - **hypothesis**
     - Detected class/id, confidence score and, if available, the 3D pose.
   * - **bbox**
     - Bounding box around the detected object in the image (pixels).
   * - **keypoints**
     - Detected keypoints of the object (e.g. the marker corners).
   * - **keypoint_names**
     - Name of each detected keypoint (e.g. ``top_left``).
   * - **keypoint_scores**
     - Confidence of each detected keypoint.
   * - **pose_valid**
     - ``true`` when ``hypothesis.pose`` holds a valid 3D pose, ``false`` for 2D
       only detections.

Configuration Parameters
------------------------

The behavior can be configured via the ``config/config.yaml`` file.

.. list-table:: Behavior Parameters
   :widths: 30 70
   :header-rows: 1

   * - Parameter
     - Description
   * - **persistent**
     - If ``true``, the behavior keeps running and publishing detections until it
       is cancelled.
   * - **run_frequency**
     - Frequency at which the detection pipeline is executed. (Hz)
   * - **use_embedded_camera**
     - If ``true``, use the internal USB camera driver. If ``false``, read images
       from a topic.
   * - **camera_image_topic**
     - Image topic to subscribe to when ``use_embedded_camera`` is ``false``.
   * - **camera_info_topic**
     - Camera info topic with the intrinsics. Optional; required for 3D pose
       estimation in topic mode.
   * - **enable_rectification**
     - If ``true``, the image is rectified using the camera intrinsics before
       detection.

The pipeline is defined under the ``pipeline`` key:

.. list-table:: Pipeline Stage Parameters
   :widths: 30 70
   :header-rows: 1

   * - Parameter
     - Description
   * - **stages**
     - Ordered list of stage names that make up the pipeline.
   * - **plugin**
     - Name of the detection plugin loaded by the stage.
   * - **input_source**
     - Source of the stage input: ``image``, ``internal`` or ``external``.
   * - **input_stage**
     - Name of the stage to take the input from, when ``input_source`` is
       ``internal``.
   * - **input_topic**
     - Topic to read the input detections from, when ``input_source`` is
       ``external``.
   * - **publish_output**
     - If ``true``, the stage detections are published.
   * - **output_topic**
     - Topic where the stage detections (``ObjectPerceptionArray``) are
       published.
   * - **enable_debug**
     - If ``true``, the stage publishes debug information.
   * - **debug_poses_topic**
     - Topic where the valid 3D poses are published as a ``PoseArray`` for
       visualization (e.g. RViz).

**ArUco Plugin Parameters**

Configured via ``plugins/aruco/config/plugin_default_config.yaml``.

.. list-table:: ArUco Plugin Parameters
   :widths: 30 70
   :header-rows: 1

   * - Parameter
     - Description
   * - **tag_dict**
     - ArUco dictionary used for detection (e.g. ``6x6_250``).
   * - **marker_size**
     - Physical size of the marker side, in meters (m). Used for pose estimation.
   * - **estimate_pose**
     - If ``true``, estimate the 6-DoF pose of each marker (requires camera
       intrinsics).

Launching the Behavior
----------------------

You can launch this behavior selecting the detection plugin and the image
source:

**1. Topic mode (images from a topic)**

.. code-block:: bash

   ros2 launch as2_behaviors_object_perception detect_behavior_launch.py \
       namespace:=<drone_namespace> \
       plugin_name:=<plugin_config_file> \
       use_embedded_camera:=false

**2. Embedded camera mode**

.. code-block:: bash

   ros2 launch as2_behaviors_object_perception detect_behavior_launch.py \
       namespace:=<drone_namespace> \
       plugin_name:=<plugin_config_file> \
       use_embedded_camera:=true \
       camera_interface_file:=<camera_interface_file> \
       calibration_file:=<camera_calibration_file>

**Sending the Goal manually:**

.. code-block:: bash

   ros2 action send_goal /<namespace>/PerceptionBehavior as2_msgs/action/DetectObjects \
       "{threshold: 0.0, target_classes: []}"

To detect only specific ArUco markers, list their ids in ``target_classes``,
e.g. ``target_classes: ['0', '5']``.

Debugging & Interfaces
----------------------

The behavior and the ArUco plugin publish the following topics for debugging and
visualization purposes:

* ``detections_data``: Detections produced by the pipeline
  (``as2_msgs/ObjectPerceptionArray``). The exact topic name is set per stage
  with ``output_topic``.
* ``debug/detection_poses``: Valid 3D poses of the detections published as a
  ``geometry_msgs/PoseArray`` (configured with ``debug_poses_topic``).
* ``debug/detection_markers``: ArUco markers published as a
  ``visualization_msgs/MarkerArray`` (cube + id text) for visualization in RViz.
