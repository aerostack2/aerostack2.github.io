.. _development_tutorials_state_estimator:

====================================
Writing a New State Estimator Plugin
====================================

.. contents:: Table of Contents
   :depth: 1
   :local:



.. _development_tutorials_state_estimator_overview:

--------
Overview
--------

This tutorial shows how to create your own State Estimator plugin within the Aerostack2
framework. You may want to review the Aerostack2
:ref:`Architecture <as2_concepts_architecture>` and its
:ref:`State Estimator <as2_concepts_state_estimator>` first.

A plugin turns the measurements available on a platform into the robot state. It does not
publish anything itself: it **reports** the parts of the state it has computed, and the
``state_estimator`` node checks each report against the plugin's declared authority and
publishes what is accepted. The plugins that ship with Aerostack2 live in
``as2_state_estimator/plugins`` and are worth reading alongside this page.



.. _development_tutorials_state_estimator_requirements:

------------
Requirements
------------

- ROS 2 Humble
- Aerostack2



.. _development_tutorials_state_estimator_steps:

--------------
Tutorial Steps
--------------



.. _development_tutorials_state_estimator_steps_layout:

1. Package Layout
=================

A plugin is a subdirectory of ``as2_state_estimator/plugins`` containing four things:

.. list-table:: Files of a plugin
   :widths: 40 60
   :header-rows: 1

   * - Path
     - Purpose
   * - ``include/<plugin>/<plugin>.hpp``
     - The plugin class, deriving from the base class
   * - ``src/<plugin>.cpp``
     - Its implementation, ending with the pluginlib export macro
   * - ``config/plugin_default.yaml``
     - Default values for **every** parameter the plugin reads
   * - ``CMakeLists.txt``
     - Builds the plugin as a shared library

The name of the directory is the name used in ``plugin_name``, and it is also the name of
the YAML block the plugin reads its parameters from.



.. _development_tutorials_state_estimator_steps_class:

2. The Base Class
=================

Every plugin inherits from ``as2_state_estimator_plugin_base::StateEstimatorBase``. The base
class hands over three members and asks for two methods:

.. list-table:: Members provided by the base class
   :widths: 35 65
   :header-rows: 1

   * - Member
     - Description
   * - ``node_ptr_``
     - The ``state_estimator`` node. Use it to declare parameters, create subscriptions and
       log
   * - ``tf_handler_``
     - A TF handler whose lookups are filtered, see step 6
   * - ``state_estimator_interface_``
     - The interface the plugin reports state through, and reads the resolved frame names
       from

.. list-table:: Methods to override
   :widths: 35 50 15
   :header-rows: 1

   * - Virtual method
     - Description
     - Required
   * - ``onSetup()``
     - Read the parameters and create the subscriptions. Called once, after the members
       above have been filled in
     - Yes
   * - ``getTransformationTypesAvailable()``
     - Declare which parts of the transform chain this plugin is able to produce
     - Yes

.. code-block:: cpp

    #include <pluginlib/class_list_macros.hpp>
    #include "as2_state_estimator/plugin_base.hpp"

    namespace my_plugin
    {

    class Plugin : public as2_state_estimator_plugin_base::StateEstimatorBase
    {
    public:
      void onSetup() override
      {
        auto topic = getParameter<std::string>(node_ptr_, "my_plugin.some_topic");

        sub_ = node_ptr_->create_subscription<geometry_msgs::msg::PoseStamped>(
          topic, rclcpp::SensorDataQoS(),
          std::bind(&Plugin::poseCallback, this, std::placeholders::_1));
      }

      std::vector<as2_state_estimator::TransformInformatonType>
      getTransformationTypesAvailable() const override
      {
        return {as2_state_estimator::TransformInformatonType::ODOM_TO_BASE,
                as2_state_estimator::TransformInformatonType::TWIST_IN_BASE};
      }

    private:
      void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
      rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
    };

    }  // namespace my_plugin



.. _development_tutorials_state_estimator_steps_authority:

3. Declaring Authority
======================

``getTransformationTypesAvailable()`` is how a plugin says what it is allowed to write. The
node records the answer when the plugin is loaded, and consults it on every report.

.. list-table:: Transform types
   :widths: 30 70
   :header-rows: 1

   * - Type
     - Claiming it means the plugin may set
   * - ``EARTH_TO_MAP``
     - The position of this drone's ``map`` frame inside the global ``earth`` frame
   * - ``MAP_TO_ODOM``
     - The correction coming from absolute measurements
   * - ``ODOM_TO_BASE``
     - The smooth, dead reckoned pose
   * - ``TWIST_IN_BASE``
     - The body velocity

A report for a type the plugin did not claim is logged as an error and dropped. Claim only
what the plugin genuinely produces: that is what allows two plugins to be loaded together,
each responsible for a different link of the chain.



.. _development_tutorials_state_estimator_steps_reporting:

4. Reporting State
==================

From a callback, hand the result to the node through ``state_estimator_interface_``:

.. code-block:: cpp

    void Plugin::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
      geometry_msgs::msg::PoseWithCovariance pose;
      pose.pose = msg->pose;

      state_estimator_interface_->setOdomToBaseLinkPose(pose, msg->header.stamp);
    }

.. list-table:: Reporting methods
   :widths: 45 55
   :header-rows: 1

   * - Method
     - Reports
   * - ``setEarthToMap(pose, stamp, is_static)``
     - ``earth`` to ``map``. Pass ``is_static`` as ``true`` to publish it on ``/tf_static``
   * - ``setMapToOdomPose(pose, stamp, is_static)``
     - ``map`` to ``odom``
   * - ``setOdomToBaseLinkPose(pose, stamp)``
     - ``odom`` to ``base_link``
   * - ``setTwistInBaseFrame(twist, stamp)``
     - The twist in the ``base_link`` frame

Each of the pose methods accepts either a ``geometry_msgs::msg::PoseWithCovariance`` or a
``tf2::Transform``. The same interface also exposes the resolved frame names, through
``getEarthFrame()``, ``getMapFrame()``, ``getOdomFrame()`` and ``getBaseFrame()``, and the
transforms currently held by the node, through ``getEarthToMapTransform()`` and its
siblings.

.. note::

   ``self_localization/pose`` is published from the twist path. A plugin that never reports
   a twist produces a transform tree but no pose topic.



.. _development_tutorials_state_estimator_steps_parameters:

5. Parameters
=============

Parameters of a plugin live in a YAML block named after the plugin, and are read with the
``getParameter<T>()`` helper of the base class, which logs every value it reads:

.. code-block:: cpp

    auto topic = getParameter<std::string>(node_ptr_, "my_plugin.some_topic");
    auto enabled = getParameter<bool>(node_ptr_, "my_plugin.some_flag");

.. code-block:: yaml

   /**:
     ros__parameters:
       my_plugin:
         some_topic: "sensor_measurements/pose"
         some_flag: false

.. warning::

   ``getParameter<T>()`` declares the parameter without a fallback value, so a parameter
   missing from the YAML stops the node. Every key the plugin reads has to exist in
   ``config/plugin_default.yaml``, including keys inside commented-out example blocks that
   users are expected to uncomment.



.. _development_tutorials_state_estimator_steps_tf:

6. Reading Transforms
=====================

Use ``tf_handler_`` rather than creating your own ``tf2_ros::TransformListener``:

.. code-block:: cpp

    auto tf = tf_handler_->getTransform(getMapFrame(), sensor_frame, msg->header.stamp);

``tf_handler_`` goes through a filter that removes the three transforms the estimator itself
publishes. Without it, a plugin can read back the output it has just produced and treat it
as an external measurement, which closes a feedback loop that is difficult to diagnose.



.. _development_tutorials_state_estimator_steps_export:

7. Exporting the Plugin
=======================

Plugins are loaded at run time, so the plugin has to be visible to ``pluginlib``.

1. Export the class at the end of the source file:

.. code-block:: cpp

    #include <pluginlib/class_list_macros.hpp>
    PLUGINLIB_EXPORT_CLASS(my_plugin::Plugin, as2_state_estimator_plugin_base::StateEstimatorBase)

2. Declare it in ``as2_state_estimator/plugins.xml``:

.. code-block:: xml

    <library path="my_plugin">
      <class type="my_plugin::Plugin" base_class_type="as2_state_estimator_plugin_base::StateEstimatorBase">
        <description>State estimator plugin for my sensor.</description>
      </class>
    </library>

3. Add the directory name to ``PLUGIN_LIST`` in ``as2_state_estimator/CMakeLists.txt``. That
   list drives the build of the plugin and the installation of its ``config`` directory:

.. code-block:: cmake

    set(PLUGIN_LIST
      raw_odometry
      ground_truth
      simple_ekf
      my_plugin
    )

The package already calls ``pluginlib_export_plugin_description_file()`` for
``plugins.xml``, so nothing more is needed there.

4. Build the workspace:

.. code-block:: bash

    colcon build --packages-select as2_state_estimator



.. _development_tutorials_state_estimator_steps_launch:

8. Launching
============

Select the plugin by name, either from the configuration file or on the command line:

.. code-block:: yaml

   /**:
     ros__parameters:
       plugin_name: "my_plugin"
       my_plugin:
         some_topic: "sensor_measurements/pose"
         some_flag: false

.. code-block:: bash

    ros2 launch as2_state_estimator state_estimator_launch.py \
        namespace:=drone0 \
        config_file:=path/to/config.yaml

The launch file merges ``config/plugin_default.yaml`` of the selected plugin underneath the
user configuration, so a user only has to list what differs from the defaults.

To confirm the plugin has been loaded, check the startup log: it prints the name of every
plugin it loads followed by every parameter that plugin read. The transform tree should
then contain the links the plugin claims:

.. code-block:: bash

    ros2 run tf2_tools view_frames
    ros2 topic echo /drone0/self_localization/pose
