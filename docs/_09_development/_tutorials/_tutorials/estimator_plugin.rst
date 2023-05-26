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

This tutorial shows how to create your own State Estimator within the AeroStack2 framework.
You may want to remember AeroStack2 :ref:`Architecture <as2_concepts_architecture>` and its :ref:`State Estimator <as2_concepts_state_estimator>`
operation.

This tutorial will be a walktrough of one existing estimators to easily understand its creation.
The one explained will be the mocap estimator plugin. The code used in this tutorial can be 
found in `Github 
<https://github.com/aerostack2/aerostack2/tree/main/as2_state_estimator/as2_state_estimator_plugin_mocap>`_.



.. _development_tutorials_state_estimator_requirements:

------------
Requirements
------------

- ROS 2 Humble
- AeroStack2



.. _development_tutorials_state_estimator_steps:

--------------
Tutorial Steps
--------------



.. _development_tutorials_state_estimator_steps_class:

1. Estimator Plugin Base
========================

Following the plugin structure, all the estimators should inherit from a base class
``as2_state_estimator_plugin_base::StateEstimatorBase``. The base class provides a set of 
virtual methods to override with expected controller functionality. The list of methods is 
presented in the table below:

.. list-table:: List of Virtual Methods
   :header-rows: 1

   * - Virtual method
     - Method description
     - Requires override?
   * - on_setup()
     - Setup the estimator. Declare subscribers to estimation sources.
     - Yes
   * - get_earth_to_map_transform()
     - Set transformation between earth and map.
     - No



.. _development_tutorials_state_estimator_steps_methods:

2. Overriden methods
====================



.. _development_tutorials_state_estimator_steps_methods_setup:

Setup
-----

During the setup, mocap subscriber is created and a static transformation between ``map``
and ``odom`` is published as null into the tf tree. In the mocap callback, the transformation
between ``base_link`` and ``map`` is received from the mation capture system and published 
into the tf tree. Moreover, the speed is also estimated based on the pose information and 
published into the AeroStack2 designed topic (``self_localization/twist``).

.. code-block:: cpp

    void on_setup() override {
        mocap_pose_sub_ = node_ptr_->create_subscription<geometry_msgs::msg::PoseStamped>(
            as2_names::topics::ground_truth::pose, as2_names::topics::ground_truth::qos,
            std::bind(&Plugin::mocap_pose_callback, this, std::placeholders::_1));

        geometry_msgs::msg::TransformStamped map_to_odom =
            as2::tf::getTransformation(get_map_frame(), get_odom_frame(), 0, 0, 0, 0, 0, 0);
        publish_static_transform(map_to_odom);

        has_earth_to_map_ = false;
    }

Transformation between ``earth`` and ``map`` is not implemented in this example.



.. _development_tutorials_state_estimator_steps_export:

3. Exporting the plugin
=======================

Now that we have created our new estimator, we need to export it so that it would be visible
to the State Estimator Node. Plugins are loaded at runtime and if they are not visible, then our
node won't be able to load it. In ROS 2, exporting and loading plugins is handled
by ``pluginlib``.

To achieve that, class ``as2_state_estimator_plugin_mocap::Plugin`` is loaded dynamically from
``as2_state_estimator_plugin_base::StateEstimatorBase`` which is the base class.

1. To export the controller, we need to provide two lines:

.. code-block:: c++

    #include <pluginlib/class_list_macros.hpp>
    PLUGINLIB_EXPORT_CLASS(as2_state_estimator_plugin_mocap::Plugin, as2_state_estimator_plugin_base::StateEstimatorBase)

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
        <class type="mocap::Plugin" base_class_type="as2_state_estimator_plugin_base::StateEstimatorBase">
            <description>State estimator plugin for mocap optitrack.</description>
        </class>
    </library>

3. Next step would be to export plugin using ``CMakeLists.txt`` by using cmake function 
``pluginlib_export_plugin_description_file()``. This function installs plugin description 
file to share directory and sets ament indexes to make it discoverable.

.. code-block:: cmake

    pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

4. Compile and it should be registered. Next, we'll use this plugin.



.. _development_tutorials_state_estimator_steps_launch:

4. Launching
============

¿**TBD**?