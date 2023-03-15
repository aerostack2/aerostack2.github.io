.. _as2_concepts_state_estimator:

===============
State Estimator
===============

State Estimator component in Aerostack2 is the one in charge get the pose and twist of the drone.
For that, is use the information provided by the sensors and the information provided by the platform.

Also, it is in charge of generate the `ROS2 Tf2 <https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Tf2-Main.html>`__ tree, so the rest of the components can consult it.
For that, it stablish a common `earth` frame for all drones and set the `map` and `odom` frames of each one.