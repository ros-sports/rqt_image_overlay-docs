Creating a Layer 
################

In this tutorial, you will create a layer that draws a `Point`_ that is being published on a topic,
onto an image. 

Create a Package
****************

Create a new empty package in your workspace ``src`` directory.

.. code-block:: console

    ros2 pkg create --build-type ament_cmake geometry_msgs_layers --dependencies rqt_image_overlay_layer --library-name geometry_msgs_layers

.. _Point: https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/Point.msg

