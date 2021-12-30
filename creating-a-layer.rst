Creating a Layer 
################

In this tutorial, you will create a layer that draws a `Point`_ that is being published on a topic,
onto an image. 

Create a Package
****************

Create a new empty package in your workspace ``src`` directory:

.. code-block:: console

    ros2 pkg create --build-type ament_cmake geometry_msgs_layers --dependencies rqt_image_overlay_layer geometry_msgs --library-name point

The command:

* Created a package called geometry_msgs_layers
* Added code to CMakeLists.txt of the new package, to generate a library called *point*
* Generated files ``include/geometry_msgs_layers/point.hpp`` and ``src/point.cpp``

Write point.hpp
***************

Open the generated ``include/geometry_msgs_layers/point.hpp`` file in your favorite editor,
and paste the following instead of it:

.. code-block:: cpp

    #ifndef GEOMETRY_MSGS_LAYERS__POINT_HPP_
    #define GEOMETRY_MSGS_LAYERS__POINT_HPP_

    #include "geometry_msgs_layers/visibility_control.h"
    #include "rqt_image_overlay_layer/plugin.hpp"
    #include "geometry_msgs/msg/point.hpp"

    namespace geometry_msgs_layers
    {

    class Point : public rqt_image_overlay_layer::Plugin<geometry_msgs::msg::Point>
    {
    protected:
    void overlay(
        QImage & layer,
        const geometry_msgs::msg::Point & msg) override;
    };

    }  // namespace geometry_msgs_layers

    #endif  // GEOMETRY_MSGS_LAYERS__POINT_HPP_

Your Point class must inherit the class rqt_image_overlay_layer::Plugin<T> where ``T`` is the msg
type you are displaying in the layer (ie. ``geometry_msgs::msg::Point``).

Write point.cpp
***************


.. _Point: https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/Point.msg

