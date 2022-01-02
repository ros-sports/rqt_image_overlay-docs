Usage
#####

..note::

    Make sure you have sourced the workspace in the terminals you work in.

Opening
*******

You have the option of Running RQt Image Overlay as either a:

#. :ref:`Standalone Application`
#. :ref:`RQt Widget`

Standalone Application
======================

Open a new terminal and run the application:

.. code-block:: console

    ros2 run rqt_image_overlay rqt_image_overlay

RQt Widget
==========

Open a new terminal and run RQt:

.. code-block:: console

    rqt

.. warning::

    If this is the first time opening rqt since you've sourced the setup file, run
    ``rqt --force-discover`` instead.

Publishing image
****************

You need a ROS2 node publishing a `sensor_msgs/Image`_ type. In this example, we will use
the `v4l2_camera`_ package to publish images from a webcam. If you have an image being published on a topic already,
you can skip this step.

To install v4l2_camera, in a new terminal run:

.. code-block:: console

    sudo apt install ros-${ROS_DISTRO}-v4l2-camera

.. note::

    ``${ROS_DISTRO}`` gets automatically substituted with the name of your ROS2 distro
    (eg. rolling, galactic, etc.) if you have sourced your ROS2 installation.

To start the v4l2 camera node, run:

.. code-block:: console

    ros2 run v4l2_camera v4l2_camera_node

In a separate terminal, check that the image is being published correctly by running:

.. code-block:: console

    ros2 topic list -t

If you see ``/image_raw [sensor_msgs/msg/Image]`` topic in the list of topics, then your camera node is running correctly.


Showing Image
*************

Go to the window with the RQt Image Overlay that you opened in :ref:`Opening`.

After clicking on the refresh button, open the drop-down, where you should see all topics
publishing `sensor_msgs/Image`_. In this example, ``image_raw`` is the only topic listed.

.. image:: images/image_combo_box.png

Select the topic ``image_raw``, you should see the output of your webcam showing in the
bottom half of your RQt Image Overlay, as below:

.. image:: images/v4l2_image.png

.. _sensor_msgs/Image: http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html
.. _v4l2_camera: https://index.ros.org/r/v4l2_camera/