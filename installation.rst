Installation
############

Follow one of the two installation methods below.

Binary Installation
*******************

The binary installation is currently only available for ROS2 Rolling. Follow the source
installation if you use ROS2 Foxy or Galactic.

.. code-block:: console

   sudo apt install ros-rolling-rqt-image-overlay

Source Installation
*******************

.. note::

   Instructions here assume that you have and are in a ROS2 workspace's
   root directory.

.. warning::

   This package targets **ROS2 Galactic onwards**. It won't compile on all ROS1
   and older ROS2 distros.

Cloning repositories
====================

In your ROS2 workspace, clone the repository and it's dependencies:

.. code-block:: console

   git clone --recursive https://github.com/ros-sports/rqt_image_overlay.git src/rqt_image_overlay
   vcs import src < src/rqt_image_overlay/dependencies.repos --recursive

Building from source
====================

To build the package and its dependencies, in the workspace root directory, run:

.. code-block:: console

   colcon build