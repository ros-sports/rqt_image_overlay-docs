Installation
############

.. note::

   Instructions here assume that you have and are in a ROS2 workspace's
   root directory.

Cloning repositories
********************

In your ROS2 workspace, clone the repository and it's dependencies:

.. code-block:: console

   git clone --recursive https://github.com/ijnek/rqt_image_overlay.git src/rqt_image_overlay
   vcs import src < src/rqt_image_overlay/dependencies.repos --recursive

Building
********

To build the package and its dependencies, in the workspace root directory, run:

.. code-block:: console

   colcon build