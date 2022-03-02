Installation
############

Follow one of the two installation methods below:

* :ref:`Binary Installation` - For most ordinary users
* :ref:`Source Installation` - For maintainers or cutting-edge users of RQt Image Overlay

.. warning::

   RQt Image Overlay is available for **ROS2 Galactic onwards**. It won't compile on older
   ROS2 distros and all ROS1 distros.

.. tabs::

   .. tab:: Binary Installation

      Source ROS2, and then run:

      .. code-block:: console

         sudo apt install ros-${ROS_DISTRO}-rqt-image-overlay

   .. tab:: Source Installation

      In your ROS2 workspace, clone the repository and it's dependencies:

      .. code-block:: console

         git clone --recursive https://github.com/ros-sports/rqt_image_overlay.git src/rqt_image_overlay
         vcs import src < src/rqt_image_overlay/dependencies.repos --recursive

      In the same directory, build the package and its dependencies by running:

      .. code-block:: console

         colcon build
