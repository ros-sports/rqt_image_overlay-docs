Installation
############

Follow one of the two installation methods below:

* Binary Installation - For most ordinary users
* Source Installation - For maintainers or cutting-edge users of RQt Image Overlay

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

         git clone https://github.com/ros-sports/rqt_image_overlay.git src/rqt_image_overlay --branch ${ROS_DISTRO}
         rosdep install --from-paths src

      In the same directory, build the package and its dependencies by running:

      .. code-block:: console

         colcon build
