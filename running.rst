Running
#######

You have the option of Running RQt Image Overlay as:

#. a standalone application
#. a widget in RQt

Standalone App
**************

Open a new terminal, source your setup file, and run the application:

.. code-block:: console

    . install/local_setup.bash
    ros2 run rqt_image_overlay rqt_image_overlay

RQt Widget
**********

Open a new terminal, source your setup file, and run RQt:

.. code-block:: console

    . install/local_setup.bash
    rqt

.. warning::

    If this is the first time opening rqt since you've sourced the setup file, run
    ``rqt --force-discover`` instead.