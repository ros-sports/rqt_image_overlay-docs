Timestamp Synchronization
#########################

Timestamp synchronization is the act of synchronizing the layers with the images using
header timestamps.

.. _Background:

Background
**********

RQt Image Overlay listens for images and messages to overlay on separate topics.
Messages on these separate topics arrive at different times.
Due to the sources of delay including:

* Image Processing
* Interprocess communication latency
* Network delays

the difference in time that the messages arrive can cause a noticeable lag.
The lag can be seen in the following video where the red bounding box around the soccer ball
doesn't match the displayed images well:

.. youtube:: BYN1tczU7xc

With this delay, RQt Image Overlay is almost useless for debugging.

Solution
********

A solution to this problem, is to use `std_msgs/msg/Header`_ to timestamp the messages, such that
they match those of the incoming images.

Since messages arrive after the image, we must allow some time for messages to be collected before
composing the overlay image and displaying it. This waiting time is called the waiting window, and
is 0.3 seconds by default.

Note that message_filter's TimeSynchronizer is not used due to its shortcomings:

* Needs to know the msg types at compilation-time as it is templated
* Can only synchronize up to 9 channels
* Reuires all topics to have complete set of matching header timestamps.

If an image is received, and a message for an overlay layer is not received within the waiting
window, the overlay is simply not drawn. If for example, you don't detect a certain object in an
image, you can not send a message on the topic, and the synchronization will work fine.


The following video shows the ball bounding box matches the images perfectly, when 
**timestamp synchronization** is being used.

.. youtube:: 1OHLSWl-JzM

.. note::

  Due to this waiting window, images aren't displayed immediately when they are received.
  They are displayed after the waiting window ends.

.. tip::

  If there are significant delays greater than the waiting window or messages dropping out,
  RQt Image Overlay can't display them correctly.
  This can appear as object recognition algorithms failing to recognise objects.

Dealing with messages with no header
************************************

Although having a header field is recommended, RQt Image Overlay can also display messages that
don't contain a header.
In such cases, the time that the images and messages are received are used to find the nearest
msg to display.
Note however, that this results in the lag behaviour discussed in :ref:`Background` for the
corresponding overlay layer however.


.. _std_msgs/msg/Header: https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/Header.msg)