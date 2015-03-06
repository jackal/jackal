Simulating Jackal
=================

Whether you actually have a Jackal robot or not, the Jackal simulator is a great way to get started with ROS
robot development. In this tutorial, we will go through the basics of starting Gazebo and Rviz and how to drive
your Jackal around.


Installation
------------

To get started with the Jackal simulation, make sure you have a :roswiki:`working ROS installation <ROS/Installation>`
set up on your Ubuntu desktop, and install the Jackal-specific metapackages for desktop and simulation:

.. parsed-literal::

    sudo apt-get install ros-|ros_distro|-jackal-simulator ros-|ros_distro|-jackal-desktop


Launch Gazebo
-------------

We will be using two tools for our Jackal simulation, the first being Gazebo. This is where we get a realistic
simulation of our robot, including wheel slippage, skidding, and inertia. It is also possible to insert objects
and load maps into Gazebo to simulate sensor data, which we will see in a following tutorial. For now begin by
launching gazebo:

.. code-block:: bash

    roslaunch jackal_gazebo jackal_world.launch

attachment:Gazebo.png


Launch rviz
-----------

The other tool we will be using is rviz. This allows us to see sensor data from a robot, and give it commands.
Unlike Gazebo, rviz is not a simulation, only a visualizer. You can using the following launch invocation to
start rviz with a pre-cooked configuration suitable for visualizing standard Jackal:

.. code-block:: bash

    roslaunch jackal_viz view_robot.launch

attachment:RViz.png


Driving Jackal
--------------

Interactive markers are the simplest way to drive Jackal. To do this, select the Interact tool from the
top toolbar in rviz. You should see red arrows and a blue circle appear around the Jackal model.

Drag the red arrows in Rviz to move in the linear x and the blue circle to move in the angular z. Rviz shows you
Jackal moving relative to its odometric frame, but it is also moving relative to the simulated world supplied by
Gazebo. If you click over to the Gazebo window, you will see Jackal moving within its simulated world. Or, if you
drive real Jackal using this method, it will have moved in the real world.

Once you start your own development, have your nodes send ``geometry_msgs/Twist`` commands to the ``cmd_vel``
topic to drive Jackal, either real or simulated. This is the standard ROS interface to differential-drive and
holonomic ground vehicles.

