# encoders
Arduino code for measuring wheel RPM with magnetic encoders.

## Dependencies

- rosserial (http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)
- rover (https://github.com/NovaRoverTeam/rover)

## Publications
Topic:       **encoders**<br />
Msg type:    rover/RPM<br />
Description: Publishes RPM values for wheels, based on magnetic encoder readings.

## Issues

When compiling the Arduino code, there may be an issue finding the rover/RPM message, 
which is required for the rosserial integration. 

Check that ``echo $PYTHONPATH`` gives you the following output: <br/>
``/home/<your username>/<your workspace name>/devel/lib/python2.7/dist-packages:/opt/ros/kinetic/lib/python2.7/dist-packages``
<br/><br/>
If you don't see the path for your catkin workspace (default catkin_ws), make sure to add this with:<br/>
``export PYTHONPATH=$PYTHONPATH:/home/<your username>/<your workspace name>/devel/lib/python2.7/dist-packages``
<br/><br/>

Also make sure that ``echo $ROS_PACKAGE_PATH`` gives you the output: <br/>
``/home/<your username>/<your workspace name>/src:/opt/ros/kinetic/share``
<br/><br/>
If you don't see the path for your catkin workspace (default catkin_ws), make sure to add this with:<br/>
``export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/<your username>/<your workspace name>/src``
<br/><br/>

Once you've done this, go to your catkin workspace and run ``catkin_make``, then rebuild the *ros_lib* Arduino library, as 
per the instructions for setting up *rosserial* (seen in the link above, under Dependencies).
