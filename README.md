FIRST STEPS

* Ensure all the motors are connected to the robot.

* Ensure to have conection with the Rpi, via Ethernet or Wifi, to be able to listen to topics

Laptop side:

* export ROS_IP=192.168.1.X in bashrc

* Launch:  

  roscore

  roslaunch bioloid_master bioloid_pubs.launch

  roslaunch bioloid_master bioloid_controllers.launch

* Rpi has a service called usb2ax service that launches everything:


* For the gesture control, launch in laptop:

  roslaunch bioloid_moveit_config bioloid_planning_execution.launch

  rosrun bioloid_planning plannin_manager.py

