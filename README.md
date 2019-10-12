![Clank](/Clank.png?raw=true style=centerme "Clank Robot")

Please refer to the following link to see the videos:
    
    https://www.youtube.com/playlist?list=PLaifSaoo3KyS3FFktKaQyA13W0rImoZuD 

Clank has been 3D printed and has the following electronics:

18 Dynamixel AX-12 for the body motion
2 Dynamixel XL-320 for head degrees of freedom

Mini IMU v5 Pololu
Raspberry Pi 3bplus with Rpi Cam
usb2ax peripheral
OpenCM 9.04b to interface I2C IMU to Serial comm

STEPS

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

Thanks to dxydas ros-bioloid, these packages are an extension of his work !
