#Orden de lanzamiento de las cosas

ANTES DE LANZAR NADA

* Asegurarse de que TODOS los servomotores están conectados (al menos los AX, los XL no es necesario), sino el control hardware
les mueve alatoriamente y puede romperse el robot

* Asegurarse de tener conectado a través de ethernet el robot, cuya IP es la 11, y el pc tiene que tener la 10. En caso de 
usar wifi cambiar el network interfaces del robot y poner IP fija del portátil a la 11. Se recomienda el uso de ethernet
sobre todo si se va a hacer seguimiento de caras

En el laptop:

* En todos los terminales export ROS_IP=192.168.1.10 con la dirección de hostname -I

* Lanzar, en distintos terminales:  

  roscore

  roslaunch bioloid_master bioloid_pubs.launch

  roslaunch bioloid_master bioloid_controllers.launch


* Ahora en la rpi, en un terminal:
  
  export ROS_IP=192.168.1.11 con la dirección de hostname -I

  export ROS_MASTER_URI=http://192.168.1.10:11311 con la dirección IP del LAPTOP

  roslaunch usb2ax_controller hw_controller.launch

* Ahora en la rpi, en otro terminal:

  export ROS_IP=192.168.1.11 con la dirección de hostname -I

  export ROS_MASTER_URI=http://192.168.1.10:11311 con la dirección IP del LAPTOP

  rosrun rosserial_python serial_node.py _port:=/dev/ttyACM1


* FInalmente, de nuevo en el laptop:

  roslaunch bioloid_moveit_config bioloid_planning_execution.launch

  rosrun bioloid_planning plannin_manager.py
