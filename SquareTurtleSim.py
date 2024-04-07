#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, TeleportRelative
from turtlesim.msg import Pose

def move_square():
  
    # Crear un objeto Twist para enviar comandos de movimiento
    move.linear.x = 5  # Velocidad lineal en el eje x
    # Definir la velocidad angular
    move.angular.z = 0.0  # No hay rotación inicialmente
    # Definir la velocidad lineal
    for n in range(4):
            move.linear.x = 4
            move.angular.z = 0.0 
            rospy.sleep(4)  # Tiempo para avanzar un lado
            pub.publish(move)
            
            move.linear.x = 0
            move.angular.z = 1.55
            rospy.sleep(4)
            pub.publish(move)
            
if __name__ == '__main__':
      # Inicializar el nodo de ROS
    rospy.init_node('move_square', anonymous=False)
     # Suscribirse al topic de la posición de la tortuga
    posicion = rospy.Subscriber('/turtle1/pose', Pose, queue_size=10)
        
    # Crear un publicador para enviar comandos de movimiento a TurtleSim
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    pub.rate = rospy.Rate(10)
    move = Twist()
    try:
        move_square()
    except rospy.ROSInterruptException:
        pass