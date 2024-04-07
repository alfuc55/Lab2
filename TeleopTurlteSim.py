#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, TeleportRelative
from pynput import keyboard as kb
import termios, sys, os
import tty
import select

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def movimiento(tecla):
    print('Se ha pulsado la tecla ' + str(tecla))
    twist.angular.z= 0
    twist.linear.x = 0
    if tecla == "w":
        twist.linear.x= 1
    elif tecla == "s":
        twist.linear.x =-1
    elif tecla == "a":
        twist.angular.z= 1
    elif tecla == "d":
        twist.angular.z= -1
    elif tecla == "r":
        twist.angular.z= 0
        twist.linear.x = 0

def talker():
    while(1):
            # Asigna a key los valores del vector de la tecla presionada
            key = getKey()
            if key=="r":
                break 
            # Obtiene los valores de x, y, z 
            movimiento(key)   
           # Se actualizan los valores de x, y y z para el twist.
            pub.publish(twist)



if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    # PUBLISHER EN turtle/cmd_vel UTILIZANDO LOS VALORES DE TWIST
    pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
    # INICIALIZA EL NODO: teleop_twist_keyboard
    rospy.init_node('teleoper', anonymous= False)
    pub.rate = rospy.Rate(10)
    twist = Twist()

    try:
        key = getKey()
        talker()
    except rospy.ROSInterruptException:
        pass