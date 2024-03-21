# Laboratorio 2 
## diseno de sistemas roboticos 

Reporte de la segunda práctica de LRT4012

Aldo FUentes Cruz 16746
# Introducción

En ROS (Robot Operating System), los "workspaces" son directorios que contienen paquetes de ROS y que han sido configurados para ser compilados.Los workspaces se utilizan principalmente para organizar y compilar los paquetes de ROS que forman parte de un proyecto específico. 
Cuando estás trabajando en un proyecto de ROS, suele ser conveniente tener un workspace específico para ese proyecto. Esto te permite mantener separados los paquetes y dependencias de diferentes proyectos. Además, puedes configurar tu entorno de trabajo (workspace) para que apunte a los recursos específicos de ese proyecto, como las bibliotecas y ejecutables compilados

En esta práctica se creará un workspace para agregar los paquetes que serán utilizado a lo largo de los laboratorios de la materia 
Los workspace necesitan ser creados en un directorio el cual contendrá los paquetes de ROS que se usaran, el directorio puede tener cualquier nombra. 
cada paquete de ROS es un conjunto de archivos y directorios que contiene código fuente, scripts, archivos de configuración y posiblemente otros recursos necesarios para un componente específico de un sistema ROS.
Un comando importante durante este proceso es el comando carkin, catkin es el sistema de construcción de ROS, similar a CMake, que se utiliza para compilar los paquetes de ROS dentro de un workspace. Catkin se encarga de manejar las dependencias entre los paquetes y de generar los archivos de construcción necesarios para compilar los paquetes correctamente.

## Creando paquetes catkin en ROS##
Para crear un nuevo paquete es necesario ejecutar los siguientes comandos en orden:

$ cd ~/catkin_ws/src

$ catkin_create_pkg *Nombre del paquete* std_msgs rospy roscpp

Esto crea un paquete dependiente de std_msgs, rospy y roscpp. Dentro de los paquetes es posible introducir la información y códigos necesarios para que funcione el proyecto.

## Nodos en ROS
Los nodos en ROS son procesos independientes que realizan tareas específicas dentro de un sistema robótico o de otro tipo de sistema controlado por ordenador. Dos nodos altamente empleados son el *talker* y *listener*, los cuales permiten el envío y recepción de nformación respectivamente. Los nodos son programados en Python y son ejecutados para que se comuniquen dentro de la red do ROS 

## Controladores P, PI y PID ##
Los controladores de procesos son necesarios para diseñar sistemas seguros y productivos, gracias a estos es que es posible que el sistema se regule a sí misma a partir de un resultado, existen una gran cantidad de controladores, algunos de los más básicos son el P, PI y PID.

El controlador proporcional (P), es la forma más sencilla de control y se caracteriza por minimizar la fluctuación en la variable de procesos, proporciona una respuesta rápida, pero al aplicarse en un sistema complejo, la diferencia de tiempo de respuesta se acumula, lo que provoca que el controlador responda más tarde  lo requerido, lo que genera desviación.

El controlador Proporcional-Integral (PI), es un controlador que mezcla el control poroporcional e integral, de este último se destaca porque es capaz de corregir cualqueir desviación que exista, de modo que el controlador PI, si bien es más lento en respeusta que el controlador P, posee la capacidad de devolver al sistema a su punto de ajuste, este controlador correlaciona la salida del controlador con el error y la integral del error.

Por último, el controlador Proporiconal-Integral-Derivado, es la mezcla del controlador PI sumándole un controlador derivado (D), la adición de este útlimo permite aumentar la respuesta del controlador porque predice perturbaciones al sistema midiendo el cambio en el error. Este controlador correlaciona la salida del controlador con el error, la integral del error y la derivada del error, si bien es el que mejor funciona de los 3, también es el más caro y complciado de diseñar.

# Nivel Basico 
Para esta primera parte de la practica denominada como Nivel Básico, se creará un workspace y se agregara un paquete llamado Practicas_Lab, donde se almacenarán los codigos usados para esta clase. Tambien se explorarán y usarán más el concepto de nodos para entender mejor su funcionamiento  

Para completar el nivel básico de la practica, se requiere crear un paquetes denominado "practicas_lab" de ros con las dependencias rospy, roscpp y std_msgs. 
Para lograr esto primero debemos cambiar hacia el directorio donde queremos almacenar nuestro paquete mediante el comando cd. 
Esto se realiza mediante los siguientes comandos:

$ cd ~/catkin_ws/src
$ catkin_create_pkg practicas_lab std_msgs rospy roscpp

Posteriormente se requiere agregar archivos correspondientes a nodos listener y talker dentro de esta carpeta. Dichos archivos se encuentran disponibles en GitHub en el siguiente repositorio:
https://github.com/cesar-martinez-torres/Formatos_LRT4102/tree/main/Codigos_clase/Lab2/Basic

Con la dirección URL obtenida, se deben de clonar los archivos de Python en nuestra carpete practicas_lab, para ello es posbile emplear el siguiente comando:

git clone https://github.com/cesar-martinez-torres/Formatos_LRT4102/tree/main/Codigos_clase/Lab2/Basic

Con los archivos copiados dentro de nuestra carptea, es necesario compilar el paquete, una vez compilado se abre el Visual Code Studio para ejecutar los archivos de Python, cuandos se ejecuten, el archivo talker.pt comenzará a emitir un mensaje, el cual se podrá ver desde el programa listener.py
![Listener](https://github.com/alfuc55/Lab2/blob/main/listener.png)
![Talker](https://github.com/alfuc55/Lab2/blob/main/Talker.png)

# Nivel Intermedio
La segunda parte de la practica es el nivel intermedio, las tareas a realizar consisten en la creación de un nodo que sea capaz de comandar al nodo de la tortuga Turtlesim incluida en ROS, y después programar un nodo que envie una trayectoria triangular y cuadrada a la tortuga. 
Para este nivel intermedio se usará el nodo turtlesim, Turtlesim es un paquete de ROS que proporciona una simple simulación gráfica de una tortuga robótica en un espacio bidimensional 
Al igual que otros nodos en ROS, Turtlesim se comunica utilizando el sistema de publicación y suscripción de tópicos

## Turtlesim 
Para ejecutar el simulador turtlesim primero se debe correr ROS con el comando $ rosrun, luego en una nueva terminal usar el comando que abre el simulador: 
$ rosrun turtlesim turtlesim_node$

![Turtlesim](https://github.com/alfuc55/Lab2/blob/main/turtlesimnode.png)

### Turtlesim Keyboard Teleop

La tortuga se comanda mediante el topico /cmd_vel, este comando es un tópico comúnmente utilizado para enviar comandos de velocidad a un robot móvil. Este tópico es fundamental en el control de movimiento de los robots y es utilizado por muchos controladores de movimiento para recibir instrucciones sobre cómo moverse. Los mensajes enviados a /cmd_vel son del tipo geometry_msgs/Twist. Este tipo de mensaje contiene dos campos: linear y angular, que representan las velocidades lineal y angular del robot respectivamente.
Tomando esto en cuenta se programo el siguiente nodo de ROS usando Python para leer las teclas pulsadas por el usuario y mandar los mensajes adecuados y que turtlesim pueda escucharlos y asi poder mover la tortuga. 

        ## declaracion de librerias
        #!/usr/bin/env python
        # license removed for brevity
        import rospy # libreria de ROS
        from std_msgs.msg import String  # libreria de los mensajes 
        from geometry_msgs.msg import Twist # libreria mensajes tio Twist
        from turtlesim.srv import TeleportAbsolute, TeleportRelative
        from pynput import keyboard as kb # libreria para leer la teclas presionadas 
        import termios, sys, os
        import tty
        import select

        # funcion para obtener las teclas presionadas
        def getKey():
            tty.setraw(sys.stdin.fileno())
            select.select([sys.stdin], [], [], 0)
            key = sys.stdin.read(1)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            return key
        # funcion para definir el valor del mensaje Twist a enviar dependiendo las teclas pulsadas
        def movimiento(tecla):
            print('Se ha pulsado la tecla ' + str(tecla))
            twist.angular.z= 0
            twist.linear.x = 0
            # adelante 
            if tecla == "w":
                twist.linear.x= 1
            # atras
            elif tecla == "s":
                twist.linear.x =-1
            # Izquierda
            elif tecla == "a":
                twist.angular.z= 1
            # derecha
            elif tecla == "d":
                twist.angular.z= -1
            # salir del programa 
            elif tecla == "r":
                twist.angular.z= 0
                twist.linear.x = 0
        # funcion que manda a traer las funciones para leer las teclas y deifinir el tipo de mivimiento
        def talker():
            while(1):
                    # Asigna a key los valores del vector de la tecla presionada
                    key = getKey()
                    if key=="r":
                        break 
                    # Obtiene los valores de x, y, z 
                    movimiento(key)   
                    # se publica el mensaje  
                    pub.publish(twist)

        if __name__ == '__main__':
            # se configura el nodo de ROS
            settings = termios.tcgetattr(sys.stdin)
            # PUBLISHER EN turtle/cmd_vel UTILIZANDO LOS VALORES DE TWIST
            pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
            # INICIALIZA EL NODO: teleop_twist_keyboard
            rospy.init_node('teleoper', anonymous= False)
            pub.rate = rospy.Rate(10)
            twist = Twist()

            try:
                # se manda a traer la funcion para obtener la tecla pulsada 
                key = getKey()
                # se usa la funcion que envia el mensaje 
                talker()
            except rospy.ROSInterruptException:
                pass

 ![TeleopTurtlesim](https://github.com/alfuc55/Lab2/blob/main/Teleop.png)   

### Trajectoria cuadrado y triangulo
Para completar la segunda tarea del nivel básico correspondiente a la programación de trayectorias usamos los mismos mensajes y tópicos usados en el código anterior, solo que esta vez los movimientos son fijos por un periodo de tiempo. 
        # declaracion de librerias 
        #!/usr/bin/env python
        import rospy
        from geometry_msgs.msg import Twist
        from std_msgs.msg import String
        from geometry_msgs.msg import Twist
        from turtlesim.srv import TeleportAbsolute, TeleportRelative
        from turtlesim.msg import Pose
        # funcion que crea el movimiento 
        def move_triangular():
            # Crear un objeto Twist para enviar comandos de movimiento
            move.linear.x = 4  # Velocidad lineal en el eje x
            # Definir la velocidad angular
            move.angular.z = 0.0  # No hay rotación inicialmente
            # crar un ciclo que ejecuta los 3 movimientos para el triangulo
            for n in range(3):
                    #Definir la velocidad angular para le movimiento lineal
                    move.angular.z = 0.0 
                    rospy.sleep(4)  # Tiempo para avanzar un lado
                    pub.publish(move) # publicar mensaje
                    #girar
                    move.linear.x = 0
                    move.angular.z = 2.2
                    rospy.sleep(3.5) # esperar 
                    pub.publish(move) # publicar mensaje 
                    move.linear.x = 4.4
                    
        if __name__ == '__main__':
            # Inicializar el nodo de ROS
            rospy.init_node('move_Triangular', anonymous=False)
            # Suscribirse al topic de la posición de la tortuga
            posicion = rospy.Subscriber('/turtle1/pose', Pose, queue_size=10)
                
            # Crear un publicador para enviar comandos de movimiento a TurtleSim
            pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
            pub.rate = rospy.Rate(10)
            move = Twist()
            try:
                # usar funcion que manda una trayectoria triangular 
                move_triangular()
            except rospy.ROSInterruptException:
                pass

![Triangularurtlesim](https://github.com/alfuc55/Lab2/blob/main/Triangular.png)   

Para hacer la trayectoria cuadrada se uso el mismo principio que en el nodo anterior solo que esta vez el numero de movimientos determinado por el ciclo for se aumento a 4. Este nuevo movimiento solo se definio en una nueva funcion:

        def move_square():
        
            # Crear un objeto Twist para enviar comandos de movimiento
            move.linear.x = 5  # Velocidad lineal en el eje x
            # Definir la velocidad angular
            move.angular.z = 0.0  # No hay rotación inicialmente
            # Definir 4 moviemientos
            for n in range(4):
                    move.linear.x = 4
                    move.angular.z = 0.0 
                    rospy.sleep(4)  # Tiempo para avanzar un lado
                    pub.publish(move)
                    
                    move.linear.x = 0
                    move.angular.z = 1.55
                    rospy.sleep(4)
                    pub.publish(move)

![TSquareurtlesim](https://github.com/alfuc55/Lab2/blob/main/square.png)   