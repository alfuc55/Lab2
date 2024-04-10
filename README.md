# Laboratorio 2 
## diseno de sistemas roboticos 

Reporte de la segunda práctica de LRT4012

Aldo FUentes Cruz 16746
Diego de Jesús Gutiérrez Reyes 165096
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

# Nivel avanzado
Para la parte del nivel avanzado se requiere la implementación de controladores para comandar la posición de velocidad mediante instrucciones cmd/vel. Como primer controlador se realizará un únicamente un controlador Proporcional. 
## Controlador P (Proporcional)
Para crear un controlador el primero paso es definir una clase debido a que un controlador siempre tiene la misma estructura, la clase se llamara MoveTurtlePIDControl y se crea con la instrucción $ class MoveTurtlePIDControl$, dentro de esta clase se definirán todas las funciones ya tributos que contienen los controladores. La primera función a crear es la función _init_, esta función inicializara el nodo, se suscribirá al tópico pose para conocer la posición de la tortuga y se definirá el tipo TWIST como mensaje a publicar en el tópico cmd_vel.

        class MoveTurtleProportionalControl:
            def __init__(self):
                rospy.init_node('control_tortuga_x')
                
                # Suscribirse al topic de la posición de la tortuga
                self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
                
                # Publicar en el topic de comandos de movimiento de la tortuga
                self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
                
                # Tasa de publicación de mensajes (10 Hz)
                self.rate = rospy.Rate(10)
                self.current_x = 0

Las siguientes funciones también pertenecen a la clase MoveTurtlePIDControl.  
Funcion para obtener la posicion actual de  la tortuga: 
            def pose_callback(self, pose):
                    # Función que se ejecuta cada vez que llega una actualización de la posición de la tortuga
                    self.current_x = pose.x

La siguiente función “move_turtle_to_desired_x” es la más importante ya que es la definición del controlador. Como primer paso es necesario establecer la ganancia del controlador, como se quiere implementar un controlador puramente proporcional, solo se necesita una constante Kp. El cálculo realizado por el controlador estará dentro de un ciclo while que se repetirá hasta que el error sea lo suficientemente pequeño (0.1) para este ejemplo. El error es calculado restando la posición actual (self.current.x) de la posición deseada (xd). Y este error será multiplicado por la constante Kp, el resultado será el valor de la velocidad lineal en la componente x que será enviado mediante el mensaje msg_Twist.          

             def move_turtle_to_desired_x(self, desired_x):
                # Constante de proporcionalidad del controlador (ajustable)
                Kp = 1
                Ki = 0.1
                Ierror = 0
                while not rospy.is_shutdown():
                    # Calcular el error de posición
                    error_x = desired_x - self.current_x
                    # Calcular la velocidad lineal del movimiento
                    vel_x = Kp * error_x
                    
                    # Crear un mensaje de Twist para enviar el comando de movimiento
                    twist_msg = Twist()
                    twist_msg.linear.x = vel_x
                    
                    # Publicar el mensaje
                    self.velocity_publisher.publish(twist_msg)
                    
                    # Imprimir la posición actual, el error y la variable vel_x en la terminal
                    rospy.loginfo("Posición actual: %f, Error: %f, Velocidad lineal: %f", self.current_x, error_x, vel_x)
                    
                    # Verificar si se alcanza la posición deseada
                    if abs(error_x) < 0.1:
                        rospy.loginfo("Posición deseada alcanzada")
                        break
                    
                    # Esperar hasta la siguiente iteración
                    self.rate.sleep()

Para establecer la posición en x a la que queremos llegar se declara la función “get_desired_x_from_user”, en la cual se pide al usuario ingrese la coordenada en x a la que desea mover la tortuga 

            def get_desired_x_from_user(self):
                    print("Ingrese la posición deseada en el eje x:")
                    return float(input("Coordenada x: "))

            def move_turtle_interactively(self):
                while not rospy.is_shutdown():
                    # Obtener la posición deseada del usuario
                    desired_x = self.get_desired_x_from_user()

                    # Mover la tortuga a la posición deseada
                    self.move_turtle_to_desired_x(desired_x)

Por último, se crea un objeto de la clase MoveTurtlePIDControl y se manda a traer las funciones para mover la tortuga 

    if __name__ == '__main__':
    try:
        move_turtle_proportional = MoveTurtleProportionalControl()
        move_turtle_proportional.move_turtle_interactively()
    except rospy.ROSInterruptException:
        pass

# Controlador PID omnidireccional
Una vez comprendido el código para le controlador el siguiente y último paso es implementar un controlador PID que sea capaz de comandar la velocidad X,Y  de la tortuga asi como también la velocidad angular z. 
La lógica y estructura principal usada para la programación del controlador es la misma que el controlador proporcional mostrada en el código anterior, solo que ahora es necesario incluirla para cada uno de los ejes y realizar una transformación espacial para que las coordenadas dadas sean tomando como punto de referencia la posición actual de la tortuga. 
En la función _init_ es necesario agregar las variables de ultimo error para cada una de las coordenadas. 
        def __init__(self):
                rospy.init_node('control_tortuga_x')
                
                # Suscribirse al topic de la posición de la tortuga
                self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
                
                # Publicar en el topic de comandos de movimiento de la tortuga
                self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
                
                # Tasa de publicación de mensajes (10 Hz)
                self.rate = rospy.Rate(20)
                
                self.current_x = 0
                self.current_y = 0
                self.current_t = 0
                self.last_error_x = 0
                self.last_error_y = 0
                self.last_error_t = 0
                self.error_accumulation = 0
                self.q1anterior = 0

Lo mismo para la función que determina la posición actual de la tortuga
                def pose_callback(self, pose):
                        # Función que se ejecuta cada vez que llega una actualización de la posición de la tortuga
                        self.current_x = pose.x
                        self.current_y = pose.y
                        self.current_t = pose.theta


Para el cálculo de las velocidades lineales, se usarán dos funciones, una función es el controlador para establecer la velocidad lineal en X y la otra función para establecer la velocidad lineal en Y. A pesar de que los controladores van separados, necesitamos en ambos las coordenadas deseadas para X y para Y debido a que se requiere hacer una transformación espacial para cambiar la referencia de las coordenadas al eje de referencia de la tortuga. 
Tambien a diferencia del controlador anterior, ahora se declaran 3 constantes KP, KI y KD, cada una correspondiente a los valores de las ganancias proporcial, intergral y derivativa. 

                 def move_turtle_to_desired_x(self, desired_x, desired_y):
        # Constantes de proporcionalidad, integral y derivativa del controlador (ajustables)
        Kp = 1
        Ki = 0.01
        Kd = 0.1
        x = self.current_x
        y = self.current_y
        q1 = self.current_t
        while not rospy.is_shutdown():
            # Calcular el error de posición
            #Ajuste del valor actual de x con referencia a la tortuga 
                #dx*cos(q1) - conj(y)*sin(q1) - cos(q1)*conj(x) + dy*sin(q1)
            currentx = self.current_x*math.cos(q1) - y*math.sin(q1) - math.cos(q1)*x + self.current_y*math.sin(q1)
            error_x = desired_x - currentx
            
            # Sumar el error a la acumulación de errores
            self.error_accumulation += error_x
            
            # Calcular la velocidad lineal del movimiento
            vel_x = Kp * error_x + Ki * self.error_accumulation + Kd * (error_x - self.last_error_x)
            
            # Guardar el error actual para usarlo en la próxima iteración
            self.last_error_x = error_x
            
            # Crear un mensaje de Twist para enviar el comando de movimiento
            twist_msg = Twist()
            twist_msg.linear.x = vel_x
            
            # Publicar el mensaje
            self.velocity_publisher.publish(twist_msg)
            
            # Imprimir la posición actual, el error y la variable vel_x en la terminal
            rospy.loginfo("Posición actual: %f, Error: %f, Velocidad lineal: %f", self.current_x, error_x, vel_x)
            
            # Verificar si se alcanza la posición deseada
            if abs(error_x) < 0.1:
                rospy.loginfo("Posición deseada alcanzada")
                break
            
            # Esperar hasta la siguiente iteración
            self.rate.sleep()

    def move_turtle_to_desired_y(self, desired_y, desired_x):
        # Constantes de proporcionalidad, integral y derivativa del controlador (ajustables)
        Kp = 1
        Ki = 0.01
        Kd = 0.1
        x = self.current_x
        y = self.current_y
        q1 = self.current_t
        while not rospy.is_shutdown():
            # Calcular el error de posición
            # conj(x)*sin(q1) - cos(q1)*conj(y) + dy*cos(q1) - dx*sin(q1)
            currenty = x*math.sin(q1) - math.cos(q1)*y + self.current_y*math.cos(q1) - self.current_x*math.sin(q1)
            error_y = desired_y - currenty
            
            # Sumar el error a la acumulación de errores
            self.error_accumulation += error_y
            
            # Calcular la velocidad lineal del movimiento
            vel_y = Kp * error_y + Ki * self.error_accumulation + Kd * (error_y - self.last_error_y)
            
            # Guardar el error actual para usarlo en la próxima iteración
            self.last_error_y = error_y
            
            # Crear un mensaje de Twist para enviar el comando de movimiento
            twist_msg = Twist()
            twist_msg.linear.y = vel_y
            
            # Publicar el mensaje
            self.velocity_publisher.publish(twist_msg)
            
            # Imprimir la posición actual, el error y la variable vel_x en la terminal
            rospy.loginfo("Posición actual: %f, Error: %f, Velocidad lineal: %f", self.current_y, error_y, vel_y)
            
            # Verificar si se alcanza la posición deseada
            if abs(error_y) < 0.1:
                rospy.loginfo("Posición deseada alcanzada")
                break
            
            # Esperar hasta la siguiente iteración
            self.rate.sleep()

Por último, un último controlador es implementado para comandar la velocidad angular de la tortuga. 
 def move_turtle_to_desired_t(self, desired_t):
        # Constantes de proporcionalidad, integral y derivativa del controlador (ajustables)
        Kp = 1
        Ki = 0.001
        Kd = 0.12
        self.error_accumulation = 0 
        desiredt=desired_t
        while not rospy.is_shutdown():
            
            # Calcular el error de posición
            error_t = desiredt - self.current_t
            
            # Sumar el error a la acumulación de errores
            self.error_accumulation += error_t
            
            # Calcular la velocidad lineal del movimiento
            vel_t = Kp * error_t + Ki * self.error_accumulation + Kd * (error_t - self.last_error_t)
            
            # Guardar el error actual para usarlo en la próxima iteración
            self.last_error_t = error_t
            
            # Crear un mensaje de Twist para enviar el comando de movimiento
            twist_msg = Twist()
            twist_msg.angular.z = vel_t
            
            # Publicar el mensaje
            self.velocity_publisher.publish(twist_msg)
            
            # Imprimir la posición actual, el error y la variable vel_x en la terminal
            rospy.loginfo("Posición actual: %f, Error: %f, Velocidad angular: %f", self.current_t, error_t, vel_t)
            
            # Verificar si se alcanza la posición deseada
            if abs(error_t) < 0.05:
                rospy.loginfo("Posición deseada alcanzada")
               
                break
            
            # Esperar hasta la siguiente iteración
            self.rate.sleep()
            self.q1anterior = self.current_t

Para obtener las coordenadas deseadas para la tortuga se usa una entrada del usuario, es importante notar que para la posición angular la entrada se pide en grados y dentro del código se cambia a radianes 

            def get_desired_x_from_user(self):
                    print("Ingrese la posición deseada en el eje x:")
                    return float(input("Coordenada x: "))

                def get_desired_y_from_user(self):
                    print("Ingrese la posición deseada en el eje y:")
                    return float(input("Coordenada y: "))
                
                def get_desired_t_from_user(self):
                    print("Ingrese la posición deseada de rotacion:")
                    pt = float(input("angulo theta: "))*(math.pi/180)
                    return pt

Ahora solo queda definir la función que hace que la tortuga se mueva y mandar a llamar a las funciones previamente declaradas 

              def move_turtle_interactively(self):
        while not rospy.is_shutdown():
            # Obtener la posición deseada del usuario
            desired_x = self.get_desired_x_from_user()
            desired_y = self.get_desired_y_from_user()
            desired_t = self.get_desired_t_from_user()
            # Mover la tortuga a la posición deseada
            self.move_turtle_to_desired_x(desired_x,desired_y)
            self.move_turtle_to_desired_y(desired_y,desired_x)
            self.move_turtle_to_desired_t(desired_t)

        if __name__ == '__main__':
            try:
                move_turtle_pid = MoveTurtlePIDControl()
                move_turtle_pid.move_turtle_interactively()
            except rospy.ROSInterruptException:
                pass

Con el objetivo de probar la funcionalidad del controlador, se realizo de nuevo la trayectoria cuadrada y triangular pero esta vez utilizando el controlador PID, el resultado se muestra en las siguientes imágenes 

 ![SquarePID](https://github.com/alfuc55/Lab2/blob/main/squarePID.png)

 ![TrianglePID](https://github.com/alfuc55/Lab2/blob/main/TrianglePID.png)               


 # Conclusiones

 El robot TurtleSim es una herramienta de ROS que permite comprender el funcionamiento de distintas tareas, comandos y configuraciones del software, puesto que al programarle secuencias en los distintos niveles de la práctica, se observa como cada una de estas modifica y mejora el comportamiento del mismo, desde el nivel básico donde se aprende la comunicación enetre nodos y la importancia del envío de mensajes, el nivel medio que requiere que el usuario sea el que envía la infomación, así como la programación de las rutas para implementar un cuadrado y un triángulos, posteriormente el nivel avanzado en el que se emplean los controladores para mejorar el movimiento del turtlesim, los resultados son satisfactorios al cumplir cada uno de lo niveles con los objetivos planteados.
