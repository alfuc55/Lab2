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

## Nivel Basico 
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
![Talker](https://github.com/alfuc55/Lab2/blob/main/listener.png)
