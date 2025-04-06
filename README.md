# Contenidos del repositorio
* **docs**: Directorio dónde se sostiene el blog del proyecto
* **pruebas**: Directorio dónde se han hecho pruebas y se ha preparado el proyecto antes de construirlo
* **GreenNao**: Directorio dónde se encuentra todo el desarrollo del proyecto final
* **.gitignore**: Fichero que indica a git qué otros ficheros no deben subirse al repositorio
* **README.md**: ¡Fichero que estás leyendo ahora mismo! Contiene un resumen de lo que trata el proyecto, cómo funciona, etc

# Introducción

Este trabajo de fin de grado consiste en construir una aplicación para convertir al robot NAO de Aldebaran Robotics en un robot de servicio. Para ello, ha sido necesario crear patrones de movimiento para que pueda operar correctamente, estos son (aunque aún están en desarrollo):
* Caminar recto hacia adelante
* Girar en el sitio ya sea a la derecha o la izquierda
* Desplazarse lateralmente hacia la izquierda o derecha
* Levantarse si se cae boca arriba
* Levantarse si se cae boca abajo

Después, para la aplicación de servicios se ha decidido hacer que nuestro pequeño NAO eche una mano en un invernadero, el cual ha sido modelado por mí para funcionar en gazebo, además de un nuevo aspecto para el robot.

## Invernadero

### Primera versión (actual)

<p align="center">
  <img src="https://github.com/user-attachments/assets/e8a3907e-f71d-43bb-86f1-4fca4d75156e" alt="Mundo Invernadero" width="800">
</p>

## NAO 

<p align="center">
  <img src="https://github.com/user-attachments/assets/a7a0bbff-29e4-4def-b91c-6f83c1629b97" alt="GrenNao" width="400">
</p>

# Requisitos de ejecución

Para que este TFG funcione correctamente, se necesita:
* Ubuntu 22.04
* Ros2 humble
* Gazebo Harmonic (versión 8.9.0)
* Todas las dependencias necesarias para comunicar ros y gazebo
  * ros-gz-sim
  * ros-humble-ros-gz-bridge

# Explicación detallada del funcionamiento
Cómo ya se ha explicado en la introducción de este README.md, este trabajo de fin de grado se compone de 2 pilares, los cuáles se explicarán corta, pero detalladamente a continuación, aunque, si quieres una explicación totalmente detallada de este TFG, puedes leer la memoria asociada a él [aquí]() (próximamente):

## Movimiento (próximamente)
## GrenNao (próximamente)

# Resultado del proyecto (próximamente)

# Seguimiento del proyecto

Si te interesa saber cómo se ha ido desarrollando este proyecto, puedes visitar su [blog](https://roboticslaburjc.github.io/2024-tfg-eva-fernandez/) en el que se describen los progresos del desarrollo por semanas, desde su inicio, hasta su fin.

# Referencias

Para poder llevar a cabo este proyecto se han visitado los siguientes enlaces:
* [Modelo de NAO utilizado](https://app.gazebosim.org/OpenRobotics/fuel/models/NAO%20with%20Ignition%20position%20controller)
* [Creación de un paquete ros2 python](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
* [Urdf de NAO](https://github.com/ros-naoqi/nao_robot/blob/master/nao_description/urdf/naoV40_generazted_urdf/nao.urdf)
* [Información sobre el plugin de movimiento de joints de gazebo](https://gazebosim.org/api/gazebo/6/classignition_1_1gazebo_1_1systems_1_1JointPositionController.html) 
* [Correspondencia de mensajes entre gazebo y ros2 humble](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/README.md)
* [Tutorial de ros-gz-bridge](https://www.youtube.com/watch?v=DsjJtC8QTQY)
* [NAO real caminando](https://www.youtube.com/watch?v=NbnELOZbsls)
* [NAO real levantándose si cae boca arriba](https://www.youtube.com/watch?v=EX_cOJhVQSQ) 
* [NAO real levantándose si cae boca abajo](https://www.youtube.com/watch?v=TvdMgqSbppQ) 
* [Otro NAO real caminando](https://youtu.be/bP8bhqcgTDA?si=pMBBCN62_7feawBX) 
* [Partido de fútbol con NAOS para la RoboCup 2018](https://youtu.be/H8xc6LpiNVs?si=_eXNvRMYu_hxb_oS)
* [Tutorial para crear un mundo personalizado en gazebo](https://www.youtube.com/watch?v=K4rHglJW7Hg)
* [Documentación sobre caminatas de Aldebaran](http://doc.aldebaran.com/2-1/naoqi/motion/control-walk.html)
* [Página oificial de NAO](https://us.softbankrobotics.com/nao)
* [Enlace de descarga para el simulador Webots](https://cyberbotics.com/)
* [Ejemplo de publicador y suscriptor para ros2 humble en python](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)

