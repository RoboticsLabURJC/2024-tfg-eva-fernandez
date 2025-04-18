# Contenidos del repositorio
* **docs**: Directorio dónde se sostiene el blog del proyecto
* **pruebas**: Directorio dónde se han hecho pruebas y se ha preparado el proyecto antes de construirlo
* **GreenNao**: Directorio dónde se encuentra todo el desarrollo del proyecto final
* **.gitignore**: Fichero que indica a git qué otros ficheros no deben subirse al repositorio
* **README.md**: ¡Fichero que estás leyendo ahora mismo! Contiene un resumen de lo que trata el proyecto, cómo funciona, etc

# Introducción

Este trabajo de fin de grado consiste en construir una aplicación para convertir al robot NAO de Aldebaran Robotics en un robot de servicio. Para ello, ha sido necesario crear patrones de movimiento para que pueda operar correctamente, estos son:
* Caminar recto hacia adelante y hacia atrás
* Girar en el sitio ya sea a la derecha o la izquierda
* Desplazarse lateralmente hacia la izquierda o derecha
* Levantarse si se cae boca arriba
* Levantarse si se cae boca abajo
* Caminar en arco hacia la derecha y la izquierda

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

## Movimiento

#### Esquema general de movimiento

El movimento del NAO se da de manera sencilla publicando posiciones en sus topics, cada uno un grado de libertad del propio robot, con un total de 24:

<p align="center">
  <img src="/docs/images_readme/topics_nao.png" alt="Topics" width="800">
</p>

A continuación dejo un esquema dónde se ve claramente de qué movimiento se encarga cada topic:

<p align="center">
  <img src="/docs/images/semana-7/esquema_joints_NAO.jpeg" alt="Esquema_Topics" width="800">
</p>

Como se puede intuir, aunque el movimiento esté tan "controlado", es decir, que tengamos tantos grados de libertad para poder hacer los movimientos que veamos oportunos, no es sencillo publicar números a cada una de las articulaciones por separado y ver qué ocurre, por lo que para este proyecto se desarrolló un editor de movimientos, el cuál está basado en la siguiente demo de gazebo harmonic:

<video width="800" controls>
  <source src="/docs/images_readme/demo_nao.webm" type="video/webm">
  Your browser does not support the video tag.
</video>

Esta demo simplemente es para que veamos cómo se mueve NAO, pero, como se puede ver, el robot está anclado en el aire, por lo que no podemos ver si los movimientos son efectivos o no. Es por eso que para este proyecto se ha desarrollado el editor de movimientos, nao_movement_pattern_creator.py, que hace lo mismo que la demo, pero, además de eso, el robot no está anclado al suelo, para que se pueda ver el impacto del movimiento a realizar y, además, da la opción de guardar patrones de movimiento en un fichero .json, diciendo: "quiero esta posicion en este tiempo". Adjunto un vídeo de su funcionamiento para que se entienda mejor, cabe destacar que el NAO cae para dar a entender que hay movimiento:

<video width="800" controls>
  <source src="/docs/images_readme/editor.webm" type="video/webm">
  Your browser does not support the video tag.
</video>

Pero, ¿qué tiene que ver esto con ROS2 y Gazebo? Nada.

Es por eso que además de este editor de movimentos ha sido necesario crear un nodo ROS2 capaz de interpretar esos ficheros .json, que tienen la secuencia dentro, para luego enviar esa secuencia al robot simulado en gazebo. Dejo aquí otro vídeo para que se vea el funcionamiento de este intérprete:

<video width="800" controls>
  <source src="/docs/images_readme/interprete.webm" type="video/webm">
  Your browser does not support the video tag.
</video>

Una vez conseguido el intérprete, podíamos pasar a tareas más complejas, cómo la caminata, la cual se divide en varias partes:

1. Caminata recta hacia adelante
2. Caminata recta hacia atrás
3. Caminata en arco hacia la derecha
4. Caminata en arco hacia la izquierda
5. Caminata lateral hacia la derecha
6. Caminata lateral hacia la izquierda

Que, por suerte, todas, excepto la 3 y la 4 (las cuales fueron desarrolladas combinando distintos patrones), son proporcionadas por el simulador webots (enlace al final de este README.md), pero, en lugar de en formato json, cómo nosotros trabajamos, están en formato motion, un formato idéntico a csv, exceptuando la extensión, así que, una vez cambiados a csv, pueden ser interpretados por el intérprete, ya que no sólo es capaz de interpretar ficheros json, también puede interpretar ficheros csv.

Además de la caminata, es necesario que NAO esté preparado para posibles caídas, esto significa:

1. Debe ser capaz de levantarse desde cúbito supino (boca arriba)
2. Debe ser capaz de levantarse desde cúbito prono (boca abajo)

El punto 2 también nos lo da resuelto webots, sin embargo, el punto 1 tuvo que ser desarrollado a mano utilizando el editor de movimientos mencionado anteriormente, para ello, lo que hace es primero darse la vuelta, para poder llamar al patrón de webots, y así aprovechar el fichero para ambos movimientos.

Cabe destacar también que nuestro NAO cuenta con un sensor IMU, para así poder detectar si ha caído, y una cámara.

#### Librería GreenNaoLib

Cómo se ha leído en la sección anterior, nuestro NAO es capaz de hacer muchas cosas, pero, ¿qué es lo que necesita una aplicación robótica útil? Ser fácil de usar, por lo que todo el esquema de movimientos relacionado con ros2 (interpretar los patrones, leer sensores, etc.), es realizable desde un sencillo programa en python como el siguiente:

```python
import GreenNaoLib

orientation = GreenNaoLib.get_face() # Gracias a lecturas de IMU, sabemos si hemos caído y de que forma

if orientation == "face normal":
    GreenNaoLib.stand_still()

elif orientation == "face up":
    GreenNaoLib.wakeup_face_up()

elif orientation == "face down":
    GreenNaoLib.wakeup_face_down()

else:
    print("ERROR: No puedo levantarme")
    sys.exit(1)
```
Para hacer esto posible, ha sido necesario desarrollar la librería GreenNaoLib, la cual se encarga de todo lo mencionado anteriormente. Aunque, para poder usarla, es necesario tener todas las dependencias instaladas, y, obviamente, tener lanzada la simulación, además de configurado un paquete de ROS2 que contenga la librería, y respetar las rutas, ya que, al depender de ficheros, son muy importantes.

A continuación, dejo una lista con todas las funciones y clases de esta librería, junto a una breve explicación de cada una de ellas:

**FUNCIONES**
* ***start()***: Inicia rclpy, necesario para poder llamar a las clases de la librería
* ***finish()***: Finaliza rclpy
* ***get_face()***: Devuelve si NAO esta boca arriba, boca abajo, de pie normal o error si no está de ninguna de esas formas
* ***wakeup_face_down()***: Hace que Nao se levante desde cubito prono
* ***wakeup_face_up()***: Hace que Nao se levante desde cubito supino
* ***stand_still()***: Hace que Nao se quede en la posición estándar, de estar quieto
* ***say_hi(hand)***: Hace que Nao salude con la mano que se le pasa como argumento
* ***side_step(side)***: Hace que Nao ande de lado en la dirección que se le pasa com argumento
* ***turn(side, degrees)***: Hace que Nao gire en el sentido que se le pasa como primer argumento, los grados que se le pasan como segundo argumento

**CLASES**
* ***Interpreter(file)***: Llama al intérprete de movimientos mencionado anteriormente
* ***setV(linear_velocity, steps)***: Hace que NAO ande recto, a la velocidad indicada, los pasos indicados
* ***setW(angular_velocity, steps)***: Hace que NAO camine en arco, a la velocidad indicada, los pasos indicados
* ***Read_IMU()***: Devuelve la aceleración en z leyendo las mediciones del IMU

##### Modos de caminar

Además de ofrecer la encapsulación necesaria para poder mover a NAO de forma cómoda, disponemos también de los modos de caminar mencionados anteriormente, pero, con una particularidad: El movimiento de caminata recto y el de caminata en arco han sido parametrizados. Esto es, disponemos de V (velocidad lineal) y W (velocidad angular).

###### Velocidad lineal (clase setV)

La velocidad lineal puede ser positiva (andar hacia adelante), o negativa (andar hacia atrás) y tiene un valor mínimo (-0.35 ó 0.35) y un valor máximo (4.35 ó -4.35), de este modo, a mayor valor abosoluto, más rápido se moverá nuestro robot, siguiendo este esquema:

<p align="center">
  <img src="/docs/images/semana-27/velocity_value.jpeg" alt="Esquema_Velocidades" width="800">
</p>

###### Velocidad angular (clase setW)

De igual modo que con la velocidad lineal, tenemos la angular, que puede ser positiva (giro a la derecha) o negativa (giro a la izquierda), y sus valores límite son 0.35 y -0.35 para el mínimo, y 1.9 y -1.9 para el máximo.

## GrenNao (próximamente)

Utilizando la librería explicada anteriormente, se ha desarrollado una aplicación para NAO que ...... (próximamente)

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
* [Tutorial de como usar tus programas como librería para un nodo ros2](https://www.youtube.com/watch?v=4XCRIaVNX6k)

