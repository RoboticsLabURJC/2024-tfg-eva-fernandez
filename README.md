# Contenidos del repositorio
* **CoordMoves**: Directorio dónde se encuentra todo el desarrollo del proyecto final
* **docs**: Directorio dónde se sostiene el blog del proyecto
* **pruebas**: Directorio dónde se han hecho pruebas y se ha preparado el proyecto antes de construirlo
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
* Caminar en arco hacia atrás a derecha e izquierda
* Coger una caja
* Dejar uuna caja

Después, para la aplicación de servicios se ha decidido hacer que nuestro pequeño NAO eche una mano en un invernadero, el cual ha sido modelado por mí para funcionar en gazebo, además de un nuevo aspecto para el robot.

## Invernadero

<p align="center">
  <img src="/docs/images_readme/mundo_invernadero.png" alt="Mundo Invernadero" width="600">
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
* Todas las librerías necesarias para python
  * Pybullet
  * numpy
  * sys
  * csv
  * json
  * time
  * atexit
  * rclpy
  * std_msgs
  * sensor_msgs

# Explicación detallada del funcionamiento
Cómo ya se ha explicado en la introducción de este README.md, este trabajo de fin de grado se compone de 2 pilares, los cuáles se explicarán corta, pero detalladamente a continuación, aunque, si quieres una explicación totalmente detallada de este TFG, puedes leer la memoria asociada a él [aquí](/CoordMoves/memoria/TFG_Eva_Fernández_de_la_Cruz.pdf):

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

<p align="center">
  <img src="/docs/images_readme/demo_nao.gif" alt="Demo Joints NAO" width="800">
</p>

Esta demo simplemente es para que veamos cómo se mueve NAO, pero, como se puede ver, el robot está anclado en el aire, por lo que no podemos ver si los movimientos son efectivos o no. Es por eso que para este proyecto se ha desarrollado el editor de movimientos, nao_movement_pattern_creator.py, que hace lo mismo que la demo, pero, además de eso, el robot no está anclado al suelo, para que se pueda ver el impacto del movimiento a realizar y, además, da la opción de guardar patrones de movimiento en un fichero .json, diciendo: "quiero esta posicion en este tiempo". Adjunto un vídeo de su funcionamiento para que se entienda mejor, cabe destacar que el NAO cae para dar a entender que hay movimiento:

<p align="center">
  <img src="/docs/images_readme/editor.gif" alt="Editor de movimientos" width="800">
</p>

Pero, ¿qué tiene que ver esto con ROS2 y Gazebo? Nada.

Es por eso que además de este editor de movimentos ha sido necesario crear un nodo ROS2 capaz de interpretar esos ficheros .json, que tienen la secuencia dentro, para luego enviar esa secuencia al robot simulado en gazebo. Dejo aquí otro vídeo para que se vea el funcionamiento de este intérprete:

<p align="center">
  <img src="/docs/images_readme/interprete.gif" alt="Interprete de movimientos" width="800">
</p>

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

#### Librería CoordMovesLib

Cómo se ha leído en la sección anterior, nuestro NAO es capaz de hacer muchas cosas, pero, ¿qué es lo que necesita una aplicación robótica útil? Ser fácil de usar, por lo que todo el esquema de movimientos relacionado con ros2 (interpretar los patrones, leer sensores, etc.), es realizable desde un sencillo programa en python como el siguiente:

```python
import CoordMovesLib

orientation = CoordMovesLib.get_face() # Gracias a lecturas de IMU, sabemos si hemos caído y de que forma

if orientation == "face normal":
    CoordMovesLib.stand_still()

elif orientation == "face up":
    CoordMovesLib.wakeup_face_up()

elif orientation == "face down":
    CoordMovesLib.wakeup_face_down()

else:
    print("ERROR: No puedo levantarme")
    sys.exit(1)
```
Para hacer esto posible, ha sido necesario desarrollar la librería CoordMovesLib, la cual se encarga de todo lo mencionado anteriormente. Aunque, para poder usarla, es necesario tener todas las dependencias instaladas, y, obviamente, tener lanzada la simulación, además de configurado un paquete de ROS2 que contenga la librería, y respetar las rutas, ya que, al depender de ficheros, son muy importantes.

A continuación, dejo una lista con todas las funciones y clases de esta librería, junto a una breve explicación de cada una de ellas:

**CLASES**
* ***Interpreter_class(file)***: Llama al intérprete de movimientos mencionado anteriormente
* ***setV_class(linear_velocity, steps)***: Hace que NAO ande recto, a la velocidad indicada, los pasos indicados
* ***setW_class(angular_velocity, steps)***: Hace que NAO camine en arco, a la velocidad indicada, los pasos indicados
* ***setNW_class(angular_velocity, steps)***: Hace que NAO camine en arco hacia atrás, a la velocidad indicada, los pasos indicados
* ***setL_class(angular_velocity, steps)***: Hace que NAO camine lateralmente, a la velocidad indicada, los pasos indicados
* ***Read_IMU()***: Devuelve la aceleración en z leyendo las mediciones del IMU


**FUNCIONES**
* ***get_face()***: Devuelve si NAO esta boca arriba, boca abajo, de pie normal o error si no está de ninguna de esas formas
* ***wakeup_face_down()***: Hace que Nao se levante desde cubito prono
* ***wakeup_face_up()***: Hace que Nao se levante desde cubito supino
* ***stand_still()***: Hace que Nao se quede en la posición estándar, de estar quieto
* ***say_hi(hand)***: Hace que Nao salude con la mano que se le pasa como argumento
* ***turn(side, degrees)***: Hace que Nao gire en el sentido que se le pasa como primer argumento, los grados que se le pasan como segundo argumento
* ***grab_box()***: Hace que NAO recoja la caja
* ***release_box()***: Hace que NAO suelte la caja
* ***setArc(linear_velocity, angular_velocity, steps)***: Combina las clases encargadas de las caminatas para que no haya que llamarlas por separado y hacer andar a NAO sea más sencillo.
* ***Interpreter(file)***: Llama a la clase del intérprete, para poder ejecutar correctamente el nodo ROS2
* ***setV(linear_velocity, steps)***: Llama a la clase de caminar recto, para poder ejecutar correctamente el nodo ROS2
* ***setW(angular_velocity, steps)***: Llama a la clase de caminar en arco, para poder ejecutar correctamente el nodo ROS2
* ***setNW(angular_velocity, steps)***: Llama a la clase de camniar en arco hacia atrás, para poder ejecutar correctamente el nodo ROS2
* ***setL(angular_velocity, steps)***: Llama a la clase de camimar lateralmente, para poder ejecutar correctamente el nodo ROS2

##### Modos de caminar

Además de ofrecer la encapsulación necesaria para poder mover a NAO de forma cómoda, disponemos también de los modos de caminar mencionados anteriormente, pero, con una particularidad: El movimiento de caminata recto y el de caminata en arco han sido parametrizados. Esto es, disponemos de V (velocidad lineal), W (velocidad angular), L (velocidad lateral) y una combinación de V y W para no tener que llamar a 2 clases por separado. Dependiendo de la clase, se tiene un número mínimo de 10 o 2 pasos, esto se indicará en este fichero README.md con un * en su nombre en caso de ser 10, y con un ^ en caso de ser 2.

###### Velocidad lineal (clase setV)*

La velocidad lineal puede ser positiva (andar hacia adelante), o negativa (andar hacia atrás) y tiene un valor mínimo (-0.35 ó 0.35) y un valor máximo (4.35 ó -4.35), de este modo, a mayor valor abosoluto, más rápido se moverá nuestro robot, siguiendo este esquema:

<p align="center">
  <img src="/docs/images/semana-27/velocity_value.jpeg" alt="Esquema_Velocidades" width="800">
</p>

###### Velocidad angular (clase setW)*

De igual modo que con la velocidad lineal, tenemos la angular, que puede ser positiva (giro a la derecha) o negativa (giro a la izquierda), y sus valores límite son 0.35 y -0.35 para el mínimo, y 1.9 y -1.9 para el máximo.

También tenemos setNW, que hace lo mismo, pero el arco es hacia atrás.

###### Velocidad angular hacia atrás (clase setNW)*

Esta clase es igual que la anterior, pero los arcos se describen hacia atrás.

###### Velocidad lateral (clase setL)^

De igual modo que con la velocidad lineal y la angular, tenemos la lateral, que puede ser positiva (hacia la derecha) o negativa (hacia la izquierda), y sus valores límite son los mismos que para la velocidad lineal.

###### Velocidad angular en el sitio (clase turnVel)*

Esta clase sirve para que NAO gire en el sitio, a izquierda (velocidad negativa) o derecha (velocidad positiva). Sus valores línme son los mismos que la clase setW.

###### Velocidad combinada V y W (función setArc)

Para encapsular las velocidades lineal y angular, está la clase setArc, para que llamar a la caminata sea más sencillo y directo. Se le pasan por argumento la velocidad lineal, la angular, y los pasos que queremos que NAO dé, y NAO seguirá el siguiente esquema dependiendo de qué valores le pasemos, adjunto un esquema para que se entienda mejor:

<p align="center">
  <img src="/docs/images/semana-31/esquema.jpeg" alt="Esquema_setArc" width="400">
</p>

Ésta es la función que se debería usar para abordar todos los modos de caminar posibles de manera compacta, ya que dependiendo de los parámetros que se le pasan, llama a uno u otro.

## GreenNao

Utilizando la librería explicada anteriormente, se ha desarrollado una aplicación para NAO que consiste en llevar una caja adaptada a él de un lugar del invernadero a otro.

Para hacerlo, primero se ha diseñado dicha caja, ya que el modelo del NAO utilizado para este proyecto no tiene dedos, y era necesario asegurar la caja de alguna forma, por lo que se ha optado por este diseño:

<p align="center">
  <img src="/docs/images_readme/caja.png" alt="Caja_NAO" width="600">
</p>

Ya que es un tamaño adecuado para el robot y hace que no precise de dedos, lo que es una ventaja porque el modelo utilizado tiene las manos sin dedos.

También cabe destacar que la misión de NAO es recoger la caja de una mesa azul (también adecuada a su tamaño) y llevarla hasta una mesa naranja (igual que la azul en cuanto a dimensiones) y dejarla allí.

Para cumplir esta tarea, existen en la librería las funciones grab_box y release_box, que son las encargadas de coger y dejar la caja, respectivamente.

Como se puede apreciar en la imagen del mundo, la tarea es sencilla, ya que NAO simplemente debe recoger la caja, darse la vuelta, ir hasta la mesa naranja y dejarla.

Para que se aprecie la potencia de la librería desarrollada, se adjunta a continuación el código de esta aplicación:

```python
import CoordMovesLib
import time

time.sleep(5)

CoordMovesLib.grab_box()

time.sleep(3)

for i in range(3):
  CoordMovesLib.turn("L", 60)

CoordMovesLib.turn("L", 40)

CoordMovesLib.setArc(1, 0)

time.sleep(3)

CoordMovesLib.release_box()
print("Terminado")
```

# Resultado del proyecto

Gracias a la integración de ambas partes anteriormente explicadas, se ha conseguido el siguiente resultado:

<p align="center">
  <img src="/docs/images_readme/greennao.gif" alt="GrenNao app" width="400">
</p>

Que, cómo se puede ver satisface el objetivo propuesto en la sección anterior.

# Seguimiento del proyecto

Si te interesa saber cómo se ha ido desarrollando este proyecto, puedes visitar su [blog](https://roboticslaburjc.github.io/2024-tfg-eva-fernandez/) en el que se describen los progresos del desarrollo por semanas, desde su inicio, hasta su fin.

# Referencias

Para poder llevar a cabo este proyecto se han visitado los siguientes enlaces:
* [Modelo de NAO utilizado](https://app.gazebosim.org/OpenRobotics/fuel/models/NAO%20with%20Ignition%20position%20controller)
* [Creación de un paquete ros2 python](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
* [Urdf de NAO](https://github.com/ros-naoqi/nao_robot/blob/master/nao_description/urdf/naoV40_generated_urdf/nao.urdf)
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
* [Geogebra (usado para un par de gráficas)](https://www.geogebra.org/calculator)
* [Parámetros físicos en Gazebo explicados](https://classic.gazebosim.org/tutorials?tut=physics_params)
* [Paper de Georgios Pierris y Michail G. Lagoudakis para el editor de movimientos](https://www.pierris.gr/me/downloads/kme.pdf)
* [Paper de 2011 IEEE International Conference on Control System, Computing and Engineering para el editor de movimientos](https://www.researchgate.net/publication/254029603_Humanoid_robot_NAO_Review_of_control_and_motion_exploration)
* [Artículo sobre robots humanoides](https://www.sciencedirect.com/topics/engineering/humanoid-robot)
