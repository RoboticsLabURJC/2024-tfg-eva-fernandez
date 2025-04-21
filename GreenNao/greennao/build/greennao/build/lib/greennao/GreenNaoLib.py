import sys
import csv
import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu

# Iniciar rclpy para poder usar las funciones y clases e esta librería ----------------------------------

def start():
    rclpy.init()

# Parar rclpy (llamar al final del programa de NAO) -----------------------------------------------------
def finish():
    rclpy.shutdown()

# Clase para replicar los movimientos de un fichero -----------------------------------------------------
class Interpreter(Node):
    def __init__(self, file_name: str):
        super().__init__('interpreter')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_ALL,
            depth=100
        )

        self.file_name = file_name
        name = '/home/evichan/Desktop/2024-tfg-eva-fernandez/GreenNao/nao_movement_pattern_creator/movements/' + self.file_name

        with open(name, 'r') as file:
            # Comprobar formato del fichero
            if self.file_name.endswith(".csv"):
                reader = csv.DictReader(file)
                self.datos = list(reader)

            elif self.file_name.endswith(".json"):
                self.datos = json.load(file)
            
            else: 
                print("ERROR: Formato de archivo no válido, por favor, utilice .json o .csv")
                sys.exit(1)

        self.art_publishers = {}
        tiempo_anterior = 0
        counter = 0

        # Crear publicadores para cada articulación, dependiendo del formato del fichero
        if self.file_name.endswith(".csv"):
            for fotograma in self.datos:
                counter = counter + 1
                tiempo_actual = float(fotograma["#WEBOTS_MOTION"])
                fotograma["tiempo_de_duracion"] = tiempo_actual - tiempo_anterior
                tiempo_anterior = tiempo_actual
                
                if counter == 1:
                    for articulacion in fotograma:
                        if articulacion != "#WEBOTS_MOTION" and articulacion != "V1.0":
                            self.art_publishers[articulacion] = self.create_publisher(Float64, f'/{articulacion}/cmd_pos', qos_profile)
        
        else:
            for fotograma in self.datos:
                counter = counter + 1
                tiempo_actual = fotograma["tiempo"]
                fotograma["tiempo_de_espera"] = tiempo_actual - tiempo_anterior
                tiempo_anterior = tiempo_actual
                
                if counter == 1:
                    for articulacion in fotograma["articulaciones"]:
                        nombre = articulacion["articulacion"]
                        self.art_publishers[nombre] = self.create_publisher(Float64, f'/{nombre}/cmd_pos', qos_profile)

        self.publish_message()

    def interpolate(self, start_value, end_value, t, duration):
        return start_value + (end_value - start_value) * 0.04

    def publish_message(self):
        if self.file_name.endswith(".csv"):
            num_fotogramas = len(self.datos)

            for i in range(num_fotogramas - 1):
                fotograma_actual = self.datos[i]
                fotograma_siguiente = self.datos[i + 1]
                duracion = float(fotograma_siguiente["tiempo_de_duracion"])
                
                time.sleep(duracion)

                for articulacion in fotograma_actual:
                    if articulacion != "#WEBOTS_MOTION" and articulacion != "V1.0":
                        pos_actual = float(fotograma_actual[articulacion])
                        pos_siguiente = float(fotograma_siguiente[articulacion])
                        interpolated_value = self.interpolate(pos_actual, pos_siguiente, duracion, duracion)
                        
                        msg = Float64()
                        msg.data = interpolated_value
                        self.art_publishers[articulacion].publish(msg)
                        time.sleep(0.001)
        else:
            msg = Float64()
            for fotograma in self.datos:
                tiempo = fotograma["tiempo_de_espera"]

                time.sleep(tiempo)
            
                for articulacion in fotograma["articulaciones"]:
                    nombre = articulacion["articulacion"]
                    msg.data = articulacion["posicion"]
                    self.art_publishers[nombre].publish(msg)
                    time.sleep(0.001)
                    
        self.get_logger().info("Fichero Completado")

# Clase para leer el IMU --------------------------------------------------------------------------------
class Read_IMU(Node):
    def __init__(self):
        super().__init__('wakeup')
        self.subscription = self.create_subscription(
            Imu,
            '/NAO/imu_sensor',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        # Medición de aceleración en los tres ejes)
        acc_x = msg.linear_acceleration.x
        acc_y = msg.linear_acceleration.y
        acc_z = msg.linear_acceleration.z

        # Normalizar el vector de aceleración para obtener la dirección de la gravedad
        acceleration_magnitude = (acc_x**2 + acc_y**2 + acc_z**2) ** 0.5
        if acceleration_magnitude == 0:
            return  # No se puede hacer nada si no hay aceleración

        # Dirección de la gravedad
        acc_x /= acceleration_magnitude
        acc_y /= acceleration_magnitude
        acc_z /= acceleration_magnitude
        
        self.acc_z = acc_z
    
    def get_z(self):
        return self.acc_z


# Función para obtener el dato del imu necesario para saber si estamos boca arriba o boca abajo ---------
def get_face():
    node = Read_IMU()
    rclpy.spin_once(node)
    z = node.get_z()
    if z > 0.9:
        side = "face normal"

    elif z < -0.1:
        side = "face up"
        
    elif z > 0.2:
        side = "face down"

    else:
        side = "ERROR"
    
    node.destroy_node()
    
    return side

# Funciones para reproducir patrones fijos de movimiento ------------------------------------------------
def wakeup_face_down():
    Interpreter("cubito_prono.csv")
    Interpreter("stand.json")
    
def wakeup_face_up():
    Interpreter("cubito_supino.json")
    time.sleep(1)
    Interpreter("cubito_prono.csv")
    Interpreter("stand.json")

def stand_still():
    Interpreter("stand.json")    

def say_hi(hand):
    if hand == "L" or hand == "left" or hand == "LEFT":
        Interpreter("say_hi_L.json")
    
    elif hand == "R" or hand == "right" or hand == "RIGHT":
        Interpreter("say_hi_R.json")
    
    else:
        print(f"ERROR: Indique correctamente la mano.\nNao solo tiene mano izquierda (L,LEFT,left) y derecha (R, RIGHT, right)")

def side_step(side, steps):
    if not isinstance(steps, int) or (steps < 2):
        print("Error: Indique un número de pasos válido, el número mínimo es 2")
    
    else:
        reps = int(steps/2)
        if side == "L" or side == "left" or side == "LEFT":
            for i in range(reps):
                Interpreter("side_step_left.csv")
        
        elif side == "R" or side == "right" or side == "RIGHT":
            for i in range(reps):
                Interpreter("side_step_right.csv")
        
        else:
            print(f"ERROR: Indique correctamente si izquierda (L,LEFT,left) o derecha (R, RIGHT, right), el número de pasos adecuado")

def turn(side, degrees):
    if (side == "L" or side == "left" or side == "LEFT") and (degrees == 40 or degrees == 60 or degrees == 180):
        deg = str(degrees)
        file = "turn_left_" + deg + ".csv"
        Interpreter(file)
    
    elif side == "R" or side == "right" or side == "RIGHT" and (degrees == 40 or degrees == 60):
        deg = str(degrees)
        file = "turn_right_" + deg + ".csv"
        Interpreter(file)
    
    else:
         print(f"ERROR: Indique correctamente si izquierda (L,LEFT,left) o derecha (R, RIGHT, right) y los grados (40, 60 y 180(solo izquierda))")

# Clase para andar recto pasando la velocidad -----------------------------------------------------------
class setV(Node):
    def __init__(self, linear_velocity: float, steps: int = 10):
        super().__init__('setv')
        
        if not ((0.35 <= abs(linear_velocity) <= 4.35) or abs(linear_velocity) == 0) or not (10 <= steps):
            print("ERROR: La velocidad lineal debe tomar un valor de entre ±0.35 y ±4.35 (aunque también puede coger 0).\nTenga en cuenta también que el mínimo de pasos (parámetro opcional) es 10, si no quiere andar, pase velocidad 0")
            sys.exit(1)
        else:
            self.V = linear_velocity
            self.steps = steps

        # Crear calidad e de servicio
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_ALL,
            depth=100
        )
        
        self.art_publishers = {}
        tiempo_anterior = 0
        counter = 0

        if self.V != 0:
            name = '/home/evichan/Desktop/2024-tfg-eva-fernandez/GreenNao/nao_movement_pattern_creator/movements/walk_forwards.csv'
            
            if self.V < 0:
                name = '/home/evichan/Desktop/2024-tfg-eva-fernandez/GreenNao/nao_movement_pattern_creator/movements/walk_backwards.csv'

            with open(name, 'r') as file:
                reader = csv.DictReader(file)
                self.datos = list(reader)

            # Crear publicadores para cada articulación
            for fotograma in self.datos:
                counter = counter + 1
                tiempo_actual = float(fotograma["#WEBOTS_MOTION"]) / abs(self.V)
                fotograma["tiempo_de_duracion"] = tiempo_actual - tiempo_anterior
                tiempo_anterior = tiempo_actual
                    
                if counter == 1:
                    for articulacion in fotograma:
                        if articulacion != "#WEBOTS_MOTION" and articulacion != "V1.0":
                            self.art_publishers[articulacion] = self.create_publisher(Float64, f'/{articulacion}/cmd_pos', qos_profile)
            
            self.publish_message()

        else:
            stand_still()
        
    def interpolate(self, start_value, end_value, t, duration):
        return start_value + (end_value - start_value) * 0.04

    def publish_message(self):
        reps = int(self.steps/10)
        
        if self.V < 0:
            reps = reps*5

        num_fotogramas = len(self.datos)

        for j in range(reps):
            for i in range(num_fotogramas - 1):
                fotograma_actual = self.datos[i]
                fotograma_siguiente = self.datos[i + 1]
                duracion = float(fotograma_siguiente["tiempo_de_duracion"])
                
                time.sleep(duracion)

                for articulacion in fotograma_actual:
                    if articulacion != "#WEBOTS_MOTION" and articulacion != "V1.0":
                        pos_actual = float(fotograma_actual[articulacion])
                        pos_siguiente = float(fotograma_siguiente[articulacion])
                        interpolated_value = self.interpolate(pos_actual, pos_siguiente, duracion, duracion)
                        
                        msg = Float64()
                        msg.data = interpolated_value
                        self.art_publishers[articulacion].publish(msg)
                        time.sleep(0.001)

        self.get_logger().info("Pasos completados")

# Clase para andar en arco pasando la velocidad -----------------------------------------------------------
class setW(Node):
    def __init__(self, angular_velocity: float, steps: int = 10):
        super().__init__('setw')
        
        if not ((0.35 <= abs(angular_velocity) <= 4.35) or abs(angular_velocity) == 0) or not (10 <= steps):
            print("ERROR: La velocidad angular debe tomar un valor de entre ±0.35 y ±1.9 (aunque también puede coger 0).\nTenga en cuenta también que el mínimo de pasos (parámetro opcional) es 10, si no quiere andar, pase velocidad 0")
            sys.exit(1)
        else:
            self.W = angular_velocity
            self.steps = steps
        
        # Crear calidad e de servicio
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_ALL,
            depth=100
        )
        
        self.art_publishers = {}
        tiempo_anterior = 0
        counter = 0

        if self.W != 0:
            name = '/home/evichan/Desktop/2024-tfg-eva-fernandez/GreenNao/nao_movement_pattern_creator/movements/walk_arc_r.csv'
            
            if self.W < 0:
                name = '/home/evichan/Desktop/2024-tfg-eva-fernandez/GreenNao/nao_movement_pattern_creator/movements/walk_arc_l.csv'
            
            with open(name, 'r') as file:
                reader = csv.DictReader(file)
                self.datos = list(reader)

            # Crear publicadores para cada articulación
            for fotograma in self.datos:
                counter = counter + 1
                tiempo_actual = float(fotograma["#WEBOTS_MOTION"]) / abs(self.W)
                fotograma["tiempo_de_duracion"] = tiempo_actual - tiempo_anterior
                tiempo_anterior = tiempo_actual
                    
                if counter == 1:
                    for articulacion in fotograma:
                        if articulacion != "#WEBOTS_MOTION" and articulacion != "V1.0":
                            self.art_publishers[articulacion] = self.create_publisher(Float64, f'/{articulacion}/cmd_pos', qos_profile)
            
            self.publish_message()

        else:
            stand_still()
        
    def interpolate(self, start_value, end_value, t, duration):
        return start_value + (end_value - start_value) * 0.04

    def publish_message(self):
        reps = int(self.steps/10)

        num_fotogramas = len(self.datos)

        for j in range(reps):
            for i in range(num_fotogramas - 1):
                fotograma_actual = self.datos[i]
                fotograma_siguiente = self.datos[i + 1]
                duracion = float(fotograma_siguiente["tiempo_de_duracion"])
                
                time.sleep(duracion)

                for articulacion in fotograma_actual:
                    if articulacion != "#WEBOTS_MOTION" and articulacion != "V1.0":
                        pos_actual = float(fotograma_actual[articulacion])
                        pos_siguiente = float(fotograma_siguiente[articulacion])
                        interpolated_value = self.interpolate(pos_actual, pos_siguiente, duracion, duracion)
                        
                        msg = Float64()
                        msg.data = interpolated_value
                        self.art_publishers[articulacion].publish(msg)
                        time.sleep(0.001)

        self.get_logger().info("Pasos completados")
