import sys
import csv
import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

rclpy.init()

# CLASE PARA ANDAR RECTO PASANDO LA VELOCIDAD --------------------------------------------------------------------------------
class SetV(Node):
    def __init__(self, velocity: float):
        super().__init__('setv')
        
        if not ((0.35 <= velocity <= 4.35) or velocity == 0):
            print("ERROR: La velocidad debe tomar un valor de entre 0.35 y 4.35 (aunque también puede coger 0).")
            sys.exit(1)
        else:
            self.velocity = velocity
        
        # Crear calidad e de servicio
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_ALL,
            depth=100
        )
        
        self.art_publishers = {}
        tiempo_anterior = 0
        counter = 0

        if self.velocity != 0:
            name = '/home/evichan/Desktop/2024-tfg-eva-fernandez/GreenNao/nao_movement_pattern_creator/movements/walk_forwards.csv'

            with open(name, 'r') as file:
                reader = csv.DictReader(file)
                self.datos = list(reader)

            # Crear publicadores para cada articulación
            for fotograma in self.datos:
                counter = counter + 1
                tiempo_actual = float(fotograma["#WEBOTS_MOTION"]) / self.velocity
                fotograma["tiempo_de_duracion"] = tiempo_actual - tiempo_anterior
                tiempo_anterior = tiempo_actual
                    
                if counter == 1:
                    for articulacion in fotograma:
                        if articulacion != "#WEBOTS_MOTION" and articulacion != "V1.0":
                            self.art_publishers[articulacion] = self.create_publisher(Float64, f'/{articulacion}/cmd_pos', qos_profile)
        else:
            name = '/home/evichan/Desktop/2024-tfg-eva-fernandez/GreenNao/nao_movement_pattern_creator/movements/stand.json'
            with open(name, 'r') as file:
                self.datos = json.load(file)
            
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
        if self.velocity != 0:
            num_fotogramas = len(self.datos)

            for j in range(3):
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

        self.get_logger().info("Pasos completados")

# CLASE PARA REPLICAR LOS MOVIMIENTOS DE UN FICHERO --------------------------------------------------------------------------------
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
                    
        self.get_logger().info("Fichero completado")


