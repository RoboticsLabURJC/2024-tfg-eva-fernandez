# greennao_lib/greennao.py

import csv
import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

rclpy.init()

class GreenNao(Node):
    def __init__(self, velocity: float, movement_path: str, stand_path: str):
        super().__init__('greennao_lib')
        self.velocity = velocity
        self.movement_path = '/home/evichan/Desktop/2024-tfg-eva-fernandez/GreenNao/nao_movement_pattern_creator/movements/walk_forwards.csv'
        self.stand_path = '/home/evichan/Desktop/2024-tfg-eva-fernandez/GreenNao/nao_movement_pattern_creator/movements/stand.json'
        self.art_publishers = {}

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_ALL,
            depth=10
        )

        if not ((0.35 <= self.velocity <= 4.35) or self.velocity == 0):
            raise ValueError("La velocidad debe ser 0 o entre 0.35 y 4.35")

        tiempo_anterior = 0
        counter = 0

        if self.velocity != 0:
            with open(self.movement_path, 'r') as file:
                reader = csv.DictReader(file)
                self.datos = list(reader)

            for fotograma in self.datos:
                counter += 1
                tiempo_actual = float(fotograma["#WEBOTS_MOTION"]) / self.velocity
                fotograma["tiempo_de_duracion"] = tiempo_actual - tiempo_anterior
                tiempo_anterior = tiempo_actual

                if counter == 1:
                    for articulacion in fotograma:
                        if articulacion not in ["#WEBOTS_MOTION", "V1.0"]:
                            self.art_publishers[articulacion] = self.create_publisher(Float64, f'/{articulacion}/cmd_pos', qos_profile)
        else:
            with open(self.stand_path, 'r') as file:
                self.datos = json.load(file)

            for fotograma in self.datos:
                counter += 1
                tiempo_actual = fotograma["tiempo"]
                fotograma["tiempo_de_espera"] = tiempo_actual - tiempo_anterior
                tiempo_anterior = tiempo_actual

                if counter == 1:
                    for articulacion in fotograma["articulaciones"]:
                        nombre = articulacion["articulacion"]
                        self.art_publishers[nombre] = self.create_publisher(Float64, f'/{nombre}/cmd_pos', qos_profile)

    def interpolate(self, start_value, end_value, t, duration):
        return start_value + (end_value - start_value) * 0.04

    def execute_movement(self):
        if self.velocity != 0:
            num_fotogramas = len(self.datos)

            for _ in range(3):
                for i in range(num_fotogramas - 1):
                    actual = self.datos[i]
                    siguiente = self.datos[i + 1]
                    duracion = float(siguiente["tiempo_de_duracion"])
                    time.sleep(duracion)

                    for art in actual:
                        if art not in ["#WEBOTS_MOTION", "V1.0"]:
                            pos_actual = float(actual[art])
                            pos_siguiente = float(siguiente[art])
                            interpolated = self.interpolate(pos_actual, pos_siguiente, duracion, duracion)

                            msg = Float64()
                            msg.data = interpolated
                            self.art_publishers[art].publish(msg)
                            time.sleep(0.001)
        else:
            msg = Float64()
            for fotograma in self.datos:
                time.sleep(fotograma["tiempo_de_espera"])
                for articulacion in fotograma["articulaciones"]:
                    nombre = articulacion["articulacion"]
                    msg.data = articulacion["posicion"]
                    self.art_publishers[nombre].publish(msg)
                    time.sleep(0.001)

        self.get_logger().info("Movimientos completados")

