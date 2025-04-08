import sys
import csv
import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class GreenNaoLib(Node):
    def __init__(self):
        super().__init__('greennaolib')
        
        # Crear calidad e de servicio
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_ALL,
            depth=10
        )
        
        # Definir valor de la velocidad que se pasa como rgumento obliatorio
        if len(sys.argv) != 2:
            print("Por favor, pase exactamente un argumento para especificar velocidad.")
            sys.exit(1)

        self.velocity = float(sys.argv[1])
        
        name = '/home/evichan/Desktop/2024-tfg-eva-fernandez/GreenNao/nao_movement_pattern_creator/movements/walk_forwards.csv'

        with open(name, 'r') as file:
            reader = csv.DictReader(file)
            self.datos = list(reader)

        self.art_publishers = {}
        tiempo_anterior = 0
        counter = 0

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
        
        self.publish_message()

    def interpolate(self, start_value, end_value, t, duration):
        return start_value + (end_value - start_value) * 0.04

    def publish_message(self):
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

        self.get_logger().info("Movimientos completados")

def main(args=None):
    rclpy.init(args=args)
    node = GreenNaoLib()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

