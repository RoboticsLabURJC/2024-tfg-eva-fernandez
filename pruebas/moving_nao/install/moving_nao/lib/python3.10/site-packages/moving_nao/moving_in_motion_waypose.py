import sys
import csv
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class Move(Node):
    def __init__(self):
        super().__init__('move')

        if len(sys.argv) != 2:
            print("Por favor, pase exactamente un argumento.")
            sys.exit(1)

        file_name = sys.argv[1]
        name = '/home/evichan/Desktop/2024-tfg-eva-fernandez/pruebas/moving_nao/nao_movement_pattern_creator/' + file_name

        with open(name, 'r') as file:
            reader = csv.DictReader(file)
            self.datos = list(reader)

        self.art_publishers = {}
        tiempo_anterior = 0

        # Crear publicadores para cada articulación
        for fotograma in self.datos:
            tiempo_actual = float(fotograma["#WEBOTS_MOTION"])
            fotograma["tiempo_de_duracion"] = tiempo_actual - tiempo_anterior
            tiempo_anterior = tiempo_actual

            for articulacion in fotograma:
                if articulacion != "#WEBOTS_MOTION" and articulacion != "V1.0":
                    self.art_publishers[articulacion] = self.create_publisher(Float64, f'/{articulacion}/cmd_pos', 10)

        self.publish_message()

    def interpolate(self, start_value, end_value, t, duration):
        return start_value + (end_value - start_value) * 0.04

    def publish_message(self):
        initial_time = time.time()
        num_fotogramas = len(self.datos)

        for i in range(num_fotogramas - 1):
            fotograma_actual = self.datos[i]
            fotograma_siguiente = self.datos[i + 1]
            duracion = float(fotograma_siguiente["tiempo_de_duracion"])

            start_time = time.time()
            while time.time() - start_time < duracion:
                t_actual = time.time() - start_time
                for articulacion in fotograma_actual:
                    if articulacion != "#WEBOTS_MOTION" and articulacion != "V1.0":
                        pos_actual = float(fotograma_actual[articulacion])
                        pos_siguiente = float(fotograma_siguiente[articulacion])
                        interpolated_value = self.interpolate(pos_actual, pos_siguiente, t_actual, duracion)

                        msg = Float64()
                        msg.data = interpolated_value
                        self.art_publishers[articulacion].publish(msg)
                
                time.sleep(0.1)
        
        self.get_logger().info("Movimientos completados")

def main(args=None):
    rclpy.init(args=args)
    node = Move()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

