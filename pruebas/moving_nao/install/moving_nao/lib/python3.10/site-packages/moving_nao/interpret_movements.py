import sys
import csv
import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class Move(Node):
    def __init__(self):
        super().__init__('move')

        # Interpretar fichero que se pasa como único argumento obligatorio
        if len(sys.argv) != 2:
            print("Por favor, pase exactamente un argumento.")
            sys.exit(1)

        self.file_name = sys.argv[1]
        name = '/home/evichan/Desktop/2024-tfg-eva-fernandez/pruebas/moving_nao/nao_movement_pattern_creator/' + self.file_name

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

        # Crear publicadores para cada articulación, dependiendo del formato del fichero
        if self.file_name.endswith(".csv"):
            for fotograma in self.datos:
                tiempo_actual = float(fotograma["#WEBOTS_MOTION"])
                fotograma["tiempo_de_duracion"] = tiempo_actual - tiempo_anterior
                tiempo_anterior = tiempo_actual

                for articulacion in fotograma:
                    if articulacion != "#WEBOTS_MOTION" and articulacion != "V1.0":
                        self.art_publishers[articulacion] = self.create_publisher(Float64, f'/{articulacion}/cmd_pos', 10)
        
        else:
            for fotograma in self.datos:
                tiempo_actual = fotograma["tiempo"]
                fotograma["tiempo_de_espera"] = tiempo_actual - tiempo_anterior
                tiempo_anterior = tiempo_actual
                for articulacion in fotograma["articulaciones"]:
                    nombre = articulacion["articulacion"]
                    self.art_publishers[nombre] = self.create_publisher(Float64, f'/{nombre}/cmd_pos', 10)

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
        else:
            msg = Float64()
            i=0
            for fotograma in self.datos:
                tiempo = fotograma["tiempo_de_espera"]
                time.sleep(tiempo)
                i = i+1
            
                for articulacion in fotograma["articulaciones"]:
                    nombre = articulacion["articulacion"]
                    msg.data = articulacion["posicion"]
                    self.art_publishers[nombre].publish(msg)

        self.get_logger().info("Movimientos completados")

def main(args=None):
    rclpy.init(args=args)
    node = Move()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

