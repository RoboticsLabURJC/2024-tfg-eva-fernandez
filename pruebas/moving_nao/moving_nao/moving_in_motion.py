import sys
import csv
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class Move(Node):
    def __init__(self):
        super().__init__('move')
        
        # Comprobar argumento (nombre del fichero a leer)
        if len(sys.argv) != 2:
            print("Por favor, pase exactamente un argumento.")
            sys.exit(1)

        file_name = sys.argv[1]
        name = '/home/2024-tfg-eva-fernandez/pruebas/moving_nao/nao_movement_pattern_creator/' + file_name

        # Leer el archivo CSV
        with open(name, 'r') as file:
            reader = csv.DictReader(file)  # Usamos DictReader para acceder a las columnas por nombre
            self.datos = list(reader)  # Convertir el objeto reader en una lista de diccionarios

        # Crear un diccionario para almacenar los publicadores de cada articulación
        self.art_publishers = {}
        tiempo_anterior = 0
        
        # Iterar sobre cada fotograma y crear un publicador para cada articulación
        for fotograma in self.datos:
            tiempo_actual = float(fotograma["#WEBOTS_MOTION"])  # Suponiendo que "#WEBOTS_MOTION" es la columna de tiempo
            fotograma["tiempo_de_espera"] = tiempo_actual - tiempo_anterior
            tiempo_anterior = tiempo_actual
            for articulacion in fotograma:
                if articulacion != "#WEBOTS_MOTION" and articulacion != "V1.0":  # Excluimos las columnas no necesarias
                    self.art_publishers[articulacion] = self.create_publisher(Float64, f'/{articulacion}/cmd_pos', 10)

        self.publish_message()

    def publish_message(self):
        while True:
            msg = Float64()
            i = 0
            for fotograma in self.datos:
                tiempo = float(fotograma["tiempo_de_espera"])  # Usamos el tiempo de espera calculado
                time.sleep(tiempo)  # Esperamos el tiempo antes de publicar
                i += 1

                # Publicar las posiciones de las articulaciones
                for articulacion in fotograma:
                    if articulacion != "#WEBOTS_MOTION" and articulacion != "V1.0":  # Excluimos las columnas no necesarias
                        msg.data = float(fotograma[articulacion])  # Convertimos la posición a float
                        self.art_publishers[articulacion].publish(msg)

                self.get_logger().info(f'Fotograma {i} publicado')

def main(args=None):    
    rclpy.init(args=args)
    node = Move()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
