import sys
import csv
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile

class Move(Node):
    def __init__(self):
        super().__init__('move')

        # Comprobar argumento (nombre del fichero a leer)
        if len(sys.argv) != 2:
            print("Por favor, pase exactamente un argumento.")
            sys.exit(1)

        file_name = sys.argv[1]
        name = '/home/evichan/Desktop/2024-tfg-eva-fernandez/pruebas/moving_nao/nao_movement_pattern_creator/' + file_name

        # Leer el archivo CSV
        with open(name, 'r') as file:
            reader = csv.DictReader(file)  # Usamos DictReader para acceder a las columnas por nombre
            self.datos = list(reader)  # Convertir el objeto reader en una lista de diccionarios

        # Crear un diccionario para almacenar los publicadores de cada articulación
        self.art_publishers = {}
        self.art_subscribers = {}
        self.art_positions = {}

        # Crear publicadores y suscriptores para cada articulación
        for fotograma in self.datos:
            for articulacion in fotograma:
                if articulacion != "#WEBOTS_MOTION" and articulacion != "V1.0":  # Excluimos las columnas no necesarias
                    self.art_publishers[articulacion] = self.create_publisher(Float64, f'/{articulacion}/cmd_pos', 10)
                    self.art_positions[articulacion] = None  # Inicializar el valor de la posición

                    # Crear suscriptor para escuchar la posición de cada articulación
                    self.art_subscribers[articulacion] = self.create_subscription(
                        Float64, f'/{articulacion}/cmd_pos', self.position_callback(articulacion), QoSProfile(depth=10)
                    )

        self.publish_message()

    def position_callback(self, articulacion):
        """Función callback para actualizar la posición de la articulación"""
        def callback(msg):
            self.art_positions[articulacion] = msg.data  # Actualizar la posición recibida
        return callback

    def wait_for_position(self, articulacion, target_position):
        """Esperar hasta que la articulación llegue a la posición deseada"""
        while self.art_positions[articulacion] is None or abs(self.art_positions[articulacion] - target_position) > 0.01:  # Tolerancia de 0.01
            self.get_logger().info(f'Esperando que {articulacion} llegue a {target_position}...')
            time.sleep(0.1)  # Esperar un poco antes de comprobar de nuevo
        self.get_logger().info(f'{articulacion} alcanzó la posición {target_position}!')

    def publish_message(self):
        while True:
            msg = Float64()
            i = 0
            for fotograma in self.datos:                
                # Publicar las posiciones de las articulaciones y esperar a que lleguen a la posición
                for articulacion in fotograma:
                    if articulacion != "#WEBOTS_MOTION" and articulacion != "V1.0":  # Excluimos las columnas no necesarias
                        target_position = float(fotograma[articulacion])
                        msg.data = target_position
                        self.art_publishers[articulacion].publish(msg)

                        # Esperar a que la articulación llegue a la posición
                        self.wait_for_position(articulacion, target_position)

                i += 1
                self.get_logger().info(f'Fotograma {i} publicado')
                time.sleep(0.5)

def main(args=None):    
    rclpy.init(args=args)
    node = Move()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
