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
            # Usar el valor de #WEBOTS_MOTION directamente si ya está en segundos
            tiempo_actual = float(fotograma["#WEBOTS_MOTION"])
            fotograma["tiempo_de_espera"] = tiempo_actual - tiempo_anterior
            tiempo_anterior = tiempo_actual

            for articulacion in fotograma:
                if articulacion != "#WEBOTS_MOTION" and articulacion != "V1.0":  # Excluimos las columnas no necesarias
                    self.art_publishers[articulacion] = self.create_publisher(Float64, f'/{articulacion}/cmd_pos', 10)

        self.publish_message()

    def publish_message(self):
        inicio_tiempo = time.time()  # Guardar el tiempo de inicio
        while True:
            msg = Float64()
            i = 0
            for fotograma in self.datos:
                tiempo_espera = float(fotograma["tiempo_de_espera"])  # Usamos el tiempo de espera calculado
                # Registrar el tiempo actual para comprobar la sincronización
                tiempo_inicio_fotograma = time.time()
                
                # Asegurarnos de que esperamos el tiempo correcto
                tiempo_trascorrido = time.time() - inicio_tiempo
                tiempo_deseado = i * tiempo_espera 
                tiempo_diferencia = tiempo_deseado - tiempo_trascorrido
                
                if tiempo_diferencia > 0:
                    time.sleep(tiempo_diferencia)  # Ajustamos la espera para cumplir los tiempos
                
                # Publicar las posiciones de las articulaciones
                for articulacion in fotograma:
                    if articulacion != "#WEBOTS_MOTION" and articulacion != "V1.0":  # Excluimos las columnas no necesarias
                        msg.data = float(fotograma[articulacion])
                        self.art_publishers[articulacion].publish(msg)

                i += 1
                self.get_logger().info(f'Fotograma {i} publicado, tiempo actual: {tiempo_trascorrido}s')
                time.sleep(0.5)

def main(args=None):    
    rclpy.init(args=args)
    node = Move()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
