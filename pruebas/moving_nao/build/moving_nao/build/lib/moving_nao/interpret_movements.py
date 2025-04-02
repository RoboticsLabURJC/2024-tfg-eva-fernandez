import sys
import csv
import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class Move(Node):
    def __init__(self):
        super().__init__('move')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_ALL,
            depth=10
        )

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
                    
        self.get_logger().info("Movimientos completados")

def main(args=None):
    rclpy.init(args=args)
    node = Move()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# import sys
# import csv
# import json
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float64
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# class Move(Node):
#     def __init__(self):
#         super().__init__('move')

#         # Configuración del QoS (ajustado)
#         qos_profile = QoSProfile(
#             reliability=ReliabilityPolicy.RELIABLE,  # Más eficiente que RELIABLE si hay muchos mensajes
#             history=HistoryPolicy.KEEP_LAST,
#             depth=10  # Evita uso innecesario de memoria
#         )

#         # Leer argumento del archivo de movimientos
#         if len(sys.argv) != 2:
#             self.get_logger().error("Por favor, pase exactamente un argumento (archivo .csv o .json).")
#             sys.exit(1)

#         self.file_name = sys.argv[1]
#         path = f'/home/evichan/Desktop/2024-tfg-eva-fernandez/pruebas/moving_nao/nao_movement_pattern_creator/{self.file_name}'

#         with open(path, 'r') as file:
#             if self.file_name.endswith(".csv"):
#                 reader = csv.DictReader(file)
#                 self.datos = list(reader)
#             elif self.file_name.endswith(".json"):
#                 self.datos = json.load(file)
#             else: 
#                 self.get_logger().error("Formato de archivo no válido, use .json o .csv")
#                 sys.exit(1)

#         self.art_publishers = {}
#         self.movement_sequence = []  # Lista de movimientos con tiempo calculado
#         self.current_frame = 0

#         # Crear publishers solo una vez
#         if self.file_name.endswith(".csv"):
#             for key in self.datos[0]:
#                 if key not in ["#WEBOTS_MOTION", "V1.0"]:
#                     self.art_publishers[key] = self.create_publisher(Float64, f'/{key}/cmd_pos', qos_profile)

#             # Procesar datos de CSV
#             prev_time = 0.0
#             for frame in self.datos:
#                 current_time = float(frame["#WEBOTS_MOTION"])
#                 duration = current_time - prev_time
#                 prev_time = current_time
#                 self.movement_sequence.append((duration, frame))

#         else:  # Formato JSON
#             for joint in self.datos[0]["articulaciones"]:
#                 self.art_publishers[joint["articulacion"]] = self.create_publisher(Float64, f'/{joint["articulacion"]}/cmd_pos', qos_profile)

#             # Procesar datos de JSON
#             prev_time = 0.0
#             for frame in self.datos:
#                 current_time = frame["tiempo"]
#                 duration = current_time - prev_time
#                 prev_time = current_time
#                 self.movement_sequence.append((duration, frame))

#         # Iniciar la publicación con un temporizador
#         self.timer = self.create_timer(0.01, self.publish_message)  # Ejecuta cada 10ms

#     def publish_message(self):
#         """Publica las posiciones de las articulaciones en intervalos de tiempo predefinidos"""
#         if self.current_frame >= len(self.movement_sequence):
#             self.get_logger().info("Movimientos completados")
#             self.timer.cancel()  # Detiene el temporizador cuando terminan los movimientos
#             return

#         duration, frame = self.movement_sequence[self.current_frame]

#         if self.file_name.endswith(".csv"):
#             for joint, value in frame.items():
#                 if joint not in ["#WEBOTS_MOTION", "V1.0"]:
#                     msg = Float64()
#                     msg.data = float(value)
#                     self.art_publishers[joint].publish(msg)

#         else:
#             for joint in frame["articulaciones"]:
#                 msg = Float64()
#                 msg.data = joint["posicion"]
#                 self.art_publishers[joint["articulacion"]].publish(msg)

#         self.current_frame += 1
#         self.timer.reset()  # Reinicia el temporizador para esperar el siguiente frame

# def main(args=None):
#     rclpy.init(args=args)
#     node = Move()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
