import json
import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Float64

class Move(Node):
    def __init__(self):
        super().__init__('move')
        
        with open('/home/2024-tfg-eva-fernandez/pruebas/moving_nao/nao_movement_pattern_creator/movement_pattern.json', 'r') as file:
            self.datos = json.load(file)
        
        # Crear un diccionario para almacenar los publicadores de cada articulación
        self.art_publishers = {}
        
        # Iterar sobre cada articulación y crear un publicador para cada una
        for fotograma in self.datos:
            for articulacion in fotograma["articulaciones"]:
                nombre = articulacion["articulacion"]
                self.art_publishers[nombre] = self.create_publisher(Float64, f'/{nombre}/cmd_pos', 10)
        
        self.publish_message()
        
    def publish_message(self):
        msg = Float64()
       
        for fotograma in self.datos:
            tiempo = fotograma["tiempo"]
            time.sleep(tiempo)
            for articulacion in fotograma["articulaciones"]:
                nombre = articulacion["articulacion"]
                msg.data = articulacion["posicion"]
                self.art_publishers[nombre].publish(msg)
                self.get_logger().info(f'Publicado en tiempo: {tiempo}')
        
def main(args=None):    
    rclpy.init(args=args)
    node = Move()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
