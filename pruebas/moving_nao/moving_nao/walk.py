import sys
import json
import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Float64

class Move(Node):
    def __init__(self):
        super().__init__('move')
        
        # Comprobar argumento (repeticiones del paso)
        if len(sys.argv) != 2:
            print("Por favor, pase exactamente un argumento.")
            sys.exit(1)

        self.reps = int(sys.argv[1])
        
        name = '/home/2024-tfg-eva-fernandez/pruebas/moving_nao/nao_movement_pattern_creator/camina.json'
        
        with open(name, 'r') as file:
            self.datos = json.load(file)
        
        # Crear un diccionario para almacenar los publicadores de cada articulación
        self.art_publishers = {}
        tiempo_anterior = 0
        
        # Iterar sobre cada articulación y crear un publicador para cada una
        for fotograma in self.datos:
            for articulacion in fotograma["articulaciones"]:
                nombre = articulacion["articulacion"]
                self.art_publishers[nombre] = self.create_publisher(Float64, f'/{nombre}/cmd_pos', 10)
        
        self.publish_message()
        
    def publish_message(self):
        msg = Float64()
        i=0
        for repetition in range(0,self.reps):
            for fotograma in self.datos:
                time.sleep(0.3)
                i = i+1
                for articulacion in fotograma["articulaciones"]:
                    nombre = articulacion["articulacion"]
                    msg.data = articulacion["posicion"]
                    self.art_publishers[nombre].publish(msg)
                self.get_logger().info(f'Fotograma {i}')

        stop_name = '/home/2024-tfg-eva-fernandez/pruebas/moving_nao/nao_movement_pattern_creator/stand.json'
        
        with open(stop_name, 'r') as file:
            stand = json.load(file)
        
        for fotograma in stand:
            time.sleep(0.3)
            for articulacion in fotograma["articulaciones"]:
              nombre = articulacion["articulacion"]
              msg.data = articulacion["posicion"]
              self.art_publishers[nombre].publish(msg)
        
        self.get_logger().info(f'Finished walk')
        
            
def main(args=None):    
    rclpy.init(args=args)
    node = Move()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

