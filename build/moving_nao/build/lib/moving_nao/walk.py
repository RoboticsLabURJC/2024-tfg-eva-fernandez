import sys
import json
import rclpy
import time
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64

class Move(Node):
    def __init__(self):
        super().__init__('move')
        
        # Leer argumentos y fichero
        if len(sys.argv) != 4:
            print("Por favor, pase exactamente 3 argumentos PASOS AMPLITUD TIEMPO_ENTRE_FOTOGRAMAS.")
            sys.exit(1)

        self.reps = int(sys.argv[1])
        self.ampli = float(sys.argv[2])
        self.tiempo = float(sys.argv[3])
        
        name = '/home/evichan/Desktop/2024-tfg-eva-fernandez/pruebas/moving_nao/nao_movement_pattern_creator/camina.json'
        
        with open(name, 'r') as file:
            self.datos = json.load(file)
        
        self.art_publishers = {}
        
        # Crear un publicador para cada articulación
        for fotograma in self.datos:
            for articulacion in fotograma["articulaciones"]:
                nombre = articulacion["articulacion"]
                self.art_publishers[nombre] = self.create_publisher(Float64, f'/{nombre}/cmd_pos', 10)
        
        self.publish_message()
        
    def publish_message(self):
        msg = Float64()
        step_time = self.tiempo 
        step_amplitude = self.ampli
        step_frequency = 2 * np.pi / (self.reps * 2)  # Frecuencia para una oscilación suave
        i = 0
        for repetition in range(self.reps):
            for idx, fotograma in enumerate(self.datos):
                i = i + 1
                time.sleep(step_time)
            
                phase = idx * step_frequency  # Determinar fase de la onda
                sinusoidal_offset = step_amplitude * np.sin(phase)  # Desplazamiento sinusoidal
            
                for articulacion in fotograma["articulaciones"]:
                    nombre = articulacion["articulacion"]
                    base_position = articulacion["posicion"]
                
                    # Alternar entre piernas correctamente
                    if "RHipPitch" in nombre or "RKneePitch" in nombre or "RAnklePitch" in nombre:
                        msg.data = base_position + sinusoidal_offset
                    elif "LHipPitch" in nombre or "LKneePitch" in nombre or "LAnklePitch" in nombre:
                        msg.data = base_position - sinusoidal_offset  # Oposición de fase para la otra pierna
                    else:
                        msg.data = base_position  # Mantener otras articulaciones sin cambios
                
                    self.art_publishers[nombre].publish(msg)
            
                self.get_logger().info(f'Fotograma {i}')

        # Volver a posición de reposo
        self.stand_still()
        
    def stand_still(self):
        stop_name = '/home/2024-tfg-eva-fernandez/pruebas/moving_nao/nao_movement_pattern_creator/stand.json'
        
        with open(stop_name, 'r') as file:
            stand = json.load(file)
        
        msg = Float64()
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

