import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math

class NaoWalker(Node):
    def __init__(self):
        super().__init__('nao_walker')
        
        # Creación de publicadores para cada articulación
        self.joint_publishers = {
            'LHipPitch': self.create_publisher(Float64, '/nao/left_hip_pitch', 10),
            'LKneePitch': self.create_publisher(Float64, '/nao/left_knee_pitch', 10),
            'LAnklePitch': self.create_publisher(Float64, '/nao/left_ankle_pitch', 10),
            'RHipPitch': self.create_publisher(Float64, '/nao/right_hip_pitch', 10),
            'RKneePitch': self.create_publisher(Float64, '/nao/right_knee_pitch', 10),
            'RAnklePitch': self.create_publisher(Float64, '/nao/right_ankle_pitch', 10),
        }
        
        self.timer = self.create_timer(0.1, self.walk_step)  # Control a 10 Hz
        self.time_step = 0.0  # Contador de tiempo

    def walk_step(self):
        # Parámetros del patrón de marcha
        amplitude = 0.5
        frequency = 1.0  # Frecuencia de oscilación
        offset = 0.2
        
        # Calcula las posiciones de las articulaciones usando senoidales
        left_hip = amplitude * math.sin(2 * math.pi * frequency * self.time_step)
        left_knee = offset + amplitude * math.sin(2 * math.pi * frequency * self.time_step)
        left_ankle = -amplitude * math.sin(2 * math.pi * frequency * self.time_step)
        
        right_hip = amplitude * math.sin(2 * math.pi * frequency * (self.time_step + 0.5))
        right_knee = offset + amplitude * math.sin(2 * math.pi * frequency * (self.time_step + 0.5))
        right_ankle = -amplitude * math.sin(2 * math.pi * frequency * (self.time_step + 0.5))
        
        # Publica los valores calculados
        self.joint_publishers['LHipPitch'].publish(Float64(data=left_hip))
        self.joint_publishers['LKneePitch'].publish(Float64(data=left_knee))
        self.joint_publishers['LAnklePitch'].publish(Float64(data=left_ankle))
        
        self.joint_publishers['RHipPitch'].publish(Float64(data=right_hip))
        self.joint_publishers['RKneePitch'].publish(Float64(data=right_knee))
        self.joint_publishers['RAnklePitch'].publish(Float64(data=right_ankle))
        
        # Incrementa el tiempo
        self.time_step += 0.1

def main(args=None):
    rclpy.init(args=args)
    nao_walker = NaoWalker()
    rclpy.spin(nao_walker)
    nao_walker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

