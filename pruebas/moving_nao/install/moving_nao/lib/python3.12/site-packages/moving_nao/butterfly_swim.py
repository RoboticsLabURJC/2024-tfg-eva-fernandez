import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class ButterflySwim(Node):
    def __init__(self):
        super().__init__('butterfly_swim')
        self.LShoulderPitch_publisher_ = self.create_publisher(Float64, '/LShoulderPitch/cmd_pos', 10)
        self.RShoulderPitch_publisher_ = self.create_publisher(Float64, '/RShoulderPitch/cmd_pos', 10)
        
        # Inicializar un temporizador para publicar mensajes cada 0.02 segundos
        self.timer = self.create_timer(0.02, self.publish_message)
        
        # Ángulo incial e incremento con el tiempo
        self.angle = 0.0
        self.increment = 0.1
        
    def publish_message(self):
        msg = Float64()

        # Actualizar el ángulo para crear un giro continuo y publicar para que se mueva
        self.angle += self.increment

        msg.data = self.angle

        self.LShoulderPitch_publisher_.publish(msg)
        self.RShoulderPitch_publisher_.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    butterfly_swim = ButterflySwim()
    rclpy.spin(butterfly_swim)

if __name__ == '__main__':
    main()

