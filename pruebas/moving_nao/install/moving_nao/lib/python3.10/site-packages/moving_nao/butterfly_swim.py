import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class ButterflySwim(Node):
    def __init__(self):
        super().__init__('butterfly_swim')
        self.LShoulderPitch_publisher_ = self.create_publisher(Float64, '/LShoulderPitch/cmd_pos', 10)
        self.RShoulderPitch_publisher_ = self.create_publisher(Float64, '/RShoulderPitch/cmd_pos', 10)
        
        # Inicializar un temporizador para publicar mensajes cada segundo
        self.timer = self.create_timer(1.0, self.publish_message)
        
    def publish_message(self):
        msg = Float64()
        msg2 = Float64()

        msg1 = 1.0
        msg2 = -1.0   

        self.LShoulderPitch_publisher_.publish(msg)
        self.RShoulderPitch_publisher_.publish(msg2)
        
        self.get_logger().info('Publishing: "%f" in topic "%s"' % (msg.data, self.publisher_.topic_name))
        
def main(args=None):
    rclpy.init(args=args)
    butterfly_swim = ButterflySwim()
    rclpy.spin(butterfly_swim)

if __name__ == '__main__':
    main()

