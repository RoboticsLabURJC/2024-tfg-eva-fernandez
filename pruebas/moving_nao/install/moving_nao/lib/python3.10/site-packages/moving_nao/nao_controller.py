import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Float64, '/HeadYaw/cmd_pos', 10)
        
        # Inicializar un temporizador para publicar mensajes cada 2 segundos
        self.timer = self.create_timer(2.0, self.publish_message)
        self.counter = 0  # Contador para alternar entre los mensajes

    def publish_message(self):
        msg = Float64()
        # Alternar entre los dos valores
        if self.counter % 2 == 0:
            msg.data = 0.5
        else:
            msg.data = -0.5
            
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%f" in topic "%s"' % (msg.data, self.publisher_.topic_name))
        
        self.counter += 1  # Incrementar el contador

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)

if __name__ == '__main__':
    main()

