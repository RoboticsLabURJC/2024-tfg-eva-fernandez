import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import threading
import json
from pynput import keyboard

class JointStateRecorder(Node):
    def __init__(self):
        super().__init__('joint_state_recorder')
        self.subscribers = {}
        self.joint_positions = {}

        # Iniciar escucha de teclado en un hilo separado
        threading.Thread(target=self.listen_for_key, daemon=True).start()

        # Revisar los tópicos cada 1s
        self.timer = self.create_timer(1.0, self.update_subscribers)

    def get_cmd_pos_topics(self):
        """ Encuentra todos los tópicos que terminan en '/cmd_pos'. """
        topics = self.get_topic_names_and_types()
        return [t[0] for t in topics if t[0].endswith('/cmd_pos')]

    def joint_callback(self, msg, topic_name):
        """ Guarda la posición de la articulación. """
        self.joint_positions[topic_name] = msg.data

    def update_subscribers(self):
        """ Suscribirse dinámicamente a nuevos tópicos de articulaciones. """
        topics = self.get_cmd_pos_topics()

        for topic in topics:
            if topic not in self.subscribers:
                self.subscribers[topic] = self.create_subscription(
                    Float64,
                    topic,
                    lambda msg, topic=topic: self.joint_callback(msg, topic),
                    10  # QoS default
                )
                self.get_logger().info(f"✅ Suscrito a {topic}")

    def listen_for_key(self):
        """ Escucha la tecla 'S' y guarda las posiciones al archivo. """
        def on_press(key):
            if key == keyboard.KeyCode.from_char('s'):
                self.save_positions()

        with keyboard.Listener(on_press=on_press) as listener:
            listener.join()

    def save_positions(self):
        """ Guarda las posiciones en un archivo JSON. """
        with open("posiciones.txt", "w") as f:
            json.dump(self.joint_positions, f, indent=2)
        self.get_logger().info("📁 Posiciones guardadas en posiciones.txt")

def main(args=None):
    rclpy.init(args=args)
    node = JointStateRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
