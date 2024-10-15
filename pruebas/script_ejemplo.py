import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import tkinter as tk

class JointControlNode(Node):
    def __init__(self):
        super().__init__('joint_control_node')
        
        # Publicador para cada joint que quieras controlar.
        self.joint1_publisher = self.create_publisher(Float64, '/joint1_position_controller/command', 10)
        self.joint2_publisher = self.create_publisher(Float64, '/joint2_position_controller/command', 10)
        
        # Configura la interfaz gráfica con Tkinter.
        self.root = tk.Tk()
        self.root.title("Joint Control")

        # Crea sliders para cada joint.
        self.joint1_slider = tk.Scale(self.root, from_=-3.14, to=3.14, resolution=0.01, orient='horizontal', command=self.update_joint1)
        self.joint1_slider.pack()
        self.joint1_slider.set(0.0)  # Ajusta la posición inicial.

        self.joint2_slider = tk.Scale(self.root, from_=-3.14, to=3.14, resolution=0.01, orient='horizontal', command=self.update_joint2)
        self.joint2_slider.pack()
        self.joint2_slider.set(0.0)  # Ajusta la posición inicial.

        # Ejecuta la interfaz gráfica en un bucle.
        self.root.mainloop()

    def update_joint1(self, value):
        # Publica el valor del slider al joint1.
        msg = Float64()
        msg.data = float(value)
        self.joint1_publisher.publish(msg)
        self.get_logger().info(f"Joint1 position set to {value}")

    def update_joint2(self, value):
        # Publica el valor del slider al joint2.
        msg = Float64()
        msg.data = float(value)
        self.joint2_publisher.publish(msg)
        self.get_logger().info(f"Joint2 position set to {value}")

def main(args=None):
    rclpy.init(args=args)
    node = JointControlNode()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

