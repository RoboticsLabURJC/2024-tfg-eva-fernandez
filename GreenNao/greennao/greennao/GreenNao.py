import GreenNaoLib
import rclpy

node = GreenNaoLib.WakeUp()
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()

