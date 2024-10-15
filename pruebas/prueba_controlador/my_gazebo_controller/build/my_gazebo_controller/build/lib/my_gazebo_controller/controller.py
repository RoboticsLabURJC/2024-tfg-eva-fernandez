import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

class GazeboController(Node):
    def __init__(self):
        super().__init__('gazebo_controller')
        
        # Publisher for controlling joints
        self.joint_pub = self.create_publisher(ModelState, '/gazebo/set_model_state', 10)

        # Create a timer to control the simulation
        self.timer = self.create_timer(0.1, self.timer_callback)
        
    def timer_callback(self):
        # Create a ModelState message
        model_state = ModelState()
        model_state.model_name = 'your_model_name'  # Replace with your model name
        model_state.pose.position.x = 0.0  # Set desired position
        model_state.pose.position.y = 0.0
        model_state.pose.position.z = 0.0
        model_state.pose.orientation.x = 0.0
        model_state.pose.orientation.y = 0.0
        model_state.pose.orientation.z = 0.0
        model_state.pose.orientation.w = 1.0
        
        # Publish the message
        self.joint_pub.publish(model_state)
        self.get_logger().info('Controlling model joints.')

def main(args=None):
    rclpy.init(args=args)
    gazebo_controller = GazeboController()
    
    rclpy.spin(gazebo_controller)
    
    gazebo_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
