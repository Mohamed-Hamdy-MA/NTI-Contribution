import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

"""A node that publishes joint states for RViz visualization."""
class JointStatePublisherNode(Node):

    """Initializes everything the object needs to work. It automatically runs when you create an object from a class."""
    def __init__(self):
        super().__init__('joint_state_publisher_node')

        # Subscriber for joint states (From the robot joints controller)
        self.subscriber_ = self.create_subscription(JointState, '/robot_joints', self.joint_state_callback, 10)
        # Publisher for joint states (RViz Topic)
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)

        self.get_logger().info('Joint State Publisher Node started.')

    """Republish the JointState message to /joint_states RViz topic."""
    def joint_state_callback(self, msg):
        self.publisher_.publish(msg)
        self.get_logger().info('Republished joint states to /joint_states RViz topic.')

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisherNode()
    
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()