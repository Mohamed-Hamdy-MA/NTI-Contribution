import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math

"""A node that received keyboard inputs to control joint positions and enforces limits."""
class JointStateControllerNode(Node):
    
    """"Initializes everything the object needs to work. It automatically runs when you create an object from a class."""
    def __init__(self):
        super().__init__('joint_state_controller_node')

        # Subscriber for controller inputs
        self.subscriber_ = self.create_subscription(String, '/keyboard_inputs', self.input_callback, 10)
        # Publisher for joint states
        self.publisher_ = self.create_publisher(JointState, '/robot_joints', 10)
        
        # Joint configuration
        self.joint_names = [
            'base_rotation_joint',
            'lower_arm_joint',
            'upper_arm_joint',
            'gripper_support_joint',
            'right_gripper_joint',
            'left_gripper_joint'
        ]
        # Joint limits (in radians, based on the URDF)
        self.joint_limits = {
            'base_rotation_joint': (-math.pi/2, math.pi/2),  # -90° to 90°
            'lower_arm_joint': (-math.pi/2, math.pi/2),      # -90° to 90°
            'upper_arm_joint': (-math.pi/2, math.pi/2),      # -90° to 90°
            'gripper_support_joint': (-math.pi/2, math.pi/2),# -90° to 90°
            'right_gripper_joint': (-math.pi/2, 0.0),        # -90° to 0°
            'left_gripper_joint': (0.0, math.pi/2)           # 0° to 90°
        }
        self.positions = [0.0] * len(self.joint_names)
        self.selected_joint = 0  # Index of the currently selected joint

        self.get_logger().info('Joint State Controller Node started.')

    """Ensure the position within the joint's limits."""
    def enforce_joint_limits(self, joint_name, position):
        lower, upper = self.joint_limits[joint_name]
        return max(lower, min(upper, position))

    """Callbacks when keyboard input recevied then update joint positions."""
    def input_callback(self, msg):
        key = msg.data
        joint_name = self.joint_names[self.selected_joint]
        
        # Interpret arrow keys
        if key == 'down':
            self.selected_joint = (self.selected_joint - 1) % len(self.joint_names)
        elif key == 'up':
            self.selected_joint = (self.selected_joint + 1) % len(self.joint_names)
        elif key == 'right':
            self.positions[self.selected_joint] += 0.1
        elif key == 'left':
            self.positions[self.selected_joint] -= 0.1
        
        # Enforce joint limits
        self.positions[self.selected_joint] = self.enforce_joint_limits(
            joint_name, self.positions[self.selected_joint])
        
        self.get_logger().info(
            f'Selected joint: {joint_name}, Position: {self.positions[self.selected_joint]:.2f}')
        self.publish_joint_state()

    """Publish the current joint positions as a JointState message."""
    def publish_joint_state(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.name = self.joint_names
        msg.position = self.positions
        msg.velocity = []
        msg.effort = []
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointStateControllerNode()
    
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()