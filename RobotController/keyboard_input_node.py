import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import termios
import tty

class KeyboardInputNode(Node):
    """A node that captures keyboard input and publishes arrow key presses."""
    
    def __init__(self):
        super().__init__('keyboard_input_node')
        self.publisher_ = self.create_publisher(String, '/keyboard_inputs', 10)
        self.valid_keys = {
            '\x1b[A': 'up',    # Arrow up
            '\x1b[B': 'down',  # Arrow down
            '\x1b[C': 'right', # Arrow right
            '\x1b[D': 'left'   # Arrow left
        }
        self.get_logger().info('Keyboard Input Node started. Use arrow keys to control joints. Ctrl+C to exit.')

    def get_key(self):
        """Capture a single keypress, including escape sequences for arrow keys."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            # Read up to 3 bytes to capture escape sequences (e.g., arrow keys)
            key = sys.stdin.read(1)
            if key == '\x1b':  # Escape character
                key += sys.stdin.read(2)  # Read the next two characters for arrow keys
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        """Main loop to capture and publish keyboard inputs."""
        while rclpy.ok():
            key = self.get_key()
            if key == '\x03':  # Ctrl+C to exit
                break
            if key in self.valid_keys:
                msg = String()
                msg.data = self.valid_keys[key]
                self.publisher_.publish(msg)
                self.get_logger().info(f'Published key: {msg.data}')
            rclpy.spin_once(self, timeout_sec=0.1)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardInputNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()