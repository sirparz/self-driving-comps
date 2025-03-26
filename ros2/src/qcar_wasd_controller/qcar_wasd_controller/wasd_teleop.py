import rclpy
from rclpy.node import Node
import sys, select, termios, tty
from qcar2_interfaces.msg import MotorCommands
# test 
# Key mappings
MOVE_BINDINGS = {
    'w': (0.0, 0.5),  # Forward
    's': (0.0, -0.5), # Reverse
    'a': (0.3, 0.25),  # Left turn
    'd': (-0.3, 0.25), # Right turn
    ' ': (0.0, 0.0),  # Stop
}

class WASDTeleop(Node):
    def __init__(self):
        super().__init__('wasd_teleop')
        self.publisher = self.create_publisher(MotorCommands, '/qcar2_motor_speed_cmd', 10)
        self.get_logger().info("WASD Teleop Node Started. Use 'WASD' to move, Space to stop.")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
        return key

    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                if key == '\x03':  # Ctrl+C
                    break
                
                if key in MOVE_BINDINGS:
                    steering, throttle = MOVE_BINDINGS[key]
                    msg = MotorCommands()
                    msg.motor_names = ["steering_angle", "motor_throttle"]
                    msg.values = [steering, throttle]
                    self.publisher.publish(msg)
                    self.get_logger().info(f"Sent Command: Steering={steering}, Throttle={throttle}")

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main():
    rclpy.init()
    node = WASDTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
