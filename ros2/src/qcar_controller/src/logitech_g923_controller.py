import rclpy
from rclpy.node import Node
from qcar2_interfaces.msg import MotorCommands
import struct
import math

# Constants
STEERING_AXIS = 0
ACCEL_AXIS = 2
BTN_DRIVE = 0
BTN_REVERSE = 2
BTN_STEER_SENS_UP = 19
BTN_STEER_SENS_DOWN = 20
BTN_PADDLE_LEFT = 5
BTN_PADDLE_RIGHT = 4

DEFAULT_STEERING_SENS = 12000
MAX_THROTTLE_MS = 1.0

class G923Controller(Node):
    def __init__(self):
        super().__init__('g923_qcar2_controller')

        self.publisher = self.create_publisher(MotorCommands, '/qcar2_motor_speed_cmd', 10)

        self.steering_sens = DEFAULT_STEERING_SENS
        self.speed_sens = 1.0
        self.gear = "NEUTRAL"
        self.steering_angle = 0.0
        self.throttle = 0.0

        self.js = open('/dev/input/js0', 'rb')
        self.get_logger().info("G923 controller running...")

        self.run_loop()

    def run_loop(self):
        while rclpy.ok():
            evbuf = self.js.read(8)
            if not evbuf:
                continue

            _, value, type_raw, number = struct.unpack('IhBB', evbuf)
            event_type = type_raw & ~0x80

            if event_type == 0x02:  # Axis
                if number == STEERING_AXIS:
                    self.steering_angle = -1*max(min((value / self.steering_sens) * (math.pi / 6), math.pi / 6), -math.pi / 6)
                elif number == ACCEL_AXIS:
                    raw = max(min(value, 32767), -32767)
                    self.throttle = (32767 - raw) / 65534.0 * self.speed_sens
                    self.throttle = min(max(self.throttle, 0.0), MAX_THROTTLE_MS)

            elif event_type == 0x01:  # Button
                if value == 1:
                    self.get_logger().info(f"Button {number} pressed")
                else:
                    self.get_logger().info(f"Button {number} released")

                if number == BTN_DRIVE and value == 1:
                    self.gear = "DRIVE"
                elif number == BTN_REVERSE and value == 1:
                    self.gear = "REVERSE"
                elif number == BTN_PADDLE_LEFT and value == 1:
                    self.speed_sens = max(0.1, self.speed_sens - 0.1)
                elif number == BTN_PADDLE_RIGHT and value == 1:
                    self.speed_sens = min(2.0, self.speed_sens + 0.1)
                elif number == BTN_STEER_SENS_UP and value == 1:
                    self.steering_sens = max(1000, self.steering_sens - 1000)
                elif number == BTN_STEER_SENS_DOWN and value == 1:
                    self.steering_sens = min(32767, self.steering_sens + 1000)

            # Publish right after any input
            throttle_value = self.throttle if self.gear == "DRIVE" else -self.throttle if self.gear == "REVERSE" else 0.0

            msg = MotorCommands()
            msg.motor_names = ["steering_angle", "motor_throttle"]
            msg.values = [self.steering_angle, throttle_value]
            self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = G923Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down.")
    finally:
        node.js.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()