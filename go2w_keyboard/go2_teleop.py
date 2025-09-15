import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select

class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop_node')
        self.pub = self.create_publisher(Twist, 'go2_sport_vel', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz

        self.current_twist = Twist()
        self.settings = termios.tcgetattr(sys.stdin)

        self.get_logger().info("KeyboardTeleopNode started. Use W/A/S/D for vx/vy, K/L for vyaw. Ctrl-C to quit.")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = None
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def timer_callback(self):
        key = self.get_key()
        twist = Twist()  # 기본값 0.0

        if key == 'w':
            twist.linear.x = 0.5
        elif key == 's':
            twist.linear.x = -0.5
        elif key == 'a':
            twist.linear.y = 0.5
        elif key == 'd':
            twist.linear.y = -0.5
        elif key == 'k':
            twist.angular.z = 0.5
        elif key == 'l':
            twist.angular.z = -0.5
        elif key == '\x03':  # Ctrl-C
            rclpy.shutdown()
            return

        # 키 눌린 순간만 값 설정, 안 눌렸으면 모두 0.0
        self.current_twist = twist

        # 발행
        self.pub.publish(self.current_twist)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()