import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
from std_msgs.msg import String 

class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.signal_publisher = self.create_publisher(String, '/cmd_key', 10)  # 신호 토픽 퍼블리셔
        self.twist = Twist()

        # 키와 명령 매핑
        self.key_bindings = {
            'i': (0.05, 0.0),    # 전진
            ',': (-0.05, 0.0),   # 후진
            'j': (0.0, 0.2),    # 좌회전
            'l': (0.0, -0.2),   # 우회전
            'k': (0.0, 0.0),    # 정지
            'u': (0.05, 0.2),    
            'o': (0.05, -0.2),
            'm': (-0.05, -0.2),
            '.': (-0.05, 0.2),
            'f': 'save',        # save
            'r': 'remove',      # delete
            'q': 'quit'
        }

    def key_callback(self, key):
        if key in self.key_bindings:
            action = self.key_bindings[key]
            if action == 'save':
                self.get_logger().info("Save action activated!")
                # 신호 메시지 생성 및 퍼블리시
                msg = String()
                msg.data = "save"
                self.signal_publisher.publish(msg)
            elif action == 'remove':
                self.get_logger().info("Remove action activated!")
                # 신호 메시지 생성 및 퍼블리시
                msg = String()
                msg.data = "remove"
                self.signal_publisher.publish(msg)
            elif action =='quit':
                self.get_logger().info("Quitting...")
                self.destroy_node()
                rclpy.shutdown()
                sys.exit(0)
            else:
                self.twist.linear.x, self.twist.angular.z = action
                self.publisher_.publish(self.twist)

    def get_key(self):
        # 터미널 설정을 사용하여 실시간으로 키 입력을 받습니다
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)  # 단일 키 입력
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        self.get_logger().info("Press keys to control, 'k' to stop, 'Ctrl+C' to quit.")
        try:
            while rclpy.ok():
                key = self.get_key()
                self.get_logger().info(f"Input key: {key}")
                self.key_callback(key)
        except KeyboardInterrupt:
            self.get_logger().info("Shutting down due to Ctrl+C...")
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    node.run()

if __name__ == '__main__':
    main()
