import rclpy
from rclpy.node import Node
from nav2_msgs.srv import SaveMap

class MapSaverClient(Node):
    def __init__(self):
        super().__init__('map_saver_client')
        self.cli = self.create_client(SaveMap, 'map_saver/save_map')  # 서비스 명칭 확인

        # 파라미터를 통해 저장 경로를 설정받기
        self.declare_parameter('map_path', '/default/path/to/map')
        self.map_path = self.get_parameter('map_path').get_parameter_value().string_value
        self.get_logger().info(f"map_path : {self.map_path}")

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for save_map service...')

        self.req = SaveMap.Request()

    def save_map(self):
        self.get_logger().info(f"run save_map...")

        self.req.map_url = self.map_path
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Map saved successfully at {self.map_path}")
        else:
            self.get_logger().error("Failed to save the map.")

def main(args=None):
    rclpy.init(args=args)
    node = MapSaverClient()
    node.save_map()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
