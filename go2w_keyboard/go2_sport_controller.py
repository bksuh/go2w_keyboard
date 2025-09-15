#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

import time
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient
from dataclasses import dataclass

# TestOption은 ID와 명령어 이름을 매핑하여 로그 가독성을 높이는 데 사용됩니다.
@dataclass
class TestOption:
    name: str
    id: int

class Go2SportControllerNode(Node):
    def __init__(self):
        super().__init__('go2_sport_controller_node')
        # 네트워크 인터페이스를 위한 ROS 파라미터 선언 (기본값: 'eth0')
        self.declare_parameter('network_interface', 'eno1')
        network_interface = self.get_parameter('network_interface').get_parameter_value().string_value
        self.get_logger().info(f"Using network interface: {network_interface}")

        # Unitree SDK 초기화
        try:
            ChannelFactoryInitialize(0, network_interface)
            self.sport_client = SportClient()
            self.sport_client.SetTimeout(10.0)
            self.sport_client.Init()
            self.get_logger().info("SportClient initialized successfully.")
            self.sport_client.SpeedLevel(1)
            self.get_logger().info("Set speed level to medium.")

        except Exception as e:
            self.get_logger().error(f"Failed to initialize SportClient: {e}")
            # 초기화 실패 시 노드를 종료합니다.
            rclpy.shutdown()
            return

        # '/test_option_id' 토픽을 구독하는 Subscriber 생성
        self.subscription = self.create_subscription(
            Twist,
            'go2_sport_vel',
            self.command_callback,
            10)
        self.get_logger().info("ROS 2 node started. Waiting for commands on /go2_sport_vel topic...")
        self.get_logger().warn("Please ensure there are no obstacles around the robot.")


    def command_callback(self, msg):
        self.sub_vx = msg.linear.x
        self.sub_vy = msg.linear.y
        self.sub_vyaw = msg.angular.z
        self.get_logger().info(f"Received command [vx: {self.sub_vx}, vy: {self.sub_vy}, vyaw: {self.sub_vyaw}]")
        self.sport_client.Move(self.sub_vx, self.sub_vy, self.sub_vyaw)


def main(args=None):
    rclpy.init(args=args)
    
    go2_sport_controller = Go2SportControllerNode()
    
    # 노드가 종료될 때까지 대기
    if rclpy.ok():
      rclpy.spin(go2_sport_controller)
    
    go2_sport_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()