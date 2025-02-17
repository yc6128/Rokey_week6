#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class DistancePublisher(Node):
    def __init__(self):
        super().__init__('distance_publisher')
        self.publisher_ = self.create_publisher(Float64, 'distance_cmd', 10)

    def publish_distance(self, distance: float):
        msg = Float64()
        msg.data = distance
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing distance: {distance} mm')

def main(args=None):
    rclpy.init(args=args)
    node = DistancePublisher()
    try:
        # 반복문으로 사용자 입력을 받아 publish
        while rclpy.ok():
            user_input = input("Enter distance in mm (or 'exit' to quit): ")
            if user_input.lower() == 'exit':
                break
            try:
                distance = float(user_input)
                node.publish_distance(distance)
            except ValueError:
                print("Invalid input. Please enter a number.")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
