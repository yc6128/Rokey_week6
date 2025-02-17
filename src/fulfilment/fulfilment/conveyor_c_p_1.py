#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from messages.msg import Control

class DistancePublisher(Node):
    def __init__(self):
        super().__init__('distance_publisher')
        self.publisher_ = self.create_publisher(Control, '/control', 10)

    def publish_distance(self, control, distance: float):
        msg = Control()
        msg.control = control
        msg.distance = distance
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing control : {control}, distance: {distance} mm')

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
                # go 200형태로 입력
                inp = user_input.split(' ')
                control = inp[0]
                if len(inp) > 1:
                    distance = float(inp[1])
                else:
                    distance = float(0)
                node.publish_distance(control, distance)
            except ValueError:
                print("Invalid input. Please enter a number.")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
