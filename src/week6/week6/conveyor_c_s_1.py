#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import serial
import time

# 1mm 당 스텝 수 (100mm -> 1040 step)
STEP_PER_MM = 10.4

class ConveyorController(Node):
    def __init__(self):
        super().__init__('conveyor_controller')
        # distance_cmd 토픽 구독
        self.subscription = self.create_subscription(
            Float64,
            'distance_cmd',
            self.listener_callback,
            10
        )
        # 시리얼 포트 초기화 (포트와 보드레이트는 환경에 맞게 수정)
        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            time.sleep(2)  # 아두이노 부팅 대기 시간
            self.get_logger().info("Serial port opened successfully!")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.serial_port = None

    def listener_callback(self, msg: Float64):
        distance_mm = msg.data
        step_value = int(distance_mm * STEP_PER_MM)
        command_str = f"{step_value}\n"  # 아두이노는 숫자 문자열을 step 값으로 인식
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.write(command_str.encode())
            self.get_logger().info(f"Sent step command: {command_str.strip()} for {distance_mm} mm")
        else:
            self.get_logger().error("Serial port is not open!")

def main(args=None):
    rclpy.init(args=args)
    node = ConveyorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial_port and node.serial_port.is_open:
            node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
