#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import serial
import time

# 1mm 당 스텝 수 (아두이노 코드 기준: 100mm → 1040 step)
STEP_PER_MM = 10.4

class ConveyorController(Node):
    def __init__(self):
        super().__init__('conveyor_controller')
        # distance_cmd 토픽을 구독 (단위: mm)
        self.subscription = self.create_subscription(
            Float64,
            'distance_cmd',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # 시리얼 포트 초기화 (포트 이름과 보드레이트는 환경에 맞게 변경)
        try:
            self.serial_port = serial.Serial('Arduiino Uno on/dev/ttyACM0', 115200, timeout=1)
            # 아두이노 리셋 및 부팅을 위해 잠시 대기
            time.sleep(2)
            self.get_logger().info("Serial port opened successfully!")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")

    def listener_callback(self, msg: Float64):
        # ROS2에서 입력받은 거리를 mm 단위로 처리
        distance_mm = msg.data
        # step 값으로 변환 (소수점 이하 버림)
        step_value = int(distance_mm * STEP_PER_MM)
        
        # step 값 문자열로 전송 (숫자만 보내면 아두이노에서는 step 모드로 인식)
        command_str = str(step_value) + "\n"
        
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.write(command_str.encode())
            self.get_logger().info(f"Sent step command: {command_str.strip()} (for {distance_mm} mm)")
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
