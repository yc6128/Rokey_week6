# 2025-02-17
# 사용자 입력 - go 200 형태
# 컨베이어 벨트에 명령은 가짐
# -------- 해야 할 것
# 컨베이어 벨트 상태 받아오기
# 명령 종류별로 다르게 움직이게
# 거리 미세 조정
# 정지 명령 활성화
# 멀티 스레드로 상태 받아오기

import rclpy
from rclpy.node import Node
import serial
import time
from messages.msg import Control
from messages.msg import Status

class conveyor(Node):
    def __init__(self):
        super().__init__('conveyor_node')

        # 상태
        self.status = Status()

        self.pub = self.create_timer(1, self.sub_timer)

    def sub_timer(self):
            
        # 시리얼 포트 초기화 (포트와 보드레이트는 환경에 맞게 수정)
        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            time.sleep(2)  # 아두이노 부팅 대기 시간
            self.get_logger().info("Serial port opened successfully!")
            if self.serial_port and self.serial_port.is_open:
                self.current_status = self.serial_port.read()
                print('------------')
                print(self.current_status)

        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.serial_port = None



# 컨베이어 생태 받아오는 Node

def main(args=None):
    rclpy.init(args=args)
    node = conveyor()
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
       


