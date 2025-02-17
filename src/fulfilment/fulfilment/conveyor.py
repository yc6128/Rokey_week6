# 2025-02-17
# 사용자 입력 - go 200 형태
# 컨베이어 벨트에 명령은 가짐

# -------- 해야 할 것 ----------

# 거리 미세 조정
# INIT, DISCONNECTED 구현

import rclpy
from rclpy.node import Node
import serial
import time
from messages.msg import Control
from messages.msg import Status
import threading
from rclpy.executors import MultiThreadedExecutor

# 1mm 당 스텝 수 (100mm -> 1040 step)
STEP_PER_MM = 10.4

class conveyor(Node):
    def __init__(self):
        super().__init__('conveyor_node')
        # 사용자 명령 수신
        self.subscription = self.create_subscription(
            Control,
            '/control',
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

    # 사용자 명령 수신 콜백
    def listener_callback(self, msg: Control):
        control = msg.control
        if control == 'go':
            distance_mm = msg.distance
            step_value = int(distance_mm * STEP_PER_MM)
            command_str = f"{step_value}\n"  # 아두이노는 숫자 문자열을 step 값으로 인식
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.write(command_str.encode())
                self.get_logger().info(f"Sent step command: {command_str.strip()} for {distance_mm} mm")

            else:
                self.get_logger().error("Serial port is not open!")


        elif control == 'stop':
            # interrupt 같은걸로 모터 멈추게 해야 함!!!!!!
            distance_mm = 1
            step_value = int(distance_mm * STEP_PER_MM)
            command_str = f"{step_value}\n"  # 아두이노는 숫자 문자열을 step 값으로 인식
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.write(command_str.encode())
                self.get_logger().info(f"stopped")
            
        else:
            print('wrong order')

class conveyor_status(Node):
    def __init__(self):
        super().__init__('conveyor_states_node')

        # 상태
        self.status = Status()
        self.flag = 'k'  # dis로 변환!!!!
        self.status.status = 'INIT'

        self.pub = self.create_timer(0.5, self.sub_timer)

        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        time.sleep(2)  # 아두이노 부팅 대기 시간
        self.get_logger().info("Serial port opened successfully!")
        if self.serial_port and self.serial_port.is_open:
            bi = self.serial_port.read()
            if self.flag != bi.decode():
                if self.flag == 's':
                    self.status.status = 'INIT'
                elif self.flag == '.':
                    self.status.status = 'READY'
                elif self.flag == '_':
                    self.status.status = 'RUN'

                print('********')
                print(self.status.status)


    def sub_timer(self):
            
        # 시리얼 포트 초기화 (포트와 보드레이트는 환경에 맞게 수정)
        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            # time.sleep(2)  # 아두이노 부팅 대기 시간
            # self.get_logger().info("Serial port opened successfully!")
            
            if self.serial_port and self.serial_port.is_open:
                bi = self.serial_port.read()
                if self.flag != bi.decode():

                    self.flag = bi.decode()
                    if self.flag == 's':
                        self.status.status = 'INIT'
                    elif self.flag == '.':
                        self.status.status = 'READY'
                    elif self.flag == '_':
                        self.status.status = 'RUN'

                    print('---------------')
                    print(self.status.status)

        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.serial_port = None


# 컨베이어 생태 받아오는 Node

def main(args=None):
    rclpy.init(args=args)
    node1 = conveyor()
    node2 = conveyor_status()

    executor = MultiThreadedExecutor()

    executor.add_node(node1)
    executor.add_node(node2)


    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if executor.serial_port and executor.serial_port.is_open:
            executor.serial_port.close()
        executor.shutdown()
        node1.destroy_node()
        node1.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
       


