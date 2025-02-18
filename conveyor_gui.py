#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time
import json
import math

class ConveyorNode(Node):
    def __init__(self):
        super().__init__('conveyor_node')

        # 시리얼 설정
        self.serial_port_name = '/dev/ttyACM0'  # 실제 환경 맞게 수정
        self.baud_rate = 115200
        self.ser = None

        # 모터/기어/롤러 계산
        self.steps_per_rev_motor = 200
        self.gear_ratio = 3.75
        self.roller_diameter_mm = 25.0
        self.roller_circum_mm = math.pi * self.roller_diameter_mm  # ~78.54 mm
        self.steps_per_mm = (self.gear_ratio * self.steps_per_rev_motor) / self.roller_circum_mm
        # 1스텝 ≈ 0.1047mm (이론)

        # 상태
        self.conveyor_state = 'INIT'
        self.last_state = None

        # Time 모드 타임아웃
        self.run_start_time = None
        self.run_timeout_sec = 0.0

        # Subscriber, Publisher
        self.control_sub = self.create_subscription(
            String, 'conveyor/control', self.control_callback, 10
        )
        self.status_pub = self.create_publisher(String, 'conveyor/status', 10)

        # 주기타이머 (0.2초)
        self.timer = self.create_timer(0.2, self.on_timer)

        self.get_logger().info("ConveyorNode initialized.")

        # 초기 상태 Publish -> GUI에서 ??? 대신 INIT 표시
        self.publish_state()

        # 시리얼 연결 시도
        self.try_connect_serial()

    def publish_state(self):
        msg = String()
        msg.data = self.conveyor_state
        self.status_pub.publish(msg)
        self.last_state = self.conveyor_state

    def try_connect_serial(self):
        """시리얼 포트 연결 & DTR 토글(Arduino 자동 리셋)"""
        if self.ser and self.ser.is_open:
            return
        try:
            self.ser = serial.Serial(self.serial_port_name, self.baud_rate, timeout=0.1)
            # 아두이노 보드 리셋
            self.ser.setDTR(False)
            time.sleep(0.2)
            self.ser.setDTR(True)
            time.sleep(0.5)

            self.conveyor_state = 'READY'
            self.get_logger().info(f"Serial connected: {self.serial_port_name}")
        except serial.SerialException as e:
            self.ser = None
            self.conveyor_state = 'DISCONNECT'
            

    def on_timer(self):
        
        # 1) USB 연결 체크
        try:
            if not (self.ser and self.ser.is_open):
                self.try_connect_serial()
                    # 2) 시리얼 수신 ('.','_' 등)
            if self.ser and self.ser.is_open and self.ser.in_waiting > 0:
                try:
                    _ = self.ser.read(self.ser.in_waiting)
                except serial.SerialException as e:
                    self.get_logger().error(f"Serial read error: {e}")
                    self.ser = None
                    self.conveyor_state = 'DISCONNECT'

        # 3) Time 모드 => 지정 시간 지나면 자동 stop
            if self.conveyor_state == 'RUN' and self.run_timeout_sec > 0:
                elapsed = time.time() - self.run_start_time
                if elapsed >= self.run_timeout_sec:
                    self.get_logger().info("Time mode done => stop conveyor")
                    self.stop_conveyor()

        # 4) 상태 변경 시만 Publish
            if self.conveyor_state != self.last_state:
                self.publish_state()
        except:
            self.ser = None
            self.conveyor_state = 'DISCONNECT'
            if self.conveyor_state != self.last_state:
                self.publish_state()

            

    def control_callback(self, msg: String):
        """
        예: 
          {"control":"go","mode":"distance","distance.mm":200}
          {"control":"go","mode":"time","time.sec":5}
          {"control":"stop"}
        """
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error("Invalid JSON in conveyor/control")
            return

        ctrl = data.get('control', '')
        if ctrl == 'go':
            mode = data.get('mode', '')
            if mode == 'distance':
                dist_mm = data.get('distance.mm', 0.0)
                self.start_distance_mode(dist_mm)
            elif mode == 'time':
                t_sec = data.get('time.sec', 0.0)
                self.start_time_mode(t_sec)
            else:
                self.get_logger().warn(f"Unknown mode: {mode}")
        elif ctrl == 'stop':
            self.stop_conveyor()
        else:
            self.get_logger().warn(f"Unknown control: {ctrl}")

    def start_distance_mode(self, distance_mm: float):
        if not (self.ser and self.ser.is_open):
            self.conveyor_state = 'DISCONNECT'
            return

        steps = int(distance_mm * self.steps_per_mm)
        if steps < 1:
            self.get_logger().info(f"Distance {distance_mm} mm => steps={steps}, ignoring.")
            return

        cmd = f"{steps}\n"
        try:
            self.ser.write(cmd.encode('utf-8'))
            self.conveyor_state = 'RUN'
            self.get_logger().info(f"[Dist Mode] {distance_mm:.1f}mm => send {steps} steps")
            self.run_start_time = time.time()
            self.run_timeout_sec = 0.0
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write error: {e}")
            self.conveyor_state = 'DISCONNECT'

    def start_time_mode(self, t_sec: float):
        if not (self.ser and self.ser.is_open):
            self.conveyor_state = 'DISCONNECT'
            return

        big_steps = 999999
        cmd = f"{big_steps}\n"
        try:
            self.ser.write(cmd.encode('utf-8'))
            self.conveyor_state = 'RUN'
            self.get_logger().info(f"[Time Mode] {t_sec:.1f}s => send {big_steps} steps")
            self.run_start_time = time.time()
            self.run_timeout_sec = t_sec
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write error: {e}")
            self.conveyor_state = 'DISCONNECT'

    def stop_conveyor(self):
        """'1\n'을 전송 -> step_count=1 -> 1~2펄스 후 0 => READY"""
        if not (self.ser and self.ser.is_open):
            self.conveyor_state = 'DISCONNECT'
            return
        try:
            self.ser.write(b"1\n")
            self.get_logger().info("[Stop] send 1 => quick stop")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write error: {e}")
            self.conveyor_state = 'DISCONNECT'
            return

        # 바로 READY로 전환
        self.conveyor_state = 'READY'
        self.run_start_time = None
        self.run_timeout_sec = 0.0

def main(args=None):
    rclpy.init(args=args)
    node = ConveyorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
