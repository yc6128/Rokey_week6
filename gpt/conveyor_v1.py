#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import serial
import serial.serialutil
import threading
import time
import math
import json

class ConveyorNode(Node):
    def __init__(self):
        super().__init__('conveyor_node')
        self.get_logger().info("ConveyorNode initializing...")

        # -- 시리얼 설정 --
        self.serial_port_name = '/dev/ttyACM0'  # 환경에 맞게 수정
        self.baud_rate = 115200
        self.ser = None  # 시리얼 핸들

        # 모터/기어/롤러 계산 (Distance 모드용)
        self.steps_per_rev_motor = 200
        self.gear_ratio = 3.75
        self.roller_diameter_mm = 25.0
        self.roller_circum_mm = math.pi * self.roller_diameter_mm  # ~78.54 mm
        # => 750스텝 => 78.54mm => 1스텝≈0.1047mm
        self.steps_per_mm = (self.gear_ratio * self.steps_per_rev_motor) / self.roller_circum_mm

        # 상태: DISCONNECT / READY / RUN / INIT 등
        self.conveyor_state = 'INIT'
        self.last_state = None

        # ROS 통신: 구독 & 발행
        self.control_sub = self.create_subscription(
            String, 'conveyor/control', self.control_callback, 10
        )
        self.status_pub = self.create_publisher(String, 'conveyor/status', 10)

        # 초기 상태 Publish (GUI에서 ??? 대신 INIT으로 인식)
        self.publish_state()

        # USB 연결/해제 처리를 위한 멀티스레드
        self.stop_thread = False
        self.thread = threading.Thread(target=self.serial_thread_func, daemon=True)
        self.thread.start()

        self.get_logger().info("ConveyorNode ready.")

    # ---------------------------
    # 상태 Publish (변경 시에만)
    # ---------------------------
    def publish_state(self):
        if self.conveyor_state != self.last_state:
            msg = String()
            msg.data = self.conveyor_state
            self.status_pub.publish(msg)
            self.last_state = self.conveyor_state

    # ---------------------------
    # 별도 스레드: 시리얼 연결 & 데이터 수신
    # ---------------------------
    def serial_thread_func(self):
        """
        1) 시리얼 포트 열기 시도
        2) 성공 시 주기적으로 수신 확인
           - '.' => READY
           - '_' => RUN
        3) 포트 끊기면 DISCONNECT 상태, 재시도
        """
        while not self.stop_thread:
            # 연결되어 있지 않으면 시도
            if not self.ser or not self.ser.is_open:
                try:
                    self.ser = serial.Serial(self.serial_port_name, self.baud_rate, timeout=0.1)
                    # DTR 토글 → 아두이노 리셋
                    self.ser.setDTR(False)
                    time.sleep(0.2)
                    self.ser.setDTR(True)
                    time.sleep(0.5)

                    self.get_logger().info(f"Serial connected: {self.serial_port_name}")
                    self.conveyor_state = 'READY'
                    self.publish_state()
                except serial.serialutil.SerialException:
                    # 연결 실패
                    if self.conveyor_state != 'DISCONNECT':
                        self.conveyor_state = 'DISCONNECT'
                        self.publish_state()
                    time.sleep(1.0)
                    continue  # 다시 while문 처음으로

            # 만약 연결되어 있다면, 데이터 수신
            if self.ser and self.ser.is_open:
                try:
                    if self.ser.in_waiting > 0:
                        data = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                        # 예: '.' => READY, '_' => RUN
                        if '.' in data:
                            # '.' 문자가 있으면 READY 상태로 볼 수 있음
                            if self.conveyor_state != 'DISCONNECT':
                                self.conveyor_state = 'READY'
                                self.publish_state()
                        if '_' in data:
                            # '_' 문자가 있으면 RUN 상태
                            if self.conveyor_state != 'DISCONNECT':
                                self.conveyor_state = 'RUN'
                                self.publish_state()
                except serial.serialutil.SerialException as e:
                    self.get_logger().error(f"Serial read error: {e}")
                    if self.ser:
                        self.ser.close()
                    self.ser = None
                    self.conveyor_state = 'DISCONNECT'
                    self.publish_state()

            time.sleep(0.1)

    # ---------------------------
    # ROS 콜백: conveyor/control
    # ---------------------------
    def control_callback(self, msg: String):
        """
        Distance 모드: {"control":"go","mode":"distance","distance.mm":200}
        Stop:          {"control":"stop"}
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
            else:
                self.get_logger().warn(f"Unknown mode: {mode}")
        elif ctrl == 'stop':
            self.stop_conveyor()
        else:
            self.get_logger().warn(f"Unknown control: {ctrl}")

    def start_distance_mode(self, distance_mm: float):
        """distance(mm) → 스텝 → 아두이노에 전송"""
        # 연결되어 있는지 체크
        if not (self.ser and self.ser.is_open):
            self.conveyor_state = 'DISCONNECT'
            self.publish_state()
            self.get_logger().error("Cannot start conveyor: no serial connection.")
            return

        steps = int(distance_mm * self.steps_per_mm)
        if steps < 1:
            self.get_logger().info(f"Distance {distance_mm} mm => steps={steps}, ignoring.")
            return

        cmd = f"{steps}\n"
        try:
            self.ser.write(cmd.encode('utf-8'))
            self.get_logger().info(f"Send steps: {steps} for {distance_mm:.1f} mm")
            self.conveyor_state = 'RUN'
            self.publish_state()
        except serial.serialutil.SerialException as e:
            self.get_logger().error(f"Serial write error: {e}")
            if self.ser:
                self.ser.close()
            self.ser = None
            self.conveyor_state = 'DISCONNECT'
            self.publish_state()

    def stop_conveyor(self):
        """'1\n' 전송 -> 거의 즉시 정지"""
        if not (self.ser and self.ser.is_open):
            self.conveyor_state = 'DISCONNECT'
            self.publish_state()
            return
        try:
            self.ser.write(b"1\n")
            self.get_logger().info("[Stop] send 1 => quick stop")
            self.conveyor_state = 'READY'
            self.publish_state()
        except serial.serialutil.SerialException as e:
            self.get_logger().error(f"Serial write error: {e}")
            if self.ser:
                self.ser.close()
            self.ser = None
            self.conveyor_state = 'DISCONNECT'
            self.publish_state()

    def destroy_node(self):
        self.get_logger().info("Shutting down ConveyorNode...")
        self.stop_thread = True
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ConveyorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
