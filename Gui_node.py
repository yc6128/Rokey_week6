#!/usr/bin/env python3

import sys
import time
import json
import io

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

# PySide6
from PySide6.QtCore import QThread, Signal, QObject, Qt
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QLabel, QPushButton, QTextEdit, QGroupBox, QRadioButton,
    QDoubleSpinBox, QButtonGroup
)
from PySide6.QtGui import QPixmap, QImage, QPainter, QPen, QColor

# ROS 메시지
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

class RosSpinThread(QThread):
    """ROS spin을 별도 스레드에서 돌리는 클래스"""
    def __init__(self, executor):
        super().__init__()
        self.executor = executor
        self.running = True

    def run(self):
        while self.running:
            self.executor.spin_once(timeout_sec=0.1)

    def stop(self):
        self.running = False

class GuiNode(Node, QObject):
    """
    ROS Node + PySide 시그널/슬롯
     - 카메라 영상 구독 (image_raw/compressed_low)
     - 컨베이어 상태 구독 (conveyor/status)
     - 컨베이어 제어 발행 (conveyor/control)
    """
    # PySide 시그널 정의 (PyQt의 pyqtSignal 대신 Signal)
    image_signal = Signal(QImage)  
    status_signal = Signal(str)

    def __init__(self):
        QObject.__init__(self)
        Node.__init__(self, 'gui_node')

        self.last_conveyor_status = None

        # 1) 카메라 구독
        self.sub_image = self.create_subscription(
            CompressedImage,
            'image_raw/compressed_low',   # 토픽 이름
            self.image_callback,
            10
        )

        # 2) 컨베이어 상태 구독
        self.sub_status = self.create_subscription(
            String,
            'conveyor/status',
            self.status_callback,
            10
        )

        # 3) 발행
        self.control_pub = self.create_publisher(String, 'conveyor/control', 10)
        self.order_pub = self.create_publisher(String, 'gui/command', 10)

        # 모터 속도 추정 (예: 52.35 mm/s)
        self.estimated_speed_mm_s = 52.35

        self.get_logger().info("GuiNode (PySide) initialized.")

    def image_callback(self, msg: CompressedImage):
        """OpenCV 디코딩 -> QImage -> 시그널 emit"""
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_img is None:
                self.get_logger().error("Failed to decode image (None).")
                return

            # BGR -> QImage
            height, width, channel = cv_img.shape
            bytes_per_line = channel * width
            qimg = QImage(cv_img.data, width, height, bytes_per_line, QImage.Format_BGR888)

            # 시그널 emit
            self.image_signal.emit(qimg)
        except Exception as e:
            self.get_logger().error(f"image_callback error: {e}")

    def status_callback(self, msg: String):
        if msg.data != self.last_conveyor_status:
            self.last_conveyor_status = msg.data
            self.status_signal.emit(msg.data)

    def publish_control(self, command: dict):
        s = String()
        s.data = json.dumps(command)
        self.control_pub.publish(s)
        self.get_logger().info(f"Publish: {s.data}")

    def publish_order(self, command: dict):
        s = String()
        s.data = json.dumps(command)
        self.order_pub.publish(s)
        self.get_logger().info(f"Publish: {s.data}")


class MainWindow(QMainWindow):
    def __init__(self, node: GuiNode):
        super().__init__()
        self.node = node
        self.setWindowTitle("Conveyor GUI (PySide + OpenCV)")

        cw = QWidget()
        self.setCentralWidget(cw)
        main_layout = QVBoxLayout()

        # 1) 카메라 영상 표시
        self.image_label = QLabel("No Image")
        self.image_label.setFixedSize(400, 300)
        self.image_label.setStyleSheet("background-color: black;")
        main_layout.addWidget(self.image_label)

        # 2) USB/컨베이어 상태 라벨
        self.status_label = QLabel("Conveyor Status: ???")
        main_layout.addWidget(self.status_label)

        # 3) 모드 선택 (Distance / Time)
        mode_box = QGroupBox("Mode")
        mode_layout = QHBoxLayout()

        self.distance_radio = QRadioButton("Distance Mode")
        self.distance_radio.setChecked(True)
        self.time_radio = QRadioButton("Time Mode")

        self.mode_group = QButtonGroup()
        self.mode_group.addButton(self.distance_radio)
        self.mode_group.addButton(self.time_radio)

        mode_layout.addWidget(self.distance_radio)
        mode_layout.addWidget(self.time_radio)
        mode_box.setLayout(mode_layout)
        main_layout.addWidget(mode_box)

        # 4) 파라미터 (distance.mm, time.sec)
        param_layout = QHBoxLayout()
        self.distance_spin = QDoubleSpinBox()
        self.distance_spin.setRange(0, 100000)
        self.distance_spin.setValue(200.0)
        self.distance_spin.setSuffix(" mm")

        self.time_spin = QDoubleSpinBox()
        self.time_spin.setRange(0, 3600)
        self.time_spin.setValue(5.0)
        self.time_spin.setSuffix(" s")

        param_layout.addWidget(QLabel("Distance:"))
        param_layout.addWidget(self.distance_spin)
        param_layout.addWidget(QLabel("Time:"))
        param_layout.addWidget(self.time_spin)

        main_layout.addLayout(param_layout)

        # 5) Start / Stop 버튼
        btn_layout = QHBoxLayout()
        self.start_btn = QPushButton("Start")
        self.stop_btn = QPushButton("Stop")

        self.start_btn.clicked.connect(self.on_start_clicked)
        self.stop_btn.clicked.connect(self.on_stop_clicked)

        btn_layout.addWidget(self.start_btn)
        btn_layout.addWidget(self.stop_btn)
        main_layout.addLayout(btn_layout)

        # 6) 사용자 명령 목록 버튼
        job_layout = QHBoxLayout()
        self.job1 = QPushButton("job1")
        self.job2 = QPushButton("job2")
        self.job3 = QPushButton("job3")

        self.job1.clicked.connect(self.on_job1_clicked)
        self.job2.clicked.connect(self.on_job2_clicked)
        self.job3.clicked.connect(self.on_job3_clicked)

        job_layout.addWidget(self.job1)
        job_layout.addWidget(self.job2)
        job_layout.addWidget(self.job3)
        main_layout.addLayout(job_layout)

        # 7) 상태창 (로그)
        self.status_text = QTextEdit()
        self.status_text.setReadOnly(True)
        main_layout.addWidget(self.status_text)

        cw.setLayout(main_layout)

        # 내부 변수
        self.run_start_time = None
        self.current_mode = None  # "distance", "time"
        self.requested_distance = 0.0
        self.requested_time = 0.0

        # 아두이노 펌웨어상 속도 추정
        self.estimated_speed_mm_s = node.estimated_speed_mm_s

        # 시그널 연결
        self.node.image_signal.connect(self.update_image)
        self.node.status_signal.connect(self.on_conveyor_status)

        self.resize(600, 600)

    # ---------------------------------------------------
    # 사용자 바구니 명령
    # ---------------------------------------------------
    def on_job1_clicked(self):
        cmd = {'red':2,'blue':1,'goal':1}
        self.node.publish_order(cmd)

    def on_job2_clicked(self):
        cmd = {'red':1,'blue':2,'goal':2}
        self.node.publish_order(cmd)

    def on_job3_clicked(self):
        cmd = {'red':1,'blue':0,'goal':3}
        self.node.publish_order(cmd)

    # ---------------------------------------------------
    # 카메라 영상 업데이트 (십자가 표시 추가)
    # ---------------------------------------------------
    def update_image(self, qimg: QImage):
        # QPixmap 변환
        pixmap = QPixmap.fromImage(qimg)

        # 라벨 크기에 맞춰 영상 스케일링
        scaled_pixmap = pixmap.scaled(
            self.image_label.width(),
            self.image_label.height(),
            Qt.KeepAspectRatio
        )

        # 스케일링된 픽스맵에 노란색 십자가 그리기
        painter = QPainter(scaled_pixmap)
        painter.setPen(QPen(QColor("yellow"), 2, Qt.SolidLine))

        w = scaled_pixmap.width()
        h = scaled_pixmap.height()
        cx = w // 2
        cy = h // 2

        # 수직선 (위->아래)
        painter.drawLine(cx, 0, cx, h)
        # 수평선 (왼->오른쪽)
        painter.drawLine(0, cy, w, cy)

        painter.end()

        # 최종 픽스맵 표시
        self.image_label.setPixmap(scaled_pixmap)

    # ---------------------------------------------------
    # 컨베이어 상태 업데이트
    # ---------------------------------------------------
    def on_conveyor_status(self, status_str: str):
        now = time.strftime("%H:%M:%S")
        self.status_label.setText(f"Conveyor Status: {status_str}")
        self.status_text.append(f"[{now}] Conveyor State: {status_str}")

        if status_str == "RUN":
            self.start_btn.setEnabled(False)
            self.stop_btn.setEnabled(True)
            self.run_start_time = time.time()
        elif status_str == "READY":
            self.start_btn.setEnabled(True)
            self.stop_btn.setEnabled(True)
            if self.run_start_time is not None:
                elapsed = time.time() - self.run_start_time
                self.status_text.append(f"   => 동작시간: {elapsed:.2f}초")

                if self.current_mode == "distance":
                    self.status_text.append(
                        f"   => 설정 거리: {self.requested_distance:.2f} mm"
                    )
                elif self.current_mode == "time":
                    traveled_mm = self.estimated_speed_mm_s * elapsed
                    self.status_text.append(
                        f"   => (Time Mode) 대략 이동 거리: {traveled_mm:.1f} mm"
                    )

            # 초기화
            self.run_start_time = None
            self.current_mode = None
            self.requested_distance = 0.0
            self.requested_time = 0.0

        elif status_str == "DISCONNECT":
            self.start_btn.setEnabled(False)
            self.stop_btn.setEnabled(False)
        else:
            # INIT 등
            self.start_btn.setEnabled(False)
            self.stop_btn.setEnabled(False)

    def on_start_clicked(self):
        if self.distance_radio.isChecked():
            mode = "distance"
            dist = self.distance_spin.value()
            cmd = {
                "control": "go",
                "mode": "distance",
                "distance.mm": dist
            }
            msg = f"Start Distance: {dist:.2f} mm"
            self.requested_distance = dist
            self.requested_time = 0.0
        else:
            mode = "time"
            t_sec = self.time_spin.value()
            cmd = {
                "control": "go",
                "mode": "time",
                "time.sec": t_sec
            }
            msg = f"Start Time: {t_sec:.2f} s"
            self.requested_distance = 0.0
            self.requested_time = t_sec

        self.current_mode = mode

        self.node.publish_control(cmd)
        now = time.strftime("%H:%M:%S")
        self.status_text.append(f"[{now}] {msg}")

    def on_stop_clicked(self):
        cmd = {"control": "stop"}
        self.node.publish_control(cmd)
        now = time.strftime("%H:%M:%S")
        self.status_text.append(f"[{now}] Stop Conveyor")


def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor(num_threads=2)

    gui_node = GuiNode()
    executor.add_node(gui_node)

    ros_thread = RosSpinThread(executor)
    ros_thread.start()

    app = QApplication(sys.argv)
    window = MainWindow(gui_node)
    window.show()

    try:
        code = app.exec_()
    except KeyboardInterrupt:
        code = 0

    ros_thread.stop()
    ros_thread.wait()
    executor.shutdown()
    gui_node.destroy_node()
    rclpy.shutdown()
    sys.exit(code)

if __name__ == "__main__":
    main()
