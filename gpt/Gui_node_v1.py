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

# PySide
from PySide6.QtCore import QThread, Signal, QObject, Qt
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QTextEdit, QGroupBox, QDoubleSpinBox,
    QComboBox, QLineEdit, QFormLayout
)
from PySide6.QtGui import QPixmap, QImage

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
    ROS Node + PySide 시그널
     - 카메라 영상 구독(image_raw/compressed)
     - 컨베이어 상태 구독(conveyor/status)
     - 컨베이어 제어(conveyor/control) 발행
     - 작업명령(gui/command) 발행
    """
    # 시그널 정의
    image_signal = Signal(QImage)       # 카메라 영상 업데이트
    conveyor_status_signal = Signal(str)  # 컨베이어 상태 업데이트

    def __init__(self):
        QObject.__init__(self)
        Node.__init__(self, 'gui_node')

        self.last_status = None

        # 1) 카메라 구독
        self.camera_sub = self.create_subscription(
            CompressedImage,
            'image_raw/compressed',  # 카메라 토픽 이름
            self.image_callback,
            10
        )

        # 2) 컨베이어 상태 구독
        self.status_sub = self.create_subscription(
            String,
            'conveyor/status',
            self.status_callback,
            10
        )

        # 3) 발행: 컨베이어 제어, GUI 작업 명령
        self.conveyor_control_pub = self.create_publisher(String, 'conveyor/control', 10)
        self.gui_command_pub = self.create_publisher(String, 'gui/command', 10)

        self.get_logger().info("GuiNode (with camera & distance mode) initialized.")

    # --------------------------------------------------
    # 카메라 콜백 → OpenCV 디코딩 → QImage 변환 → 시그널 emit
    # --------------------------------------------------
    def image_callback(self, msg: CompressedImage):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # BGR
            if cv_img is None:
                self.get_logger().error("Failed to decode camera image (None).")
                return

            # BGR -> QImage
            h, w, ch = cv_img.shape
            bytes_per_line = ch * w
            qimg = QImage(cv_img.data, w, h, bytes_per_line, QImage.Format_BGR888)

            # 시그널로 GUI 업데이트
            self.image_signal.emit(qimg)
        except Exception as e:
            self.get_logger().error(f"image_callback error: {e}")

    # --------------------------------------------------
    # 컨베이어 상태 콜백
    # --------------------------------------------------
    def status_callback(self, msg: String):
        if msg.data != self.last_status:
            self.last_status = msg.data
            self.conveyor_status_signal.emit(msg.data)

    # --------------------------------------------------
    # Publish methods
    # --------------------------------------------------
    def publish_conveyor_control(self, command: dict):
        """
        ex) {"control": "go", "mode":"distance", "distance.mm":200}
            {"control": "stop"}
        """
        s = String()
        s.data = json.dumps(command)
        self.conveyor_control_pub.publish(s)
        self.get_logger().info(f"Publish to 'conveyor/control': {s.data}")

    def publish_gui_command(self, command: dict):
        """
        ex) {"red":2, "blue":1, "goal":1}
        """
        s = String()
        s.data = json.dumps(command)
        self.gui_command_pub.publish(s)
        self.get_logger().info(f"Publish to 'gui/command': {s.data}")


class MainWindow(QMainWindow):
    def __init__(self, node: GuiNode):
        super().__init__()
        self.node = node
        self.setWindowTitle("Conveyor GUI (Camera + Distance)")

        cw = QWidget()
        self.setCentralWidget(cw)
        main_layout = QVBoxLayout()

        # 1) 카메라 영상 표시
        self.image_label = QLabel("No Image")
        self.image_label.setStyleSheet("background-color: black;")
        self.image_label.setFixedSize(400, 300)
        main_layout.addWidget(self.image_label)

        # 2) USB/Conveyor 상태 표시
        self.status_label = QLabel("Conveyor State: ???")
        main_layout.addWidget(self.status_label)

        # 3) Distance 모드: 거리 입력 + Start/Stop
        dist_layout = QHBoxLayout()
        self.distance_spin = QDoubleSpinBox()
        self.distance_spin.setRange(0, 10000)
        self.distance_spin.setValue(200.0)
        self.distance_spin.setSuffix(" mm")

        dist_layout.addWidget(QLabel("Distance:"))
        dist_layout.addWidget(self.distance_spin)

        self.start_btn = QPushButton("Start")
        self.start_btn.clicked.connect(self.on_start_clicked)
        self.stop_btn = QPushButton("Stop")
        self.stop_btn.clicked.connect(self.on_stop_clicked)

        dist_layout.addWidget(self.start_btn)
        dist_layout.addWidget(self.stop_btn)
        main_layout.addLayout(dist_layout)

        # 4) Job 선택 & red/blue 입력
        job_group = QGroupBox("Job Selection")
        job_layout = QHBoxLayout()

        self.job_combo = QComboBox()
        self.job_combo.addItem("Job1")
        self.job_combo.addItem("Job2")
        self.job_combo.addItem("Job3")

        # red, blue 개수 입력
        form_layout = QFormLayout()
        self.red_line = QLineEdit("0")
        self.blue_line = QLineEdit("0")
        form_layout.addRow("Red:", self.red_line)
        form_layout.addRow("Blue:", self.blue_line)

        self.job_exec_btn = QPushButton("Execute Job")
        self.job_exec_btn.clicked.connect(self.on_job_execute)

        job_layout.addWidget(self.job_combo)
        job_layout.addLayout(form_layout)
        job_layout.addWidget(self.job_exec_btn)
        job_group.setLayout(job_layout)
        main_layout.addWidget(job_group)

        # 5) 로그 출력
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        main_layout.addWidget(self.log_text)

        cw.setLayout(main_layout)

        # 시그널 연결
        self.node.image_signal.connect(self.update_image)
        self.node.conveyor_status_signal.connect(self.on_conveyor_status)

        self.resize(600, 600)

    # ---------------------------------
    # 카메라 영상 업데이트
    # ---------------------------------
    def update_image(self, qimg: QImage):
        pix = QPixmap.fromImage(qimg)
        self.image_label.setPixmap(
            pix.scaled(self.image_label.width(), self.image_label.height(), Qt.KeepAspectRatio)
        )

    # ---------------------------------
    # 컨베이어 상태 업데이트
    # ---------------------------------
    def on_conveyor_status(self, status_str: str):
        now = time.strftime("%H:%M:%S")
        self.status_label.setText(f"Conveyor State: {status_str}")
        self.log_text.append(f"[{now}] Conveyor State: {status_str}")

        if status_str == "RUN":
            self.start_btn.setEnabled(False)
            self.stop_btn.setEnabled(True)
        elif status_str == "READY":
            self.start_btn.setEnabled(True)
            self.stop_btn.setEnabled(True)
        else:
            # DISCONNECT, INIT 등
            self.start_btn.setEnabled(False)
            self.stop_btn.setEnabled(False)

    # ---------------------------------
    # Start 버튼 (Distance 모드)
    # ---------------------------------
    def on_start_clicked(self):
        dist = self.distance_spin.value()
        cmd = {
            "control": "go",
            "mode": "distance",
            "distance.mm": dist
        }
        now = time.strftime("%H:%M:%S")
        self.log_text.append(f"[{now}] Start Distance: {dist} mm")
        self.node.publish_conveyor_control(cmd)

    # ---------------------------------
    # Stop 버튼
    # ---------------------------------
    def on_stop_clicked(self):
        cmd = {"control": "stop"}
        now = time.strftime("%H:%M:%S")
        self.log_text.append(f"[{now}] Stop conveyor")
        self.node.publish_conveyor_control(cmd)

    # ---------------------------------
    # Job 실행
    # ---------------------------------
    def on_job_execute(self):
        idx = self.job_combo.currentIndex() + 1  # 1, 2, 3
        red_cnt = int(self.red_line.text()) if self.red_line.text().isdigit() else 0
        blue_cnt = int(self.blue_line.text()) if self.blue_line.text().isdigit() else 0

        # 예: {"red": 2, "blue": 1, "goal": 1}
        cmd = {
            "red": red_cnt,
            "blue": blue_cnt,
            "goal": idx
        }
        now = time.strftime("%H:%M:%S")
        self.log_text.append(f"[{now}] Execute Job{idx} - red={red_cnt}, blue={blue_cnt}")
        self.node.publish_gui_command(cmd)


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
