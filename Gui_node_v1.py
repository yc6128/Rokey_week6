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
    image_signal = Signal(QImage)  
    status_signal = Signal(str)

    def __init__(self):
        QObject.__init__(self)
        Node.__init__(self, 'gui_node')

        self.last_conveyor_status = None

        # 1) 카메라 구독
        self.sub_image = self.create_subscription(
            CompressedImage,
            'image_raw/compressed_low',   # 예시 토픽 이름
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

        # 모터 속도 추정
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

        # 윈도우 기본 크기 설정: 1280 x 720
        self.resize(1280, 720)

        # 메인 중앙 위젯
        cw = QWidget()
        self.setCentralWidget(cw)

        # 전체 레이아웃: 수직
        main_layout = QVBoxLayout()
        cw.setLayout(main_layout)

        # ---------------------------------------------------
        # 상단 레이아웃 (좌: 카메라 영상, 우: 작업 모드)
        # ---------------------------------------------------
        top_layout = QHBoxLayout()
        main_layout.addLayout(top_layout)

        # [A] 카메라 영상 표시 영역 (좌상단)
        self.image_label = QLabel("No Image")
        # 영상 표시 라벨 크기: 640 x 360
        self.image_label.setFixedSize(640, 360)
        self.image_label.setStyleSheet("background-color: black;")
        top_layout.addWidget(self.image_label)

        # [B] 작업 모드 UI (우상단)
        mode_box = QGroupBox("작업 모드")
        mode_box_layout = QVBoxLayout(mode_box)
        top_layout.addWidget(mode_box)

        # (B-1) 현재 상태 + ONE CLICK 버튼 (동그라미)
        state_layout = QHBoxLayout()
        self.current_state_label = QLabel("현재 상태: Rest")
        state_layout.addWidget(self.current_state_label)

        self.oneclick_button = QPushButton("ONE CLICK")
        # 동그란 모양, 색상/글꼴 등 스타일 예시
        self.oneclick_button.setFixedSize(80, 80)
        self.oneclick_button.setStyleSheet(
            "QPushButton {"
            "  border-radius: 40px;"       # 반지름 = 절반 크기로 동그라미
            "  background-color: #FFC000;" # 예시 노란색
            "  font-weight: bold;"
            "}"
        )
        # 필요시 클릭 시 어떤 동작을 할지 연결
        self.oneclick_button.clicked.connect(self.on_oneclick_button)
        state_layout.addWidget(self.oneclick_button)

        mode_box_layout.addLayout(state_layout)

        # (B-2) 4가지 모드: (작업장 이동, 분류, 픽업 이동, 배달) + Play/Stop
        # -- 작업장 이동 --
        work_layout = QHBoxLayout()
        self.work_label = QLabel("작업장 이동")
        self.work_play_btn = QPushButton("Play")
        self.work_stop_btn = QPushButton("Stop")
        work_layout.addWidget(self.work_label)
        work_layout.addWidget(self.work_play_btn)
        work_layout.addWidget(self.work_stop_btn)
        mode_box_layout.addLayout(work_layout)
        self.work_play_btn.clicked.connect(self.on_work_play)
        self.work_stop_btn.clicked.connect(self.on_work_stop)

        # -- 분류 --
        classify_layout = QHBoxLayout()
        self.classify_label = QLabel("분류")
        self.classify_play_btn = QPushButton("Play")
        self.classify_stop_btn = QPushButton("Stop")
        classify_layout.addWidget(self.classify_label)
        classify_layout.addWidget(self.classify_play_btn)
        classify_layout.addWidget(self.classify_stop_btn)
        mode_box_layout.addLayout(classify_layout)
        self.classify_play_btn.clicked.connect(self.on_classify_play)
        self.classify_stop_btn.clicked.connect(self.on_classify_stop)

        # -- 픽업 이동 --
        pickup_layout = QHBoxLayout()
        self.pickup_label = QLabel("픽업 이동")
        self.pickup_play_btn = QPushButton("Play")
        self.pickup_stop_btn = QPushButton("Stop")
        pickup_layout.addWidget(self.pickup_label)
        pickup_layout.addWidget(self.pickup_play_btn)
        pickup_layout.addWidget(self.pickup_stop_btn)
        mode_box_layout.addLayout(pickup_layout)
        self.pickup_play_btn.clicked.connect(self.on_pickup_play)
        self.pickup_stop_btn.clicked.connect(self.on_pickup_stop)

        # -- 배달 --
        deliver_layout = QHBoxLayout()
        self.deliver_label = QLabel("배달")
        self.deliver_play_btn = QPushButton("Play")
        self.deliver_stop_btn = QPushButton("Stop")
        deliver_layout.addWidget(self.deliver_label)
        deliver_layout.addWidget(self.deliver_play_btn)
        deliver_layout.addWidget(self.deliver_stop_btn)
        mode_box_layout.addLayout(deliver_layout)
        self.deliver_play_btn.clicked.connect(self.on_deliver_play)
        self.deliver_stop_btn.clicked.connect(self.on_deliver_stop)

        # ---------------------------------------------------
        # 하단 레이아웃 (컨베이어 관련)
        # ---------------------------------------------------
        bottom_layout = QVBoxLayout()
        main_layout.addLayout(bottom_layout)

        # (1) 컨베이어 상태 라벨
        self.status_label = QLabel("Conveyor Status: ???")
        bottom_layout.addWidget(self.status_label)

        # (2) Distance 모드만
        conveyor_box = QGroupBox("컨베이어 제어")
        conveyor_layout = QHBoxLayout()
        conveyor_box.setLayout(conveyor_layout)
        bottom_layout.addWidget(conveyor_box)

        # Distance Spin
        self.distance_label = QLabel("Distance:")
        self.distance_spin = QDoubleSpinBox()
        self.distance_spin.setRange(0, 100000)
        self.distance_spin.setValue(200.0)
        self.distance_spin.setSuffix(" mm")

        conveyor_layout.addWidget(self.distance_label)
        conveyor_layout.addWidget(self.distance_spin)

        # Start/Stop 버튼
        self.start_btn = QPushButton("Start")
        self.stop_btn = QPushButton("Stop")
        self.start_btn.clicked.connect(self.on_start_clicked)
        self.stop_btn.clicked.connect(self.on_stop_clicked)
        conveyor_layout.addWidget(self.start_btn)
        conveyor_layout.addWidget(self.stop_btn)

        # (3) 사용자 명령 목록 버튼
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
        bottom_layout.addLayout(job_layout)

        # (4) 로그창
        self.status_text = QTextEdit()
        self.status_text.setReadOnly(True)
        bottom_layout.addWidget(self.status_text)

        # 내부 변수
        self.run_start_time = None
        self.requested_distance = 0.0
        self.estimated_speed_mm_s = node.estimated_speed_mm_s

        # 시그널 연결
        self.node.image_signal.connect(self.update_image)
        self.node.status_signal.connect(self.on_conveyor_status)


    # ---------------------------------------------------
    # ONE CLICK 버튼 콜백 (필요시 추가 로직)
    # ---------------------------------------------------
    def on_oneclick_button(self):
        now = time.strftime("%H:%M:%S")
        self.status_text.append(f"[{now}] ONE CLICK 버튼 눌림!")

    # ---------------------------------------------------
    # 작업 모드별 Play/Stop 콜백
    # ---------------------------------------------------
    def on_work_play(self):
        self.current_state_label.setText("현재 상태: step1(작업장 이동)")
    def on_work_stop(self):
        self.current_state_label.setText("현재 상태: Rest")

    def on_classify_play(self):
        self.current_state_label.setText("현재 상태: step2(분류)")
    def on_classify_stop(self):
        self.current_state_label.setText("현재 상태: Rest")

    def on_pickup_play(self):
        self.current_state_label.setText("현재 상태: step3(픽업 이동)")
    def on_pickup_stop(self):
        self.current_state_label.setText("현재 상태: Rest")

    def on_deliver_play(self):
        self.current_state_label.setText("현재 상태: step4(배달)")
    def on_deliver_stop(self):
        self.current_state_label.setText("현재 상태: Rest")

    # ---------------------------------------------------
    # job1,2,3 버튼 -> 메시지 발행 + 로그 출력
    # ---------------------------------------------------
    def on_job1_clicked(self):
        cmd = {'red':2, 'blue':1, 'goal':1}
        self.node.publish_order(cmd)
        now = time.strftime("%H:%M:%S")
        self.status_text.append(f"[{now}] job1 clicked -> {cmd}")

    def on_job2_clicked(self):
        cmd = {'red':1, 'blue':2, 'goal':2}
        self.node.publish_order(cmd)
        now = time.strftime("%H:%M:%S")
        self.status_text.append(f"[{now}] job2 clicked -> {cmd}")

    def on_job3_clicked(self):
        cmd = {'red':1, 'blue':0, 'goal':3}
        self.node.publish_order(cmd)
        now = time.strftime("%H:%M:%S")
        self.status_text.append(f"[{now}] job3 clicked -> {cmd}")

    # ---------------------------------------------------
    # 카메라 영상 업데이트 (노란 십자가 유지)
    # ---------------------------------------------------
    def update_image(self, qimg: QImage):
        pixmap = QPixmap.fromImage(qimg)
        # 라벨 크기에 맞춰 영상 스케일링 (640x360)
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
        painter.drawLine(cx, 0, cx, h)
        painter.drawLine(0, cy, w, cy)
        painter.end()
        self.image_label.setPixmap(scaled_pixmap)

    # ---------------------------------------------------
    # 컨베이어 상태 콜백
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
            # 동작 시간 출력
            if self.run_start_time is not None:
                elapsed = time.time() - self.run_start_time
                self.status_text.append(f"   => 동작시간: {elapsed:.2f}초")

            self.run_start_time = None

        elif status_str == "DISCONNECT":
            self.start_btn.setEnabled(False)
            self.stop_btn.setEnabled(False)
        else:
            # 기타 (INIT 등)
            self.start_btn.setEnabled(False)
            self.stop_btn.setEnabled(False)

    # ---------------------------------------------------
    # 컨베이어 Start/Stop (Distance만)
    # ---------------------------------------------------
    def on_start_clicked(self):
        dist = self.distance_spin.value()
        cmd = {"control": "go", "mode": "distance", "distance.mm": dist}
        msg = f"Start Distance: {dist:.2f} mm"

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
