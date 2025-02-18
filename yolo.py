# 2025-02-18
# usb 카메라로 비디오 받아서 
# undistort
# CompressedImage로 발행

# ---------해야 하는 거-----------
# 카메라 영상 보내주는 노드 새로 만들기
# 영상 받아오는걸로 수정

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class Yolo(Node):
    def __init__(self):
        super().__init__('yolo')
        
        # 이미지 퍼블리셔 생성
        # self.publisher_ = self.create_publisher(CompressedImage, 'image_raw/compressed', 10)
        self.publisher_low_ = self.create_publisher(CompressedImage, 'image_raw/compressed_low', 10)

        # 이미지 서브스크라이버 생성
        self.subscription_rgb = self.create_subscription(
            CompressedImage,
            'image_raw/compressed',
            #'image_raw/compressed_low',
            self.listener_callback_rgb,
            10)
        self.subscription_rgb  # prevent unused variable warning        
        
        # OpenCV와 ROS 간 변환을 위한 CvBridge 초기화
        self.bridge = CvBridge()

        # 주기적인 이미지 전송을 위한 타이머 설정 (주기: 1초)
        # self.timer = self.create_timer(0.1, self.publish_image)

        # # OpenCV 비디오 캡처 객체 생성 (카메라 0번 장치 사용)
        # self.cap = cv2.VideoCapture(2, cv2.CAP_V4L2)
        # # self.cap = cv2.VideoCapture(2)
        # # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        # # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        # self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        # self.cap.set(cv2.CAP_PROP_FPS, 5)
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        # 이미 얻어진 distortion, 비틀어짐 계산 결과값

        self.mtx = np.array([[1.37275781e+03, 0.00000000e+00, 7.22150428e+02],
        [0.00000000e+00, 1.37374056e+03, 3.95908524e+02],
        [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

        self.dist =  np.array([[-2.69454647e-02,  5.20703990e-01, -1.14129861e-02,
         -9.54622834e-04, -1.42495907e+00]])
        
        
        #print(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH), self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    def get_calibrated_image(self, fname, mtx, dist):
        # img = cv2.imread(fname)
        img = fname
        h,  w = img.shape[:2]
        ####################################################################3    
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

        # undistort
        dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
     
        # crop the image
        # x, y, w, h = roi
        # dst = dst[y:y+h, x:x+w]

        return img, dst
    
    def listener_callback_rgb(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # Decode to color image

        # undistort
        _, frame = self.get_calibrated_image(image_np, self.mtx, self.dist)

        # # OpenCV 이미지 (BGR)을 JPEG로 압축
        # encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]  # 90은 압축 품질
        # _, compressed_image = cv2.imencode('.jpg', frame, encode_param)

        # # 압축된 이미지를 CompressedImage 메시지로 변환
        # msg = CompressedImage()
        # msg.header.stamp = self.get_clock().now().to_msg()  # 타임스탬프 추가
        # msg.header.frame_id = "camera"  # 프레임 ID 설정
        # msg.format = "jpeg"  # 압축 형식 설정
        # msg.data = compressed_image.tobytes()  # 압축된 이미지 데이터

        # # CompressedImage 퍼블리시
        # self.publisher_.publish(msg)



        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 30]  # 90은 압축 품질
        _, compressed_image_low = cv2.imencode('.jpg', cv2.resize(frame,(640,360)), encode_param)
        msg.data = compressed_image_low.tobytes()  # 압축된 이미지 데이터
        self.publisher_low_.publish(msg)

        #self.get_logger().info('Publishing compressed image... {%d}, {%d}' % (len(compressed_image), len(compressed_image_low)))


def main(args=None):
    rclpy.init(args=args)
    image_publisher = Yolo()
    
    # ROS 2 노드 실행
    rclpy.spin(image_publisher)

    # 종료 시 리소스 해제
    image_publisher.cap.release()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
