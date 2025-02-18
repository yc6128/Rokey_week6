# 2025-02-18
# usb 카메라로 비디오 받아서 
# undistort
# CompressedImage로 발행
# yolo 동작

# ---------해야 하는 거-----------
# 박스위치 가져오기

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

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

        # yolo pt file
        # self.model = YOLO('/home/rokey2/bag_c/test_ws/best_aml.pt')
        self.model = YOLO('yolov8s.pt')

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

        results = self.model(frame)

        for result in results:
            frame = result.plot()
            print('-----------------')
            for box in result.boxes.xyxy:
                center_x = (box[2] - box[0])/2
                center_y = (box[3] - box[1])/2
                label_text = f'x : {center_x:.2f}, y : {center_y:.2f}'
                cv2.putText(frame, label_text, (int(center_x), int(center_y)),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

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
