# 2025-02-17
# 메시지 발행 노드
# 일단은 한번만 스핀

import rclpy
from rclpy.node import Node
from messages.msg import Control

class pub(Node):
    def __init__(self):
        super().__init__('conveyor_node')
        # 명령 발행 {"control":"go","distance.mm":200}
        self.order_publihs = self.create_publisher(Control, '/control', 10)
        self.timer = self.create_timer(0.1,self.timer_callback)
        print('-----')

    def timer_callback(self):
        
        message = Control()
        
        message.control = 'go'
        
        message.distance = 200
        self.order_publihs.publish(message)
        

def main(args=None):
    rclpy.init(args=args)
    conveyor_node = pub()

    try:
        rclpy.spin(conveyor_node)
    except:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
