#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray

class Subscriber(Node):
    """
    ROS2 YOLOv5 구독자 노드 클래스.

    이 클래스는 'detections' 토픽을 구독하고, 수신된 메시지를
    listener_callback 메서드를 통해 처리합니다.
    """
    def __init__(self):
        # 노드 이름을 'subscribe'로 초기화합니다
        super().__init__('subscribe')
        
        self.subscription = self.create_subscription(
            Detection2DArray,         # 메시지 타입
            '/yolo_result',           # 토픽 이름 (실제 발행되는 이름)
            self.listener_callback,   # 메시지 수신 시 호출할 함수
            10)
        self.subscription

    def listener_callback(self, msg):
        # TODO: Implemnt a callback
        """
        /yolo_result 토픽에서 메시지를 수신했을 때 실행되는 콜백 함수
        """
        # 감지된 각 객체에 대한 정보를 로그로 출력합니다.
        for detection in msg.detections:
            # results 배열이 비어있는 경우를 대비합니다.
            if not detection.results:
                continue

            # 객체의 클래스(class_id)와 신뢰도 점수(score)를 가져옵니다.
            class_id = detection.results[0].hypothesis.class_id
            score = detection.results[0].hypothesis.score

            # 요구사항에 맞게 화면에 로그를 출력합니다.
            self.get_logger().info(f'Detected: class={class_id}, score={score:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = Subscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
