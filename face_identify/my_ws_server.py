import rclpy
from rclpy.node import Node
from originbot_face_identify_msgs.msg import FaceDetection
from websocket_server import WebsocketServer
import json
import threading

class FaceDetectionWebSocketServer(Node):
    def __init__(self):
        super().__init__('face_detection_websocket_server')
        self.face_sub = self.create_subscription(
            FaceDetection,
            'face_detection',
            self.face_detection_callback,
            10)
        self.ws_server = WebsocketServer(host='0.0.0.0', port=9001)
        self.ws_thread = threading.Thread(target=self.ws_server.run_forever)
        self.ws_thread.daemon = True
        self.ws_thread.start()
        self.get_logger().info("WebSocket server started on port 9001")

    def face_detection_callback(self, msg: FaceDetection):
        faces = []
        for i in range(len(msg.x)):
            face = {
                'x': msg.x[i],
                'y': msg.y[i],
                'width': msg.width[i],
                'height': msg.height[i],
                'score': msg.score[i],
            }
            faces.append(face)
        message = json.dumps(faces)
      
        self.ws_server.send_message_to_all(message)

def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectionWebSocketServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
