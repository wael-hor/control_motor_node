import rclpy
from rclpy.node import Node
import cv2
from http.server import BaseHTTPRequestHandler, HTTPServer
from threading import Thread  # To properly run HTTP server in a separate thread

class CameraStreamer(Node):
    def __init__(self):
        super().__init__('camera_streamer')
        self.camera_index = 0  # Change if needed to match your camera
        self.cap = cv2.VideoCapture(self.camera_index)

        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open camera at index {self.camera_index}")
            return
        self.get_logger().info("Camera opened successfully")

        # Initialize MJPEG Server
        self.server = HTTPServer(('0.0.0.0', 8080), self.MJPEGHandler)
        self.server.node = self  # Attach the ROS2 node instance to the server

        # Start HTTP server in a separate thread
        self.server_thread = Thread(target=self.server.serve_forever, daemon=True)
        self.server_thread.start()
        self.get_logger().info("MJPEG server started at http://0.0.0.0:8080/stream.mjpg")

    class MJPEGHandler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == '/stream.mjpg':
                self.send_response(200)
                self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=--jpgboundary')
                self.end_headers()
                while True:
                    ret, frame = self.server.node.cap.read()
                    if not ret:
                        self.server.node.get_logger().warn("Failed to capture frame")
                        break
                    frame = cv2.resize(frame, (320, 240))  # Resize frame for performance
                    _, jpg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                    self.wfile.write(b'--jpgboundary\r\n')
                    self.send_header('Content-type', 'image/jpeg')
                    self.send_header('Content-length', len(jpg))
                    self.end_headers()
                    self.wfile.write(jpg.tobytes())
                    self.wfile.write(b'\r\n')

    def destroy_node(self):
        self.cap.release()
        self.server.shutdown()  # Ensure proper shutdown of the server
        self.server.server_close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraStreamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
