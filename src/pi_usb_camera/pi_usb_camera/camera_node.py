import rclpy
from rclpy.node import Node
import cv2
from http.server import BaseHTTPRequestHandler, HTTPServer
import time

class CameraStreamer(Node):
    def __init__(self):
        super().__init__('camera_streamer')
        self.camera_index = 0  # Adjust if needed (e.g., 2 if previously identified)
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open camera at index {self.camera_index}")
            return
        self.get_logger().info("Camera opened successfully")

        # Start MJPEG server
        self.server = HTTPServer(('0.0.0.0', 8080), self.MJPEGHandler)
        self.server.node = self  # Attach the node to the server for access in the handler
        self.server_thread = self.create_timer(0.01, self.run_server)

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
                    frame = cv2.resize(frame, (320, 240))  # Reduce resolution for better performance
                    ret, jpg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                    if not ret:
                        self.server.node.get_logger().warn("Failed to encode frame")
                        continue
                    self.wfile.write(b'--jpgboundary\r\n')
                    self.send_header('Content-type', 'image/jpeg')
                    self.send_header('Content-length', len(jpg))
                    self.end_headers()
                    self.wfile.write(jpg)
                    self.wfile.write(b'\r\n')
                    time.sleep(0.05)  # Adjust for ~20 FPS

    def run_server(self):
        self.get_logger().info("MJPEG server started at http://192.168.137.5:8080/stream.mjpg")
        self.server.handle_request()

    def destroy_node(self):
        self.cap.release()
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
