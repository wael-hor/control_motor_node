import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import RPi.GPIO as GPIO
import socket
import threading
import time
import math

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.left_steps_pub = self.create_publisher(Int32, '/left_steps', 10)
        self.right_steps_pub = self.create_publisher(Int32, '/right_steps', 10)
        self.timer = self.create_timer(0.1, self.update_motors)

        # Set up GPIO using BCM numbering
        GPIO.setmode(GPIO.BCM)
        self.LEFT_STEP_PIN = 17
        self.LEFT_DIR_PIN = 18
        self.RIGHT_STEP_PIN = 22
        self.RIGHT_DIR_PIN = 23

        # Set up GPIO pins as outputs
        GPIO.setup(self.LEFT_STEP_PIN, GPIO.OUT)
        GPIO.setup(self.LEFT_DIR_PIN, GPIO.OUT)
        GPIO.setup(self.RIGHT_STEP_PIN, GPIO.OUT)
        GPIO.setup(self.RIGHT_DIR_PIN, GPIO.OUT)

        # Initialize pins to low
        GPIO.output(self.LEFT_STEP_PIN, GPIO.LOW)
        GPIO.output(self.LEFT_DIR_PIN, GPIO.LOW)
        GPIO.output(self.RIGHT_STEP_PIN, GPIO.LOW)
        GPIO.output(self.RIGHT_DIR_PIN, GPIO.LOW)

        self.wheel_diameter = 0.1
        self.steps_per_revolution = 200
        self.distance_per_step = (math.pi * self.wheel_diameter) / self.steps_per_revolution
        self.wheel_base = 0.3

        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.total_left_steps = 0
        self.total_right_steps = 0
        self.manual_control = False

        socket_thread = threading.Thread(target=self.run_socket_server)
        socket_thread.daemon = True
        socket_thread.start()

        self.get_logger().info("Motor Control Node with Socket Server started")

    def cmd_vel_callback(self, msg):
        if not self.manual_control:
            self.linear_vel = msg.linear.x
            self.angular_vel = msg.angular.z

    def run_socket_server(self):
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind(('0.0.0.0', 12345))
        server_socket.listen(5)
        self.get_logger().info("Socket server listening on port 12345")

        while True:
            try:
                client_socket, addr = server_socket.accept()
                self.get_logger().info(f"Client connected: {addr}")

                while True:
                    data = client_socket.recv(1024).decode().strip()
                    if not data:
                        self.get_logger().info("Client disconnected")
                        break

                    command = data
                    self.get_logger().info(f"Received command: {command}")

                    if command == "forward":
                        self.manual_control = True
                        self.linear_vel = 0.1
                        self.angular_vel = 0.0
                    elif command == "backward":
                        self.manual_control = True
                        self.linear_vel = -0.1
                        self.angular_vel = 0.0
                    elif command == "left":
                        self.manual_control = True
                        self.linear_vel = 0.0
                        self.angular_vel = 0.5
                    elif command == "right":
                        self.manual_control = True
                        self.linear_vel = 0.0
                        self.angular_vel = -0.5
                    elif command == "stop":
                        self.manual_control = True
                        self.linear_vel = 0.0
                        self.angular_vel = 0.0
                    elif command == "auto":
                        self.manual_control = False
                        self.linear_vel = 0.0
                        self.angular_vel = 0.0
                    else:
                        self.get_logger().warn(f"Unknown command: {command}")

                    response = f"Command received: {command}\n"
                    client_socket.send(response.encode())
            except Exception as e:
                self.get_logger().error(f"Socket server error: {str(e)}")
            finally:
                client_socket.close()

    def update_motors(self):
        left_vel = self.linear_vel - (self.angular_vel * self.wheel_base / 2.0)
        right_vel = self.linear_vel + (self.angular_vel * self.wheel_base / 2.0)

        left_steps_per_sec = left_vel / self.distance_per_step
        right_steps_per_sec = right_vel / self.distance_per_step

        left_steps = int(left_steps_per_sec * 0.1)
        right_steps = int(right_steps_per_sec * 0.1)

        self.total_left_steps += left_steps
        self.total_right_steps += right_steps

        left_msg = Int32()
        left_msg.data = self.total_left_steps
        right_msg = Int32()
        right_msg.data = self.total_right_steps
        self.left_steps_pub.publish(left_msg)
        self.right_steps_pub.publish(right_msg)

        self.send_steps(self.LEFT_STEP_PIN, self.LEFT_DIR_PIN, left_steps)
        self.send_steps(self.RIGHT_STEP_PIN, self.RIGHT_DIR_PIN, right_steps)

        self.get_logger().info(f"Steps: left={left_steps}, right={right_steps}")

    def send_steps(self, step_pin, dir_pin, steps):
        if steps == 0:
            return

        GPIO.output(dir_pin, GPIO.HIGH if steps > 0 else GPIO.LOW)
        steps = abs(steps)

        for _ in range(steps):
            GPIO.output(step_pin, GPIO.HIGH)
            time.sleep(0.0005)
            GPIO.output(step_pin, GPIO.LOW)
            time.sleep(0.0005)

    def shutdown(self):
        GPIO.cleanup()
        self.get_logger().info("GPIO pins cleaned up")

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()