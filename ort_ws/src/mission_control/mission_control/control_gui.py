import rclpy
from rclpy import executors
from rclpy.node import Node
from rclpy.action.client import ActionClient

import imgui.core as imgui
import glfw
import OpenGL.GL as gl
from imgui.integrations.glfw import GlfwRenderer

from mission_control.gui.dashboard import Dashboard
from mission_control.stream.stream_client import StreamClient
from mission_control.config.network import COMM_PORT, PORT_MAIN, PORT_SECONDARY, PI_IP

from threading import Thread
from multiprocessing.connection import Listener

import numpy as np

# Messages ---
from ort_interfaces.action import CalibrateImu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3


ACCEL_GRYO = 0
MAG = 1
ZERO_AXIS = 2


def impl_glfw_init(window_name="Project Gorgon", width=2200, height=1300):
    if not glfw.init():
        print("Could not initialize OpenGL context")
        exit(1)

    # OS X supports only forward-compatible core profiles from 3.2
    glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
    glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
    glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)

    glfw.window_hint(glfw.OPENGL_FORWARD_COMPAT, gl.GL_TRUE)

    # Create a windowed mode window and its OpenGL context
    window = glfw.create_window(int(width), int(height), window_name, None, None)
    glfw.make_context_current(window)

    if not window:
        glfw.terminate()
        print("Could not initialize Window")
        exit(1)

    return window


class GuiClient(Node):
    def __init__(self):
        super().__init__("gui_client")
        self.calibration_client_ = ActionClient(self, CalibrateImu, "/imu/calibrate")
        self.current_step = None

        # Topics -----------
        self.euler_angles_pub_ = self.create_subscription(
            Vector3, "/elysium/euler_angles", self.eulerCB_, 10
        )
        self.odom_pub_ = self.create_subscription(
            Odometry, "/elysium/odom", self.odomCB_, 10
        )
        # ------------------

        # Messages
        self.eulerAngles = Vector3()
        self.odom = Odometry()

    def send_goal(self, code):
        goal_msg = CalibrateImu.Goal()
        goal_msg.code = code

        # self.calibration_client_.wait_for_server()

        self._send_goal_future = self.calibration_client_.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Result: {0}".format(result.result))

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.current_step = feedback
        self.get_logger().info("Received feedback: {0}".format(feedback.seconds))

    def eulerCB_(self, msg: Vector3):
        self.eulerAngles = msg

    def odomCB_(self, msg: Odometry):
        self.odom = msg


class GUI(Node):
    def __init__(self, cams, port):
        super().__init__("control_gui")
        self.backgroundColor = (0, 0, 0, 1)
        self.window = impl_glfw_init()
        gl.glClearColor(*self.backgroundColor)
        imgui.create_context()
        self.impl = GlfwRenderer(self.window)

        # REPLACE THESE PATHS WITH ABSOLUTE PATH OF SHADERS ON INSTALL
        # alternative place all shader code with string in a python file
        self.dashboard = Dashboard(cams)

        self.address = ("localhost", port)  # family is deduced to be 'AF_INET'
        self.listener = Listener(self.address, authkey=b"123")
        self.last_qr_ = "None"

        self.qr_conn = self.listener.accept()
        comms_thread = Thread(target=self.qrComms)
        comms_thread.start()

        self.client_ = GuiClient()

    def qrComms(self):
        while True:
            recieved_qr = self.qr_conn.recv()
            if recieved_qr is not None:
                self.last_qr_ = str(recieved_qr)

    def run(self):
        glfw.poll_events()
        self.impl.process_inputs()
        gl.glClearColor(*self.backgroundColor)
        gl.glClear(gl.GL_COLOR_BUFFER_BIT)

        imgui.get_io().font_global_scale = 1.2

        self.dashboard.draw()
        imgui.new_frame()

        imgui.begin("QR-Display", True)
        imgui.set_window_size(180, 80)
        imgui.text(self.last_qr_)
        imgui.end()

        imgui.begin("Performance")
        io = imgui.get_io()
        imgui.text(f"FPS: {io.framerate:.2f}")
        imgui.end()

        imgui.begin("CalibrateImu")
        imgui.text(str(self.client_.current_step))
        imgui.set_window_size(180, 80)

        if imgui.button("Calibrate Accelerometer & Gyro"):
            self.client_.send_goal(ACCEL_GRYO)

        elif imgui.button("Calibrate Magnometer"):
            self.client_.send_goal(MAG)

        elif imgui.button("Zero Axis"):
            self.client_.send_goal(ZERO_AXIS)

        imgui.end()

        imgui.begin("Telemetry")
        imgui.text(
            f"""
            Yaw: {rad_degrees(self.client_.eulerAngles.x):.2f}
            Pitch: {rad_degrees(self.client_.eulerAngles.y):.2f} 
            Roll: {rad_degrees(self.client_.eulerAngles.z):.2f}
            """
        )
        imgui.text(
            f""" 
            x: {self.client_.odom.pose.pose.position.x:.2f} 
            y: {self.client_.odom.pose.pose.position.y:.2f}
            z: {self.client_.odom.pose.pose.position.z:.2f}
            """
        )
        imgui.text(
            f"""
            x_vel: {self.client_.odom.twist.twist.linear.x:.2f}
            y_vel: {self.client_.odom.twist.twist.linear.y:.2f}
            z_vel: {self.client_.odom.twist.twist.linear.z:.2f}
            """
        )
        imgui.end()

        # Display Testing Window
        # imgui.show_test_window()

        imgui.render()

        self.impl.render(imgui.get_draw_data())
        glfw.swap_buffers(self.window)


def rad_degrees(num):
    return (180 / np.pi) * num

def main(args=None):
    rclpy.init(args=args)

    cams = [
        StreamClient("Stereo", PI_IP, "udp", PORT_MAIN, 640, 480, stereo=False),
        StreamClient("Stereo", PI_IP, "udp", PORT_SECONDARY, 640, 480, stereo=False),
    ]
    gui = GUI(cams, COMM_PORT)

    # DON'T BOTH WITH EXECUTORS
    # ONLY USING ROS NODE WRAPPER FOR LAUNCH FILE
    executor = executors.MultiThreadedExecutor()
    executor.add_node(gui.client_)

    node_thread = Thread(target=executor.spin)
    node_thread.start()

    # Run GUI
    while not glfw.window_should_close(gui.window):
        gui.run()

    # Cleanup After Shutdown
    gui.impl.shutdown()
    glfw.terminate()
    rclpy.shutdown()
