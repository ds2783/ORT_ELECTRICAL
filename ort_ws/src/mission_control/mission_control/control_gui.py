import re
from qreader import QReader
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
from mission_control.config.network import COMM_PORT, PORT_MAIN, PORT_SECONDARY, PI_IP, tofQoS
from mission_control.config.gui import CALLIBRATE_IMU, WIDTH, HEIGHT, CALLIBRATE_IMU, ZERO_AXIS 

from threading import Thread
from multiprocessing.connection import Listener

# Messages ---
from ort_interfaces.action import CalibrateImu
from std_msgs.msg import Bool, Float32




def impl_glfw_init(window_name="Project Gorgon", width=WIDTH, height=HEIGHT):
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

        self.reset_pos_pub_ = self.create_publisher(Bool, "/elysium/reset_pos", 10)
        self.led_pub_ = self.create_publisher(Float32, "/led", tofQoS)
    
    def publish_led(self, slider_float: float):
        msg = Float32()
        msg.data = slider_float
        self.led_pub_.publish(msg)

    def send_goal(self, code):
        goal_msg = CalibrateImu.Goal()
        if code == ZERO_AXIS:
            msg = Bool()
            msg.data = True
            self.reset_pos_pub_.publish(msg)
        goal_msg.code = code

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

        # QR 
        self.last_qr_ = "None"
        
        # Calbration Client
        self.client_ = GuiClient()

        # Position
        self.elysium_x = "0"
        self.elysium_y = "0"
        self.elysium_z = "0"

        # Attitude
        self.elysium_yaw = "0"
        self.elysium_pitch = "0"
        self.elysium_roll = "0"

        # Twist
        self.elysium_x_vel = "0"
        self.elysium_y_vel = "0"
        self.elysium_z_vel = "0"

        # Camera Rotation
        self.camera_yaw = "0"
        self.camera_pitch = "0"

        # tof
        self.q_tof = "0"
        self.o_tof = "0"

        # IR LED
        self.led = 0.0
        self.current_value = 0.0
        
        self.address = ("localhost", port)  # family is deduced to be 'AF_INET'
        self.listener = Listener(self.address, authkey=b"123")
        
        self.conn = self.listener.accept()
        comms_thread = Thread(target=self.qrComms)
        comms_thread.start()


    def qrComms(self):
        while True:
            recieved_msg = self.conn.recv()
            if recieved_msg is not None:
                data = str(recieved_msg)[6:]
                match recieved_msg[:5]:
                    case "qr---":
                        self.last_qr_ = data
                    case "x----":
                        self.elysium_x = data
                    case "y----":
                        self.elysium_y = data
                    case "z----":
                        self.elysium_z = data
                    case "yaw--":
                        self.elysium_yaw = data
                    case "pitch":
                        self.elysium_pitch = data
                    case "roll-":
                        self.elysium_roll = data
                    case "x_vel":
                        self.elysium_x_vel = data
                    case "y_vel":
                        self.elysium_y_vel = data 
                    case "z_vel":
                        self.elysium_z_vel = data
                    case "cam_y":
                        self.camera_yaw = data
                    case "cam_p":
                        self.camera_pitch = data
                    case "q-tof":
                        self.q_tof = data
                    case "o-tof":
                        self.o_tof = data

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

        if imgui.button("Calibrate IMU"):
            self.client_.send_goal(CALLIBRATE_IMU)

        elif imgui.button("Zero Axis"):
            self.client_.send_goal(ZERO_AXIS)

        imgui.end()

        imgui.begin("Telemetry")
        imgui.text("All data is in degrees.")
        imgui.text(
        f"""
        Yaw: {self.elysium_yaw}
        Pitch: {self.elysium_pitch} 
        Roll: {self.elysium_roll}
        """
        )
        imgui.text(
        f""" 
        x: {self.elysium_x} 
        y: {self.elysium_y}
        z: {self.elysium_z}
        """
        )
        imgui.text(
        f"""
        x_vel: {self.elysium_x_vel}
        y_vel: {self.elysium_y_vel}
        z_vel: {self.elysium_z_vel}
        """
        )
        imgui.text(
        f"""
        camera-yaw: {self.camera_yaw}
        camera-pitch: {self.camera_pitch}
        """
                )
        imgui.text(
        f"""
        bottom-dist: {self.o_tof}
        camera-dist: {self.q_tof}
        """
                )
        imgui.end()
        
        imgui.begin("IR Cam")
        changed, self.current_value = imgui.slider_float(
        "IR Cam", self.current_value,
        min_value=0.0, max_value=100.0,
        format="%.0f"
        )
        imgui.text("Changed: %s, Value: %s" % (changed, self.current_value))
        if self.led != self.current_value:
            self.led = self.current_value
            self.client_.publish_led(self.led/100)
            
        imgui.end()

        # Display Testing Window
        # imgui.show_test_window()

        imgui.render()

        self.impl.render(imgui.get_draw_data())
        glfw.swap_buffers(self.window)


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
    rclpy.try_shutdown()
