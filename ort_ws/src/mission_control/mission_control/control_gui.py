import rclpy
from rclpy import executors
from rclpy.node import Node
import rclpy.utilities
from rclpy.action.client import ActionClient

import OpenGL.GL as gl
import glfw
import imgui.core as imgui
from imgui.integrations.glfw import GlfwRenderer
import colorsys

from mission_control.gui.dashboard import Dashboard
from mission_control.stream.stream_client import StreamClient
from mission_control.config.network import (
    COMM_PORT,
    PORT_MAIN,
    PORT_SECONDARY,
    PI_IP,
    tofQoS,
)
from mission_control.config.gui import (
    CALLIBRATE_IMU,
    QR_DIRECTORY,
    WIDTH,
    HEIGHT,
    CALLIBRATE_IMU,
    ZERO_AXIS,
)

from threading import Thread
from multiprocessing.connection import Listener
import json
from PIL import Image
import numpy as np

# Messages ---
from std_msgs.msg import Bool, Float32
from ort_interfaces.action import CalibrateImu


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
        glfw.set_window_refresh_callback(self.window, self.window_refresh_CB_)
        glfw.set_framebuffer_size_callback(self.window, self.frame_buffer_size_CB_)

        gl.glClearColor(*self.backgroundColor)
        imgui.create_context()
        self.impl = GlfwRenderer(self.window)

        # Window Size
        self.width = WIDTH
        self.height = HEIGHT

        # REPLACE THESE PATHS WITH ABSOLUTE PATH OF SHADERS ON INSTALL
        # alternative place all shader code with string in a python file
        self.dashboard = Dashboard(cams, self.get_logger)

        # QR
        self.last_qr_ = "None"
        self.qr_dict_ = {}
        self.qr_texID = gl.glGenTextures(1)

        # Calbration Client
        self.client_ = GuiClient()
        self.logger = self.client_.get_logger

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

        # Battery
        self.soc = 0.5

        # IR LED
        self.led = 0.0
        self.current_value = 100.0

        self.address = ("localhost", port)  # family is deduced to be 'AF_INET'
        self.listener = Listener(self.address, authkey=b"123")

        self.listen = True
        self.conn = self.listener.accept()
        self.comms_thread = Thread(target=self.qrComms)
        self.comms_thread.start()

        # IMGUI
        imgui.get_io().font_global_scale = 1.2
        self.style = imgui.get_style()
        self.style.window_rounding = 3.0

    def window_refresh_CB_(self, window):
        self.run()
        gl.glFinish()

    def frame_buffer_size_CB_(self, window, width, height):
        gl.glViewport(0, 0, width, height)
        self.width = width
        self.height = height

        scale_change = 0.5 * (((width) / (WIDTH) * 1.2) + ((height) / (HEIGHT) * 1.2))

        ARBRITRARY_SCALE = 0.4
        imgui.get_io().font_global_scale = (
            1.2 - ARBRITRARY_SCALE + scale_change * ARBRITRARY_SCALE
        )

    def shutdown(self):
        self.listen = False
        self.comms_thread.join()
        self.destroy_node()

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
                    case "soc--":
                        self.soc = float(data)
                    case "qrdic":
                        self.get_logger().info("Recieved JSON file from Base Station.")
                        self.qr_dict_ = json.loads(data)

    def bind_image(self, img):
        image = np.array(img)
        gl.glBindTexture(gl.GL_TEXTURE_2D, self.qr_texID)

        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MIN_FILTER, gl.GL_NEAREST)
        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MAG_FILTER, gl.GL_LINEAR)

        # Set texture clamping method
        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_S, gl.GL_REPEAT)
        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_T, gl.GL_REPEAT)

        gl.glTexImage2D(
            gl.GL_TEXTURE_2D,
            0,
            gl.GL_RGB,
            image.shape[1],
            image.shape[0],
            0,
            gl.GL_RGB,
            gl.GL_UNSIGNED_BYTE,
            image,
        )

    def run(self):
        glfw.poll_events()
        self.impl.process_inputs()
        gl.glClearColor(*self.backgroundColor)
        gl.glClear(gl.GL_COLOR_BUFFER_BIT)

        self.dashboard.draw()

        # IMGUI - BEGIN ---
        imgui.new_frame()

        # PERFORMANCE
        imgui.begin("Performance")
        io = imgui.get_io()
        imgui.text(f"FPS: {io.framerate:.2f}")
        imgui.end()

        # MAIN DASHBOARD
        offset = self.width / 40
        imgui.set_next_window_position(self.width / 2 + offset, 0)
        imgui.set_next_window_size(self.width / 2 - offset, self.height)

        imgui.begin(
            "Dashboard",
            flags=imgui.WINDOW_NO_COLLAPSE
            | imgui.WINDOW_NO_MOVE
            | imgui.WINDOW_NO_RESIZE
            | imgui.WINDOW_MENU_BAR
            | imgui.WINDOW_NO_BRING_TO_FRONT_ON_FOCUS,
        )

        # TELEMETRY
        imgui.begin_group()
        imgui.text("Telemetry:")
        imgui.text("(All data is in degrees)\n\n")
        imgui.text(
            f"Yaw: {self.elysium_yaw}\nPitch: {self.elysium_pitch}\nRoll: {self.elysium_roll}\n"
        )
        imgui.text(f"x: {self.elysium_x}\ny: {self.elysium_y}\nz: {self.elysium_z}\n")
        imgui.text(
            f"x_vel: {self.elysium_x_vel}\ny_vel: {self.elysium_y_vel}\nz_vel: {self.elysium_z_vel}\n"
        )
        imgui.text(
            f"camera-yaw: {self.camera_yaw}\ncamera-pitch: {self.camera_pitch}\n"
        )
        imgui.text(f"bottom-dist: {self.o_tof}\ncamera-dist: {self.q_tof}\n")
        imgui.end_group()

        imgui.same_line(position=self.width * 9 / 32)

        # BATTERY MONITOR
        imgui.begin_group()
        imgui.text("Battery Readout:")
        # from 0 to 255 -> OpenGL and ImGui uses float values to conversion is needed
        # Magic Colours ->
        low_bat_colour = (181, 56, 56)
        high_bat_colour = (31, 161, 91)

        batt_colour_rgb = colour_interpolate(
            low_bat_colour, high_bat_colour, self.soc, linear
        )
        display_batt_colour = normalise_rgb(batt_colour_rgb)

        imgui.push_style_color(
            imgui.COLOR_PLOT_HISTOGRAM,
            *display_batt_colour,
        )
        imgui.progress_bar(self.soc, (self.width / 14, 18 + 1 / 200 * self.height), "")
        imgui.pop_style_color(1)
        imgui.same_line()
        imgui.text(f"{self.soc * 100}%")

        imgui.new_line()

        # IR LIGHT
        imgui.begin_child("IR Light", self.width / 6, self.height / 15, True)
        changed, self.current_value = imgui.slider_float(
            "IR Light",
            self.current_value,
            min_value=0.0,
            max_value=100.0,
            format="%.1f",
        )
        if self.led != self.current_value:
            self.led = self.current_value
            self.client_.publish_led(self.led / 100)
        imgui.end_child()

        # CALIBRATION CLIENT
        imgui.begin_child("Calibration Client", self.width / 6, self.height / 20, True)
        if imgui.button("Calibrate Rover"):
            imgui.open_popup("Calibration Client")
        if imgui.begin_popup_modal("Calibration Client").opened:
            imgui.text("Feedback: " + str(self.client_.current_step))

            if imgui.button("Calibrate IMU"):
                self.client_.send_goal(CALLIBRATE_IMU)

            elif imgui.button("Zero Axis"):
                self.client_.send_goal(ZERO_AXIS)

            elif imgui.button("Close Client"):
                imgui.close_current_popup()
            imgui.end_popup()
        imgui.end_child()

        # QR DISPLAY
        imgui.begin_child("QR-Display", self.width / 6, self.height / 10, True)
        imgui.text("QR-Display:")
        imgui.text(self.last_qr_)
        imgui.end_child()

        imgui.end_group()

        imgui.spacing()
        imgui.spacing()
        imgui.spacing()

        # QR LIST
        imgui.begin_child("QR-List", self.width / 2 - offset, self.height / 4, True)
        for key in self.qr_dict_.keys():
            expanded, visible = imgui.collapsing_header(str(key), None)
            if expanded:
                imgui.text("x: " + str(self.qr_dict_[key]["x"]))
                imgui.text("y: " + str(self.qr_dict_[key]["y"]))
                imgui.text("distance: " + str(self.qr_dict_[key]["distance"]))
                if imgui.button("Show Image"):
                    imgui.open_popup("Image: " + str(key))
                    try:
                        self.qr_image = Image.open(
                            QR_DIRECTORY + str(self.qr_dict_[key]["filename"])
                        )
                        self.bind_image(self.qr_image)
                    except:
                        self.get_logger().warn("Issue loading required image.")
                if imgui.begin_popup_modal("Image: " + str(key)).opened:
                    try:
                        # TO DO: Use aspect ratio to dynamically resize the image
                        aspect_ratio = self.qr_image.width / self.qr_image.height
                        width = 1000
                        imgui.image(self.qr_texID, width, width/aspect_ratio)
                    except Exception as e:
                        self.get_logger().warn("No image available. Exception: " + str(e))
                        imgui.close_current_popup()
                    if imgui.button("Close Image"):
                        imgui.close_current_popup()
                    imgui.end_popup()
        imgui.end_child()

        imgui.end()

        imgui.render()
        # END OF IMGUI -----------

        self.impl.render(imgui.get_draw_data())
        glfw.swap_buffers(self.window)


def colour_interpolate(c1, c2, percentage_c2, interpolator):
    hsv_c1 = colorsys.rgb_to_hsv(*c1)
    hsv_c2 = colorsys.rgb_to_hsv(*c2)

    final = (
        interpolator(hsv_c1[0], hsv_c2[0], percentage_c2),
        interpolator(hsv_c1[1], hsv_c2[1], percentage_c2),
        interpolator(hsv_c1[2], hsv_c2[2], percentage_c2),
    )

    return colorsys.hsv_to_rgb(*final)


def linear(v1, v2, percentage_v2):
    return v1 * (1 - percentage_v2) + v2 * percentage_v2


def normalise_rgb(rgb):
    r, g, b = rgb
    return (r / 255, g / 255, b / 255)


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

    try:
        # Run GUI
        while not glfw.window_should_close(gui.window):
            gui.run()
    except:
        # Cleanup After Shutdown
        gui.impl.shutdown()
        glfw.terminate()
        gui.shutdown()
        rclpy.utilities.try_shutdown()
        node_thread.join()
