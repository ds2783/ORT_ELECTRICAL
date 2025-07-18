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
    flagQoS,
    baseQoS,
    RetCodes,
)
from mission_control.config.gui import (
    OFS_DEFAULT_MOVE_TIME,
    QR_DIRECTORY,
    WIDTH,
    HEIGHT,
    CALIBRATE_OFS,
)

from threading import Thread
from multiprocessing.connection import Listener
from PIL import Image
import numpy as np
import time

# Messages ---
from std_msgs.msg import Bool, Float32, Int8
from ort_interfaces.srv import CullCalibration
from ort_interfaces.action import Calibrate


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
        # Action Servers
        self.calibration_client_ = ActionClient(self, Calibrate, "/imu/calibrate")
        self.optical_calibration_client_ = ActionClient(
            self, Calibrate, "/optical_flow/calibrate"
        )

        self.current_step = 0
        self.last_result = ""

        # Publishers
        self.reset_pos_pub_ = self.create_publisher(Bool, "/elysium/reset_pos", baseQoS)
        self.led_pub_ = self.create_publisher(Float32, "/led", tofQoS)
        self.rock_num_pub_ = self.create_publisher(Int8, "/rock_num", flagQoS)

        # Services
        self.reset_optical_calibration_client_ = self.create_client(
            CullCalibration, "/elysium/srv/cull_calibration"
        )

    def cull_calibration(self, index=0, clear_all=True):
        timeout = time.monotonic()
        now = time.monotonic()
        while (
            not self.reset_optical_calibration_client_.wait_for_service(1.5)
            and (now - timeout) < 1.5
        ):
            now = time.monotonic()
            self.get_logger().warn(
                "Waiting for connection to optical calibration service."
            )

        if (now - timeout) < 1.0:
            request = CullCalibration.Request()
            request.clear_all = clear_all
            request.index = index
            future = self.reset_optical_calibration_client_.call_async(request)
            future.add_done_callback(self.complete_cullCB_)
        else:
            self.get_logger().warn("Optical cull calibration service is unavailable.")

    def update_rock_num(self, rock_num):
        msg = Int8()
        if 1 <= rock_num <= 5:
            msg.data = int(rock_num)
            self.rock_num_pub_.publish(msg)
        else:
            self.get_logger().warn("Invalid rock number selected.")

    def complete_cullCB_(self, future):
        response = future.result()
        if response.ret_code == RetCodes.SUCCESS:
            self.get_logger().info(
                "Calibration of optical flow sensor successfully culled."
            )
            self.last_result = "Success: culled OFS Calibration"
        elif response.ret_code == RetCodes.FAIL:
            self.get_logger().error("Calibration of optical flow sensro failed.")
            self.last_result = "Failure: could not Cull OFS Calibration."

    def publish_led(self, slider_float: float):
        msg = Float32()
        msg.data = slider_float
        self.led_pub_.publish(msg)

    def reset_axis(self):
        msg = Bool()
        msg.data = True
        self.reset_pos_pub_.publish(msg)

    def send_goal(self, code, move_time=None):
        goal_msg = Calibrate.Goal()
        goal_msg.code = code
        if move_time:
            goal_msg.move_time = move_time

        if code == CALIBRATE_OFS:
            self._send_goal_future = self.optical_calibration_client_.send_goal_async(
                goal_msg, feedback_callback=self.feedback_callback
            )
        else:
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
        match result.result:
            case RetCodes.SUCCESS:
                result = "Success"
            case RetCodes.FAIL:
                result = "Fail"
            case RetCodes.FAIL_UNRECOGNISED_OP_CODE:
                result = "Fail, unrecognised OP-code."
            case RetCodes.FAIL_DETECTED_NO_OFS_FORWARD:
                result = "Fail, no change in distance measured by the OFS."
            case RetCodes.FAIL_DETECTED_NO_TOF_FORWARD:
                result = "Fail, no change in distance measure by the TOF."
            case RetCodes.FAIL_TOF_DETECTED_NO_REASONABLE_RANGE:
                result = "Fail, TOF detected an unreasonable change in distance."
        self.get_logger().info("Result: {0}".format(result))
        self.last_result = result

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.current_step = feedback.seconds
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

        self.dashboard = Dashboard(cams, self.get_logger)

        # Network
        self.last_msg = None
        self.bulk = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "x_vel": 0.0,
            "y_vel": 0.0,
            "z_vel": 0.0,
            "yaw": 0.0,
            "pitch": 0.0,
            "roll": 0.0,
            "cam_y": 0.0,
            "cam_p": 0.0,
            "o-tof": 0.0,
            "gps-d": 0.0,
            "soc": 0.0,
        }

        # QR
        self.last_qr_ = "None"
        self.qr_dict_ = {}
        self.qr_texID = gl.glGenTextures(1)

        # Calbration Client
        self.client_ = GuiClient()
        self.logger = self.client_.get_logger

        # GPS
        self.gps_dist = 0.0

        # tof
        self.q_tof = 0.0

        # IR LED
        self.led = 0.0
        self.current_value = 100.0

        self.address = ("localhost", port)  # family is deduced to be 'AF_INET'
        self.listener = Listener(self.address, authkey=b"123")
        self.listen = True
        self.conn = self.listener.accept()
        self.comms_thread = Thread(target=self.qrComms)
        self.comms_thread.start()

        # OFS
        self.move_time = OFS_DEFAULT_MOVE_TIME

        # IMGUI
        imgui.get_io().font_global_scale = 1.2
        self.style = imgui.get_style()
        self.style.window_rounding = 3.0

        self.rock_check_boxes_ = {
            "rock-1": False,
            "rock-2": False,
            "rock-3": False,
            "rock-4": False,
            "rock-5": False,
        }

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
                if self.last_msg == "dump" and type(recieved_msg) is dict:
                    self.bulk = recieved_msg
                elif self.last_msg == "qrdic" and type(recieved_msg) is dict:
                    self.qr_dict_ = recieved_msg
                    self.get_logger().info("Recieved qr codes from Base Station.")
                elif type(recieved_msg) is str:
                    data = str(recieved_msg)[6:]
                    match recieved_msg[:5]:
                        case "qr---":
                            self.last_qr_ = data
                        case "q-tof":
                            self.q_tof = float(data)
                self.last_msg = recieved_msg

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

    def reset_rock_checkboxes_(self, rock):
        for key in self.rock_check_boxes_.keys():
            if rock != key:
                self.rock_check_boxes_[key] = False

        self.client_.update_rock_num(int(rock[-1]))

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
        # offset = self.width / 40
        offset = 0
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

        imgui_text = [
            "Telemetry:",
            "(All data is in degrees)\n\n",
            "Yaw: {:.3f}\nPitch: {:.3f}\nRoll: {:.3f}\n".format(
                self.bulk["yaw"], self.bulk["pitch"], self.bulk["roll"]
            ),
            "x: {:.3f}\ny: {:.3f}\nz: {:.3f}\n".format(
                self.bulk["x"], self.bulk["y"], self.bulk["z"]
            ),
            "x_vel: {:.3f}\ny_vel: {:.3f}\nz_vel: {:.3f}\n".format(
                self.bulk["x_vel"], self.bulk["y_vel"], self.bulk["z_vel"]
            ),
            "camera-yaw: {:.3f}\ncamera-pitch: {:.3f}\n".format(
                self.bulk["cam_y"], self.bulk["cam_p"]
            ),
            "bottom-dist: {:.3f}\ncamera-dist: {:.3f}\n".format(
                self.bulk["o-tof"], self.q_tof
            ),
        ]

        for text in imgui_text:
            imgui.text(text)

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
            low_bat_colour, high_bat_colour, self.bulk["soc"], linear
        )
        display_batt_colour = normalise_rgb(batt_colour_rgb)

        imgui.push_style_color(
            imgui.COLOR_PLOT_HISTOGRAM,
            *display_batt_colour,
        )
        imgui.progress_bar(
            self.bulk["soc"], (self.width / 14, 18 + 1 / 200 * self.height), ""
        )
        imgui.pop_style_color(1)
        imgui.same_line()
        imgui.text(f"{self.bulk["soc"] * 100:.1f}%")

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
            imgui.text(
                "Seconds remaining until completion: " + str(self.client_.current_step)
            )
            imgui.text("Last result: (" + str(self.client_.last_result) + ")")
            imgui.spacing()

            # if imgui.button("Calibrate IMU - KEEP IMU STILL"):
            #     self.client_.send_goal(CALIBRATE_IMU)

            if imgui.button("Zero Axis"):
                self.client_.reset_axis()

            elif imgui.button(
                "Calibrate OFS"
            ):
                self.move_time = 0.6
                imgui.open_popup("Set move time.")

            elif imgui.button(
                "Hard Reset OFS Calibration - To be done for new surfaces."
            ):
                self.client_.cull_calibration(clear_all=True)

            elif imgui.button("Close Client"):
                imgui.close_current_popup()

            if imgui.begin_popup_modal("Set move time.").opened:
                _, self.move_time = imgui.slider_float(
                    "move time",
                    self.move_time,
                    min_value=0.0,
                    max_value=10.0,
                    format="%.1f",
                )
                imgui.same_line()
                if imgui.button("Submit."):
                    self.client_.send_goal(CALIBRATE_OFS, self.move_time)
                    imgui.close_current_popup()
                imgui.same_line()
                if imgui.button("Cancel."):
                    imgui.close_current_popup()
                imgui.text(
                    "0.6 seconds move time, \nis approximately 0.2 meters of movement on a smooth surface."
                )
                imgui.end_popup()

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

        imgui.begin_child(
            "Rock Select", self.width / 2.15 - offset, self.height / 16, True
        )
        event_box_1, self.rock_check_boxes_["rock-1"] = imgui.checkbox(
            "Rock 1", self.rock_check_boxes_["rock-1"]
        )
        if event_box_1:
            self.reset_rock_checkboxes_("rock-1")

        imgui.same_line()
        event_box_2, self.rock_check_boxes_["rock-2"] = imgui.checkbox(
            "Rock 2", self.rock_check_boxes_["rock-2"]
        )
        if event_box_2:
            self.reset_rock_checkboxes_("rock-2")

        imgui.same_line()
        event_box_3, self.rock_check_boxes_["rock-3"] = imgui.checkbox(
            "Rock 3", self.rock_check_boxes_["rock-3"]
        )
        if event_box_3:
            self.reset_rock_checkboxes_("rock-3")

        imgui.same_line()
        event_box_4, self.rock_check_boxes_["rock-4"] = imgui.checkbox(
            "Rock 4", self.rock_check_boxes_["rock-4"]
        )
        if event_box_4:
            self.reset_rock_checkboxes_("rock-4")

        imgui.same_line()
        event_box_5, self.rock_check_boxes_["rock-5"] = imgui.checkbox(
            "Rock 5", self.rock_check_boxes_["rock-5"]
        )
        if event_box_5:
            self.reset_rock_checkboxes_("rock-5")
        imgui.end_child()

        # QR LIST
        imgui.begin_child("QR-List", self.width / 2 - offset, self.height / 3, True)

        for category in ["Rock 1", "Rock 2", "Rock 3", "Rock 4", "Rock 5"]:
            expanded, visible = imgui.collapsing_header(category, None)
            if expanded:
                for key in self.qr_dict_.keys():
                    if self.qr_dict_[key]["rock-num"] == int(category[-1]):
                        self.display_entry(key)

        imgui.end_child()

        imgui.end()

        imgui.render()
        # END OF IMGUI -----------

        self.impl.render(imgui.get_draw_data())
        glfw.swap_buffers(self.window)

    def display_entry(self, key):
        imgui.indent()
        expanded, visible = imgui.collapsing_header(str(key), None)
        if expanded:
            imgui.text("x: " + str(self.qr_dict_[key]["x"]))
            imgui.text("y: " + str(self.qr_dict_[key]["y"]))
            imgui.text(
                "distance-ofs-imu: " + str(self.qr_dict_[key]["distance-ofs-imu"])
            )
            imgui.text("distance-gps: " + str(self.qr_dict_[key]["distance-gps"]))
            imgui.text("more-reliable: " + str(self.qr_dict_[key]["more-reliable"]))
            imgui.text(
                "distance-from-cam: " + str(self.qr_dict_[key]["distance-from-cam"])
            )
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
                    width = 1000
                    aspect_ratio = self.qr_image.width / self.qr_image.height
                    imgui.set_window_size(width + 10, width / aspect_ratio + 90)
                    width, height = imgui.get_window_size()
                    imgui.image(self.qr_texID, width, width / aspect_ratio)
                except Exception as e:
                    self.get_logger().warn("No image available. Exception: " + str(e))
                    imgui.close_current_popup()
                if imgui.button("Close Image"):
                    imgui.close_current_popup()
                imgui.end_popup()

        imgui.unindent()


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
