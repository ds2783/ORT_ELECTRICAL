import rclpy
from rclpy.node import Node

import imgui.core as imgui
import glfw
import OpenGL.GL as gl
from imgui.integrations.glfw import GlfwRenderer

from mission_control.gui.dashboard import Dashboard
from mission_control.streaming.stream_client import StreamClient
from mission_control.config.ports import PORT1, PORT_MAIN, PORT_SECONDARY

from threading import Thread
from multiprocessing.connection import Listener

# DON'T BOTH WITH EXECUTORS
# ONLY USING ROS NODE WRAPPER FOR LAUNCH FILE

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

        self.address = ('localhost', port)     # family is deduced to be 'AF_INET'
        self.listener = Listener(self.address, authkey=b'123')
        self.last_qr_ = "None"
        
        self.conn = self.listener.accept()
        comms_thread = Thread(target=self.recieveComms)
        comms_thread.start()

    def recieveComms(self):
        while True:
            recieved_qr = self.conn.recv()
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

        # Display Testing Window
        # imgui.show_test_window()

        imgui.render()

        self.impl.render(imgui.get_draw_data())
        glfw.swap_buffers(self.window)

def main(args=None):
    rclpy.init(args=args)
    
    cams = [
        StreamClient("Stereo", "192.168.0.101", "udp", PORT_MAIN, 640, 480, stereo=False),
        StreamClient("Stereo", "192.168.0.101", "udp", PORT_SECONDARY, 640, 480, stereo=False),
    ]
    gui = GUI(cams, PORT1)

    # DON'T BOTH WITH EXECUTORS
    # ONLY USING ROS NODE WRAPPER FOR LAUNCH FILE

    # Run GUI
    while not glfw.window_should_close(gui.window):
        gui.run()

    # Cleanup After Shutdown
    gui.impl.shutdown()
    glfw.terminate()
    rclpy.shutdown()
