import imgui.core as imgui
import glfw
import OpenGL.GL as gl
from imgui.integrations.glfw import GlfwRenderer

from mission_control.gui.dashboard import Dashboard


def impl_glfw_init(window_name="Project Gorgon", width=1200, height=1200):
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


class GUI(object):
    def __init__(self, base_node):
        super().__init__()
        self.backgroundColor = (0, 0, 0, 1)
        self.window = impl_glfw_init()
        gl.glClearColor(*self.backgroundColor)
        imgui.create_context()
        self.impl = GlfwRenderer(self.window)
        self.base_node = base_node

        # REPLACE THESE PATHS WITH ABSOLUTE PATH OF SHADERS ON INSTALL
        # alternative place all shader code with string in a python file
        self.dashboard = Dashboard(self.base_node.main_cam)

    def run(self):
        glfw.poll_events()
        self.impl.process_inputs()
        gl.glClearColor(*self.backgroundColor)
        gl.glClear(gl.GL_COLOR_BUFFER_BIT)
        
        imgui.get_io().font_global_scale = 1.2

        self.dashboard.draw()
        imgui.new_frame()
        # Set Gorgon Control-Mode Panel
        imgui.begin("Set Gorgon Control-Mode", True)
        imgui.text("Select From Different Operation Modes")

        if imgui.button("INERT"):
            self.base_node.gui_set_mode("INERT")
        elif imgui.button("MANUAL"):
            self.base_node.gui_set_mode("MANUAL")
        elif imgui.button("AUTOMATIC_MODE_ONE"):
            self.base_node.gui_set_mode("AUTOMATIC_MODE_ONE")
        elif imgui.button("AUTOMATIC_MODE_TWO"):
            self.base_node.gui_set_mode("AUTOMATIC_MODE_TWO")
        elif imgui.button("BUTTON"):
            self.base_node.gui_set_mode("BUTTON")

        imgui.end()

        imgui.begin("QR-Display", True)
        imgui.set_window_size(180, 80)
        imgui.text(self.base_node.last_qr)
        imgui.end()

        # Display Testing Window
        imgui.show_test_window()

        imgui.render()

        self.impl.render(imgui.get_draw_data())
        glfw.swap_buffers(self.window)
