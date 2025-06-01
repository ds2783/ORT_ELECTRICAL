import OpenGL.GL as gl
from mission_control.gui.data_structures import VAO, VBO, shaders
import ctypes
import numpy as np


class Triangle:
    def __init__(self):
        self.load_shaders()
        self.vao = VAO.VAO()
        self.vertex_data = np.array([-1, -1, 1, -1, 0, 1], dtype=np.float32)
        self.vbo = VBO.staticArrayVBO(self.vertex_data)
        self.vao.Bind()
        self.vao.LinkAttrib(
            self.vbo,
            0,
            2,
            gl.GL_FLOAT,
            2 * self.vertex_data.dtype.itemsize,
            ctypes.c_void_p(),
        )

    def load_shaders(self):
        shaderCode = {
            gl.GL_VERTEX_SHADER: """\
                    #version 330 core
                    layout(location = 0) in vec2 vertexPosition_modelspace;
                    void main(){
                      gl_Position.xy = vertexPosition_modelspace;
                      gl_Position.z = 0.0;
                      gl_Position.w = 1.0;
                    }
                    """,
            gl.GL_FRAGMENT_SHADER: """\
                    #version 330 core
                    out vec3 color;
                    void main(){
                      color = vec3(1,0,0);
                    }
                    """,
        }
        self.shader = shaders.rawShaderProgram(
            shaderCode[gl.GL_VERTEX_SHADER], shaderCode[gl.GL_FRAGMENT_SHADER]
        )

    def render(self):
        self.vao.Bind()
        self.shader.Activate()
        gl.glDrawArrays(gl.GL_TRIANGLES, 0, 3)  # Starting from vertex 0
