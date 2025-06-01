import OpenGL.GL as gl
import numpy as np


class staticArrayVBO:
    def __init__(self, data):
        self.ID = gl.glGenBuffers(1)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.ID)
        gl.glBufferData(gl.GL_ARRAY_BUFFER, data.nbytes, data, gl.GL_STATIC_DRAW)

    def Bind(self):
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.ID)

    def Unbind(self):
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)

    def Delete(self):
        gl.glDeleteBuffers(1, self.ID)


class dynamicArrayVBO:
    def __init__(self, data: np.ndarray):
        self.ID = gl.glGenBuffers(1)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.ID)
        gl.glBufferData(gl.GL_ARRAY_BUFFER, data.nbytes, data, gl.GL_DYNAMIC_DRAW)

    def update(self, data: np.ndarray):
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.ID)
        gl.glBufferData(gl.GL_ARRAY_BUFFER, data.nbytes, data, gl.GL_DYNAMIC_DRAW)

    def Bind(self):
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.ID)

    def Unbind(self):
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)

    def Delete(self):
        gl.glDeleteBuffers(1, self.ID)
