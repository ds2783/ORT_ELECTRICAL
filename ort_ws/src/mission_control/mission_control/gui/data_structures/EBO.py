import OpenGL.GL as gl


class EBO:
    def __init__(self, indices):
        self.ID = gl.glGenBuffers(1)
        gl.glBindBuffer(gl.GL_ELEMENT_ARRAY_BUFFER, self.ID)
        gl.glBufferData(
            gl.GL_ELEMENT_ARRAY_BUFFER, indices.nbytes, indices, gl.GL_STATIC_DRAW
        )

    def Bind(self):
        gl.glBindBuffer(gl.GL_ELEMENT_ARRAY_BUFFER, self.ID)

    def Unbind(self):
        gl.glBindBuffer(gl.GL_ELEMENT_ARRAY_BUFFER, 0)

    def Delete(self):
        gl.glDeleteBuffers(1, self.ID)
