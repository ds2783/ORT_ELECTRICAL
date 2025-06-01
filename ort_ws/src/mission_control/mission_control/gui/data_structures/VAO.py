import OpenGL.GL as gl


class VAO:
    def __init__(self) -> None:
        self.ID = gl.glGenVertexArrays(1)

    def LinkAttrib(self, VBO, layout, numCompenents, dataType, stride, offset):
        self.Bind()
        VBO.Bind()
        gl.glVertexAttribPointer(
            layout, numCompenents, dataType, gl.GL_FALSE, stride, offset
        )
        gl.glEnableVertexAttribArray(layout)
        VBO.Unbind()

    def Bind(self):
        gl.glBindVertexArray(self.ID)

    def Unbind(self):
        gl.glBindVertexArray(0)

    def Delete(self):
        gl.glDeleteVertexArrays(1, self.ID)
