# MapSize is 100x100meters
from uuid import main
import numpy as np
import OpenGL.GL as gl

from ctypes import c_void_p

from mission_control.gui.data_structures import shaders, EBO, VBO, VAO


vertexSource = """
#version 330 core

// Positions/Coordinates
layout (location = 0) in vec2 aPos;
// Texture Coordinates
layout (location = 1) in vec2 aTex;

// Outputs the texture coordinates to the fragment shader
out vec2 texCoord;

// Controls the scale of the vertices
// uniform float scale;
const float scale = 1.0f;

void main()
{
	// Outputs the positions/coordinates of all vertices
	gl_Position = vec4(aPos.x + aPos.x * scale, aPos.y + aPos.y * scale, 0, 1.0);
	// Assigns the texture coordinates from the Vertex Data to "texCoord"
	texCoord = aTex;
}
"""

fragmentSource = """
#version 330 core

// Outputs colors in RGBA
out vec4 FragColor;


// Inputs the texture coordinates from the Vertex Shader
in vec2 texCoord;

// Gets the Texture Unit from the main function
uniform sampler2D tex0;


void main()
{
	FragColor = texture(tex0, texCoord);
}
"""


class Dashboard:
    def __init__(self, cams):
        self.cam_texID = gl.glGenTextures(2)

        self.cams = cams
        self.shaderProgram = shaders.rawShaderProgram(vertexSource, fragmentSource)

        main_coords = [
            -0.5, 0, 0, 0.0, 0.0,  # lower left corner
            -0.5, 0.5, 1.0, 0.0,  # upper left corner
             0.1, 0.5, 1.0, 1.0,  # upper right corner
             0.1, 0.0, 0.0, 1.0,  # lower right corner
        ]

        secondary_coords = [
            -0.5, -0.5, 0.0, 0.0,  # lower left corner
            -0.5, 0.0, 1.0, 0.0,  # upper left corner
             0.1, 0.0, 1.0, 1.0,  # upper right corner
             0.1,-0.5, 0.0, 1.0,  # lower right corner
        ]
        self.VAOs = [self.generateVAO(main_coords), self.generateVAO(secondary_coords)]
        if len(cams) != self.VAOs:
            # fix to use the ROS2 logger
            print("ERROR, incorrect dimesions of Cams passed.")
        self.generateEBO()

    def generateVAO(self, coords):
        formatted_coords = np.array(coords, dtype=np.float32)
        vbo = VBO.staticArrayVBO(formatted_coords)
        vao = VAO.VAO()
        vao.LinkAttrib(
            vbo, 0, 2, gl.GL_FLOAT, 4 * formatted_coords.dtype.itemsize, c_void_p()
        )
        vao.LinkAttrib(
            vbo,
            1,
            2,
            gl.GL_FLOAT,
            4 * formatted_coords.dtype.itemsize,
            c_void_p(2 * formatted_coords.dtype.itemsize),
        )
        vbo.Unbind()
        vao.Unbind()

        return vao

    def generateEBO(self):
        indices = [0, 1, 3, 1, 2, 3]  # Upper triangle  # Lower triangle
        formatted_indices = np.array(indices, dtype=np.uint32)
        self.EBO = EBO.EBO(formatted_indices)
        self.indices = formatted_indices

    def draw_main(self):
        for index, cam in enumerate(self.cams):
            image = cam.fetch_frame()
            if image is not None:
                gl.glBindTexture(gl.GL_TEXTURE_2D, self.cam_texID[index])

                gl.glTexParameteri(
                    gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MIN_FILTER, gl.GL_NEAREST
                )
                gl.glTexParameteri(
                    gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MAG_FILTER, gl.GL_LINEAR
                )

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
                    gl.GL_BGR,
                    gl.GL_UNSIGNED_BYTE,
                    image,
                )

                self.VAOs[index].Bind()
                self.EBO.Bind()

                texUni = gl.glGetUniformLocation(self.shaderProgram.ID, "tex0")
                self.shaderProgram.Activate()

                gl.glUniform1i(texUni, 0)

                gl.glDrawElements(
                    gl.GL_TRIANGLES, len(self.indices), gl.GL_UNSIGNED_INT, None
                )

    def draw(self):
        gl.glClear(gl.GL_COLOR_BUFFER_BIT)
        self.draw_main()
