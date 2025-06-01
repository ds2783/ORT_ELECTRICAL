# MapSize is 100x100meters
from uuid import main
import numpy as np
import OpenGL.GL as gl

from ctypes import c_void_p

from mission_control.gui.data_structures import shaders, EBO, VBO, VAO
from mission_control.streaming.stream_client import StreamClient


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

fragmentSource ="""
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
    def __init__(self, main_cam):
        # defined from centre to corner
        self.texWidth = 800
        self.texHeight = 800
    
        # self.main_cam = main_cam
        self.main_cam_texID = gl.glGenTextures(1)

        self.main_cam = main_cam
        self.shaderProgram = shaders.rawShaderProgram(vertexSource, fragmentSource)

        self.generateVAO()
        self.generateEBO()

    def generateVAO(self):
        coords = [
	            -0.5, -0.5, 0.0, 1.0, # Lower left corner
	            -0.5,  0.5, 0.0, 0.0, # Upper left corner
	             0.5,  0.5, 1.0, 0.0, # Upper right corner
	             0.5, -0.5, 1.0, 1.0  # Lower right corner
                ]
        formatted_coords = np.array(coords, dtype=np.float32)
        coordsVBO = VBO.staticArrayVBO(formatted_coords)
        self.VAO = VAO.VAO()
        self.VAO.LinkAttrib(coordsVBO, 0, 2, gl.GL_FLOAT, 4 * formatted_coords.dtype.itemsize, c_void_p())
        self.VAO.LinkAttrib(coordsVBO, 1, 2, gl.GL_FLOAT, 4 * formatted_coords.dtype.itemsize, 
                            c_void_p(2 * formatted_coords.dtype.itemsize))
        coordsVBO.Unbind()
        self.VAO.Unbind()

    def generateEBO(self):
        indices = [
                	0, 2, 1, # Upper triangle
	                0, 3, 2 # Lower triangle
                ]
        formatted_indices = np.array(indices, dtype=np.uint32)
        self.EBO = EBO.EBO(formatted_indices)
        self.indices = formatted_indices

    def draw_main(self, image):
        gl.glBindTexture(gl.GL_TEXTURE_2D, self.main_cam_texID)
        
        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MIN_FILTER, gl.GL_NEAREST);
        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MAG_FILTER, gl.GL_LINEAR);

        # Set texture clamping method
        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_S, gl.GL_REPEAT);
        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_T, gl.GL_REPEAT);
   

        gl.glTexImage2D(gl.GL_TEXTURE_2D,
                        0,
                        gl.GL_RGB,
                        image.shape[1],
                        image.shape[0],
                        0,     
                        gl.GL_BGR,
                        gl.GL_UNSIGNED_BYTE,  
                        image)
        
        self.VAO.Bind()
        self.EBO.Bind()

        texUni = gl.glGetUniformLocation(self.shaderProgram.ID, "tex0")
        self.shaderProgram.Activate()
        
        gl.glUniform1i(texUni, 0)
        
        gl.glDrawElements(gl.GL_TRIANGLES, len(self.indices), gl.GL_UNSIGNED_INT, None)

        
    def draw(self):
        gl.glClear(gl.GL_COLOR_BUFFER_BIT)
        
        image = self.main_cam.fetch_frame()
        if image is not None:
            self.draw_main(image)

