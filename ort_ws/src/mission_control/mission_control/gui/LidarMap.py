# MapSize is 100x100meters
import numpy as np
import OpenGL.GL as gl

from ctypes import c_void_p
import random

from project_gorgon.gui.data_structures import shaders, EBO, VBO, VAO


vertexSource = """
#version 330 core

layout (location = 0) in vec2 aPos;
layout (location = 1) in float aWeightOne;
layout (location = 2) in float aWeightTwo;

out float weightOne;
out float weightTwo;
out vec2 pos;

void main()
{
  gl_Position = vec4(aPos.x, aPos.y, 0.0f, 1.0f);
  weightOne = aWeightOne;
  weightTwo = aWeightTwo;
  pos = aPos;
}
"""

fragmentSource ="""
#version 330 core

out vec4 FragColour;

in float weightOne;
in float weightTwo;
in vec2 pos;


// APPLY GAMMA CORRECTION
// so that colour values are human readable

// Change gamma to Uniform so it can be adjusted by the user of the program.
//uniform float gamma
const float gamma = 2.2;

void main()
{
  FragColour = vec4(weightOne, 0, weightTwo, 1.0);
  FragColour.rgb = pow(FragColour.rgb, vec3(1.0/gamma));
}"""

class AIMap:
    def __init__(self):
        # defined from centre to corner
        self.hexagonSize = 2
        self.hexagonWidth = np.sqrt(3) * self.hexagonSize
        self.hexagonHeight = 2 * self.hexagonSize

        self.mapWidth = 100
        self.mapHeight = 100

        self.texWidth = 800
        self.texHeight = 800

        self.generateVAO()
        # self.generateFrameBuffer()

        self.shaderProgram = shaders.rawShaderProgram(vertexSource, fragmentSource)
        # self.drawToTexture()

    def generatePositionArray(self):
        # pointy top hexagon
        numHorizontal = self.mapWidth / (np.sqrt(3) * self.hexagonSize)
        numVertical = self.mapHeight / (3 / 2 * self.hexagonSize)

        horizontalSpacing = 2 / numHorizontal
        verticalSpacing = 2 / numVertical

        coordinates = []
        x = -1.0 - horizontalSpacing / 2
        for _ in range(int(numHorizontal)):
            x += horizontalSpacing
            y = -1.0 - verticalSpacing / 2
            for _ in range(int(numVertical)):
                y += verticalSpacing
                coordinates.append(x)
                coordinates.append(y)

        return np.array(coordinates, dtype=np.float32)

    def generateVertexArray(self):
        # pointy top hexagon
        numHorizontal = self.mapWidth / (np.sqrt(3) * self.hexagonSize)
        numVertical = self.mapHeight / (3 / 2 * self.hexagonSize)

        horizontalSpacing = 2 / numHorizontal
        verticalSpacing = 2 / numVertical

        width = horizontalSpacing
        height = 4 / 3 * verticalSpacing

        vertexCoordinates = []
        indiceOrder = []
        index = 0
        x = -1.0 - horizontalSpacing / 2
        for _ in range(int(numHorizontal)):
            x += horizontalSpacing
            y = -1.0 - verticalSpacing / 2
            for ind in range(int(numVertical)):
                if ind % 2:
                    x_coord = x + 1 / 2 * horizontalSpacing
                else:
                    x_coord = x

                y += verticalSpacing
                # centre point is (x_coord, y)

                # COORD1
                v1_x = x_coord - 1 / 2 * width
                v1_y = y - 1 / 4 * height
                vertexCoordinates.extend([v1_x, v1_y])

                # COORD2
                v2_x = x_coord - 1 / 2 * width
                v2_y = y + 1 / 4 * height
                vertexCoordinates.extend([v2_x, v2_y])

                # COORD3
                v3_x = x_coord
                v3_y = y + 1 / 2 * height
                vertexCoordinates.extend([v3_x, v3_y])

                # COORD4
                v4_x = x_coord + 1 / 2 * width
                v4_y = y + 1 / 4 * height
                vertexCoordinates.extend([v4_x, v4_y])

                # COORD5
                v5_x = x_coord + 1 / 2 * width
                v5_y = y - 1 / 4 * height
                vertexCoordinates.extend([v5_x, v5_y])

                # COORD6
                v6_x = x_coord
                v6_y = y - 1 / 2 * height
                vertexCoordinates.extend([v6_x, v6_y])

                # Draw Order
                # Triangle 1
                indiceOrder.extend([index, index + 1, index + 2])

                # Triangle 2
                indiceOrder.extend([index, index + 2, index + 5])

                # Triangle 3
                indiceOrder.extend([index + 2, index + 3, index + 5])

                # Triangle 4
                indiceOrder.extend([index + 3, index + 4, index + 5])

                index += 6
        return [
            np.array(vertexCoordinates, dtype=np.float32),
            np.array(indiceOrder, dtype=np.uint32),
        ]

    def generateVAO(self):
        vertexArray = self.generateVertexArray()

        positions = vertexArray[0]
        indices = vertexArray[1]

        self.mapVAO = VAO.VAO()
        self.mapVAO.Bind()

        positionVBO = VBO.staticArrayVBO(positions)
        self.indexEBO = EBO.EBO(indices)

        self.mapVAO.LinkAttrib(
            positionVBO, 0, 2, gl.GL_FLOAT, 2 * positions.dtype.itemsize, c_void_p()
        )

        # dictionary of Weights
        self.weightVBOs = {}
        # TEST WEIGHTS
        weights = []
        testWeight = 0.0
        inc = 1 / (len(indices) / 12)
        for _ in range(int(len(indices) / 12)):
            weights.extend(6 * [testWeight])
            testWeight += inc

        weights = np.array(weights, dtype=np.float32)
        self.weightVBOs[1] = VBO.dynamicArrayVBO(weights)
        self.mapVAO.LinkAttrib(
            self.weightVBOs[1], 1, 1, gl.GL_FLOAT, weights.dtype.itemsize, c_void_p()
        )
        # ---

        # TEST WEIGHT TWO
        weights = []
        for _ in range(int(len(indices) / 12)):
            num = random.uniform(0, 1)
            weights.extend(6 * [num])
        weights = np.array(weights, dtype=np.float32)
        self.weightVBOs[2] = VBO.dynamicArrayVBO(weights)
        self.mapVAO.LinkAttrib(
            self.weightVBOs[2], 2, 1, gl.GL_FLOAT, weights.dtype.itemsize, c_void_p()
        )

        self.indices = indices
        # self.indexEBO.Unbind()
        self.mapVAO.Unbind()

    def addWeight(self, weights):
        key = len(self.weightVBOs)
        self.weightVBOs[key] = VBO.dynamicArrayVBO(weights)

        self.mapVAO.Bind()
        self.mapVAO.LinkAttrib(
            self.weightVBOs[key], key, 1, gl.GL_FLOAT, weights.dtype.itemsize, c_void_p()
        )
        self.mapVAO.Unbind()

    def changeWeights(self, weightID, newWeights):
        weightsBIND = []
        for weight in newWeights:
            weightsBIND.extend(6 * [weight])
        
        weightsBIND = np.array(weightsBIND, dtype=np.float32)
        
        self.weightVBOs[weightID]



    # (FIX FOR LATE IF NEEDED)
    # THIS FUNCTION CURRENTLY THROWS ERRORS
    def _generateFrameBuffer(self):
        self.fboID = gl.glGenFramebuffers(1)
        self.textureID = gl.glGenTextures(1)

        gl.glBindFramebuffer(gl.GL_FRAMEBUFFER, self.fboID)

        # Using TEXTURE10 as it is unlikely to be in use
        # gl.glActiveTexture(gl.GL_TEXTURE10)
        gl.glBindTexture(gl.GL_TEXTURE_2D, self.textureID)

        antiAliasingSamples = 8
        gl.glTexImage2DMultisample(
            gl.GL_TEXTURE_2D_MULTISAMPLE,
            antiAliasingSamples,
            gl.GL_RGB16F,
            self.texWidth,
            self.texHeight,
            gl.GL_TRUE,
        )

        # gl.glTexParameteri(gl.GL_TEXTURE_2D_MULTISAMPLE, gl.GL_TEXTURE_MIN_FILTER, gl.GL_NEAREST)
        # gl.glTexParameteri(gl.GL_TEXTURE_2D_MULTISAMPLE, gl.GL_TEXTURE_MAG_FILTER, gl.GL_NEAREST)

        gl.glBindTexture(gl.GL_TEXTURE_2D, 0)

        gl.glFramebufferTexture2D(
            gl.GL_FRAMEBUFFER,
            gl.GL_COLOR_ATTACHMENT0,
            gl.GL_TEXTURE_2D_MULTISAMPLE,
            self.textureID,
            0,
        )

        self.rboID = gl.glGenRenderbuffers(1)
        gl.glBindRenderbuffer(gl.GL_RENDERBUFFER, self.rboID)
        gl.glRenderbufferStorageMultisample(
            gl.GL_RENDERBUFFER,
            antiAliasingSamples,
            gl.GL_DEPTH24_STENCIL8,
            self.texWidth,
            self.texHeight,
        )
        gl.glBindRenderbuffer(gl.GL_RENDERBUFFER, 0)
        gl.glFramebufferRenderbuffer(
            gl.GL_FRAMEBUFFER,
            gl.GL_DEPTH_STENCIL_ATTACHMENT,
            gl.GL_RENDERBUFFER,
            self.rboID,
        )

        gl.glBindFramebuffer(gl.GL_FRAMEBUFFER, 0)

    # _ for unused functions yet to be completed
    def _drawToTexture(self):
        gl.glViewport(0, 0, self.texWidth, self.texHeight)
        gl.glBindFramebuffer(gl.GL_FRAMEBUFFER, self.fboID)
        gl.glClear(gl.GL_COLOR_BUFFER_BIT)
        gl.glClear(gl.GL_DEPTH_BUFFER_BIT)

        gl.glDisable(gl.GL_DEPTH_TEST)
        self.mapVAO.Bind()
        self.shaderProgram.Activate()

        gl.glDrawElements(gl.GL_TRIANGLES, len(self.indices), gl.GL_UNSIGNED_INT, None)
        gl.glBindFramebuffer(gl.GL_FRAMEBUFFER, 0)
        self.mapVAO.Unbind()

    def draw(self):
        gl.glClear(gl.GL_COLOR_BUFFER_BIT)
        gl.glClear(gl.GL_DEPTH_BUFFER_BIT)

        self.mapVAO.Bind()
        self.shaderProgram.Activate()

        gl.glDrawElements(gl.GL_TRIANGLES, len(self.indices), gl.GL_UNSIGNED_INT, None)
