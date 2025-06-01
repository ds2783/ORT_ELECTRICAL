import OpenGL.GL as gl

class shaderProgram():
    def __init__(self, rawVertex, rawFrag):
        f = open(rawVertex, "r")
        vertexCode = f.read()

        f = open(rawFrag, "r")
        fragCode = f.read()
        
        vertexShader = gl.glCreateShader(gl.GL_VERTEX_SHADER)
        gl.glShaderSource(vertexShader, vertexCode)
        gl.glCompileShader(vertexShader)
        
        fragmentShader = gl.glCreateShader(gl.GL_FRAGMENT_SHADER)
        gl.glShaderSource(fragmentShader, fragCode)
        gl.glCompileShader(fragmentShader)

        # print(gl.glGetShaderiv(fragmentShader, gl.GL_COMPILE_STATUS))
       
        self.ID = gl.glCreateProgram()
        gl.glAttachShader(self.ID, vertexShader)
        gl.glAttachShader(self.ID, fragmentShader)
        
        gl.glLinkProgram(self.ID)
       
        # check correctly compiled
        string = gl.glGetProgramInfoLog(self.ID)
        # print(string)

        gl.glDeleteShader(vertexShader)
        gl.glDeleteShader(fragmentShader)

    def Activate(self):
        gl.glUseProgram(self.ID)

    def Delete(self):
        gl.glDeleteShader(self.ID)


class rawShaderProgram():
    def __init__(self, vertexCode, fragCode):
        
        vertexShader = gl.glCreateShader(gl.GL_VERTEX_SHADER)
        gl.glShaderSource(vertexShader, vertexCode)
        gl.glCompileShader(vertexShader)
        
        fragmentShader = gl.glCreateShader(gl.GL_FRAGMENT_SHADER)
        gl.glShaderSource(fragmentShader, fragCode)
        gl.glCompileShader(fragmentShader)

        # print(gl.glGetShaderiv(fragmentShader, gl.GL_COMPILE_STATUS))
       
        self.ID = gl.glCreateProgram()
        gl.glAttachShader(self.ID, vertexShader)
        gl.glAttachShader(self.ID, fragmentShader)
        
        gl.glLinkProgram(self.ID)
       
        # check correctly compiled
        string = gl.glGetProgramInfoLog(self.ID)
        # print(string)

        gl.glDeleteShader(vertexShader)
        gl.glDeleteShader(fragmentShader)

    def Activate(self):
        gl.glUseProgram(self.ID)

    def Delete(self):
        gl.glDeleteShader(self.ID)
