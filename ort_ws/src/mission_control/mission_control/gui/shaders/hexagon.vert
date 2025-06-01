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
