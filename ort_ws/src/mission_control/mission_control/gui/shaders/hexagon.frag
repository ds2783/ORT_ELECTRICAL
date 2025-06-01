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
}
