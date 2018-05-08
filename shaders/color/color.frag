#version 330 core

uniform vec3 givenColor;

out vec4 color;

void main()
{
    color = vec4(givenColor, 1.0f);
}