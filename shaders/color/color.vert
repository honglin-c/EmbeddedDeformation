#version 330 core
layout (location = 0) in vec3 position;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

uniform mat4 scaling;

void main()
{
    gl_Position = projection * view * model *  scaling * vec4(position, 1.0f);
}
