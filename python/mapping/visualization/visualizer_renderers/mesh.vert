#version 330 core

layout(location = 0) in vec3 in_Position;
layout(location = 1) in vec3 in_Color;

uniform mat4 u_ModelView;
uniform mat4 u_Projection;
uniform float u_Opacity;

out vec3 frag_Position;
out vec3 frag_Color;

void main()
{
    vec4 v_Position = u_ModelView * vec4(in_Position.xyz, 1.0);
    gl_Position = u_Projection * v_Position;

    frag_Position = gl_Position.xyz;
    frag_Color = in_Color.xyz;
}
