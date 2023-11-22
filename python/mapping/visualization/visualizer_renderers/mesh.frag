#version 330 core

uniform float u_Opacity;

in vec3 frag_Position;
in vec3 frag_Color;

out vec4 out_Color;

void main()
{
    out_Color = vec4(frag_Color.xyz, u_Opacity);
}
