#version 150

uniform mat4 u_ModelViewProjection;
attribute vec3 a_Position;

void main() {
    gl_PointSize = 3.0;
    gl_Position = u_ModelViewProjection * vec4(a_Position.xyz, 1.0);
}
