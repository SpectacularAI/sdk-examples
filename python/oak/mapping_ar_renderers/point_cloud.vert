#version 150

uniform mat4 u_ModelViewProjection;
attribute vec3 a_Position;
varying vec4 v_Position;

void main() {
    v_Position = u_ModelViewProjection * vec4(a_Position.xyz, 1.0);
    gl_Position = v_Position;
    gl_PointSize = 1.0 + 7.0 / v_Position.z;
}
