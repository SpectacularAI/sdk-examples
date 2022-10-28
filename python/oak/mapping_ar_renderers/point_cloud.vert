#version 150

uniform mat4 u_ModelView;
uniform mat4 u_Projection;
attribute vec3 a_Position;
attribute vec3 a_Effect;
varying vec4 v_Position;
varying vec3 v_Effect;

void main() {
    v_Position = u_ModelView * vec4(a_Position.xyz, 1.0);
    v_Effect = a_Effect;
    gl_Position = u_Projection * v_Position;
    gl_PointSize = 12.0 / v_Position.z;
}
