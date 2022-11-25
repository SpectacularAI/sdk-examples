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
    vec4 shift = vec4(a_Effect.xy, 0, 0);
    gl_Position = u_Projection * v_Position + shift;
    gl_PointSize = 9.0 / v_Position.z;
}
