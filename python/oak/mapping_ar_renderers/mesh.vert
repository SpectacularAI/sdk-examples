#version 150
precision mediump float;

uniform mat4 u_ModelViewProjection;
attribute vec3 a_Position;
attribute vec3 a_Normal;
attribute vec3 a_TexCoord;
varying vec3 v_Normal;
varying vec3 v_TexCoord;
varying vec3 v_Position;
varying vec4 v_ScreenNdcHomog;

void main() {
    v_Normal = a_Normal;
    v_TexCoord = a_TexCoord;
    v_Position = a_Position.xyz;
    v_ScreenNdcHomog = u_ModelViewProjection * vec4(a_Position.xyz, 1.0);
    gl_Position = v_ScreenNdcHomog;
}
