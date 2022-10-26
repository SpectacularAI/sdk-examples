#version 150

precision mediump float;
varying vec4 v_Position;

void main() {
    float a = 1.0 / max(1.5 * v_Position.z, 1.0);
    gl_FragColor = vec4(1 - a, a, 0.2, 1.0);
}
