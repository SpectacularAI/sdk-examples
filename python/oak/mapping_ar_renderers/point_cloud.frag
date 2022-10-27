#version 150

precision mediump float;
varying vec4 v_Position;

void main() {
    float alpha = 1 - length(2 * (gl_PointCoord - 0.5));
    gl_FragColor = vec4(0, 1, 0, alpha);
}
