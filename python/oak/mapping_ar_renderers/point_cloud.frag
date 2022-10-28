#version 150

precision mediump float;
varying vec4 v_Position;
varying vec3 v_Effect;

void main() {
    float r = 1 - length(2 * (gl_PointCoord - 0.5));
    float alpha = (1 - v_Effect.z) * r;
    gl_FragColor = vec4(0, 1, 0, alpha);
}
