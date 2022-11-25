#version 150

precision mediump float;
varying vec4 v_Position;
varying vec3 v_Effect;

void main() {
    float d = length(2 * (gl_PointCoord - 0.5));
    float alpha = 1 - d * d;
    float c = 0;

    const float TIME_APPEAR = 0.1;
    const float TIME_DISAPPEAR = 0.3;
    if (v_Effect.z < TIME_APPEAR) {
        alpha *= v_Effect.z / TIME_APPEAR;
    }
    else if (v_Effect.z > TIME_DISAPPEAR) {
        c = (v_Effect.z - TIME_DISAPPEAR) / (1 - TIME_DISAPPEAR);
        alpha *= (1 - c * c);
    }
    gl_FragColor = vec4(c, 1, c, alpha);
}
