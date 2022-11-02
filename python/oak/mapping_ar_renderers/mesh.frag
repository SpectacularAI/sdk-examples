#version 150

precision mediump float;
varying vec3 v_Normal;
varying vec3 v_TexCoord;
varying vec3 v_Position;
varying vec4 v_ScreenNdcHomog;

void main() {
    vec3 objColor = vec3(0.5) + v_Normal*0.5;
    float distanceAlpha = exp(-abs(v_ScreenNdcHomog.w) / 400.0);
    float meshAlpha = 0.6 * distanceAlpha;
    float alpha = v_TexCoord.z * meshAlpha;
    vec2 border = v_TexCoord.xy;

    float maxBord = 0.015;
    float gridAlpha = min(1.0, 2.0 * distanceAlpha);
    float relBord = 1.0 - min(border.x, border.y) / maxBord;
    if (relBord > 0.0) {
        vec3 gridCol = vec3(0.0, 1.0, 1.0);
        float gridAlpha = gridAlpha * sqrt(v_TexCoord.z);
        objColor = objColor * (1.0 -  gridAlpha) + gridAlpha * gridCol;
        alpha = gridAlpha + (1.0 - gridAlpha) * alpha;
    }
    gl_FragColor = vec4(objColor, alpha);
}
