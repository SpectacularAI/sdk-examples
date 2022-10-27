#version 150
precision mediump float;

uniform vec4 u_Options;
varying vec3 v_Normal;
varying vec3 v_TexCoord;
varying vec3 v_Position;
varying vec4 v_ScreenNdcHomog;

void main() {
    vec3 objColor = vec3(0.5) + v_Normal*0.5;
    float distanceAlpha = exp(-abs(v_ScreenNdcHomog.w) / 400.0);
    float meshAlpha = 0.6 * distanceAlpha;
    float alpha = v_TexCoord.z * meshAlpha;

    vec2 border;
    if (u_Options.y > 0.0) {
        // "Fake mesh".
        const float GRID_SZ = 4.0;
        vec3 relPos = u_Options.y * v_Position / GRID_SZ;

        relPos = relPos - floor(relPos);
        if (u_Options.z > 0.0) {
            // Z levels only.
            border = vec2(relPos.z);
        }
        else {
            vec3 ano = abs(v_Normal);
            if (ano.z > max(ano.x, ano.y)) {
                border = relPos.xy;
            } else if (ano.x > max(ano.y, ano.z)) {
                border = relPos.yz;
            } else {
                border = relPos.xz;
            }
        }
    }
    else {
        // border = abs(v_TexCoord.xy - 0.5)*2.0; // funny pattern
        border = v_TexCoord.xy; // true mesh
    }

    float maxBord = 0.015;
    float gridAlpha = min(1.0, 2.0 * distanceAlpha);
    float relBord = 1.0 - min(border.x, border.y) / maxBord;
    if (u_Options.x > 0.0 && relBord > 0.0) {
        vec3 gridCol = vec3(0.0, 1.0, 1.0);
        float gridAlpha = gridAlpha * sqrt(v_TexCoord.z);
        objColor = objColor * (1.0 -  gridAlpha) + gridAlpha * gridCol;
        alpha = gridAlpha + (1.0 - gridAlpha) * alpha;
    }

    gl_FragColor = vec4(objColor, u_Options.w * alpha);
}
