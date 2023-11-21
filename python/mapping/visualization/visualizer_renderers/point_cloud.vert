#version 330 core

layout(location = 0) in vec3 in_Position;
layout(location = 1) in vec3 in_Color;
layout(location = 2) in vec3 in_Normal;

uniform mat4 u_Model;
uniform mat4 u_View;
uniform mat4 u_Projection;
uniform vec3 u_CameraPositionWorld;
uniform float u_PointSize;
uniform float u_Opacity;
// 0 = original colors
// 1 = x
// 2 = y
// 3 = z
// 4 = distance from camera (depth)
// 5 = world normals
uniform int u_ColorMode;
uniform int u_HasColor;  // 0 == false, otherwise true
uniform int u_HasNormal;  // 0 == false, otherwise true

out vec4 frag_Color;

const float MAX_Z = float({MAX_Z});
const float COLOR_MAP_SCALE = float({COLOR_MAP_SCALE});

vec3 colorMapJET(float value) {
    float r = clamp(1.5 - abs(3.0 - 4.0 * value), 0.0, 1.0);
    float g = clamp(1.5 - abs(2.0 - 4.0 * value), 0.0, 1.0);
    float b = clamp(1.5 - abs(1.0 - 4.0 * value), 0.0, 1.0);
    return vec3(r, g, b);
}

mat3 extractMat3(mat4 matrix) {
    return mat3(matrix[0].xyz, matrix[1].xyz, matrix[2].xyz);
}

void main() {
    vec4 v_PositionWorld = u_Model * vec4(in_Position.xyz, 1.0);
    vec4 v_PositionCamera = u_View * v_PositionWorld;
    gl_Position = u_Projection * v_PositionCamera;
    gl_PointSize = u_PointSize;

    vec3 color = vec3(0.0, 0.17, 0.21); // default color
    float opacity = u_Opacity;

    if (MAX_Z != float(0.0) && v_PositionWorld.z > MAX_Z) {
        // discard
        gl_Position.w = 0;
    } else {
        switch (u_ColorMode) {
            case 1:
                color = colorMapJET(0.5 - v_PositionWorld.x * COLOR_MAP_SCALE);
                break;

            case 2:
                color = colorMapJET(0.5 - v_PositionWorld.y * COLOR_MAP_SCALE);
                break;

            case 3:
                color = colorMapJET(0.5 + v_PositionWorld.z * COLOR_MAP_SCALE * 2.0);
                break;

            case 4:
                float depth = length(v_PositionWorld.xyz - u_CameraPositionWorld.xyz) * COLOR_MAP_SCALE * 5.0;
                color = colorMapJET(exp(-depth*0.5));
                break;

            case 5:
                if (u_HasNormal > 0) {
                    vec3 v_NormalWorld = extractMat3(u_Model) * in_Normal.xyz;
                    color = (v_NormalWorld.xyz + vec3(1.0, 1.0, 1.0)) * 0.5;
                }
                break;

            default:
                if (u_HasColor > 0) color = in_Color.xyz;
                break;
        }
    }

    frag_Color = vec4(color.xyz, opacity);
}
