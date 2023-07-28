#pragma once
#include "CommonValues.h"
#include <cmath>

inline matrix4 MatrixMultiply(matrix4 a, matrix4 b) {
    matrix4 ret = matrix4{};
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                ret.matrix[i + j * 4] += a.matrix[i + k * 4] * b.matrix[k + j * 4];
            }
        }
    }
    return ret;
}

inline vec4 VecMatrixMultiply(matrix4 m, vec4 v) {
    vec4 ret;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            ret.vec[i] += m.matrix[i + j * 4] * v.vec[j];
        }
    }
    return ret;
}

inline matrix4 Transposed(matrix4 mat) {
    matrix4 result = matrix4{};
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            result.matrix[i + j * 4] = mat.matrix[j + i * 4];
        }
    }
    return result;
}

inline vec3 CanvasToViewport(int x, int y) {
    return vec3({ x * viewportWidth / canvasHeight , y * viewportHeight / canvasWidth , zPlane });
}


inline float Length(vec3 v) {
    return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

inline vec3 Normalize(vec3 v) {
    float length = Length(v);
    return v / length;
}
inline float DotProduct(const vec3& a, const vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline std::vector<float> Interpolate(float i0, float d0, float i1, float d1) {
    std::vector<float> ret;
    if (i0 == i1) {
        ret.push_back(d0);
        return ret;
    }

    float a = (d1 - d0) / (i1 - i0);
    float d = d0;
    for (int i = i0; i <= i1; i++) {
        ret.push_back(d);
        d += a;
    }

    return ret;
}


inline point2D ViewportToCanvas(point2D p2d) {
    return point2D{ int(p2d.x * canvasWidth / viewportWidth) | 0,
       int(p2d.y * canvasHeight / viewportHeight) | 0 };
}

inline point2D ProjectVertex(vec4 v) {
    return ViewportToCanvas(point2D(v.vec[0] * zPlane / v.vec[2],
        v.vec[1] * zPlane / v.vec[2]));
}