// Rasterizer.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <stdio.h>
#include <cmath>
#include <SDL.h>
#include <vector>
#include <array>
#include <chrono>
#include <thread>
#include <ctime>
#include <ratio>
#include <chrono>
using namespace std::chrono;

/// <summary>
/// COMMON STRUCTS
/// </summary>
struct matrix4 {
    float matrix[4][4];
};

struct vec4 {
   float vec[4];
};

struct vec3 {
    float x;
    float y;
    float z;

    //Vector ops
    vec3 operator -(const vec3& v)
    {
        vec3 ret;
        ret.x = x - v.x;
        ret.y = y - v.y;
        ret.z = z - v.z;
        return ret;
    }

    vec3 operator +(const vec3& v)
    {
        vec3 ret;
        ret.x = x + v.x;
        ret.y = y + v.y;
        ret.z = z + v.z;
        return ret;
    }

    //Const ops
    vec3 operator *(const float f)
    {
        vec3 ret;
        ret.x = x * f;
        ret.y = y * f;
        ret.z = z * f;
        return ret;
    }

    vec3 operator /(const float f)
    {
        vec3 ret;
        ret.x = x / f;
        ret.y = y / f;
        ret.z = z / f;
        return ret;
    }
};

struct point2D {
    int x;
    int y;
};

struct color {
    Uint8 r;
    Uint8 g;
    Uint8 b;
    Uint8 a;

    bool operator ==(const color& v)
    {
        return r == v.r && g == v.g && b == v.b && a == v.a;
    }

    bool operator !=(const color& v)
    {
        return !(r == v.r && g == v.g && b == v.b && a == v.a);
    }

    color operator *(const float f)
    {
        color ret;
        ret.r = r * f;
        ret.g = g * f;
        ret.b = b * f;
        ret.a = a * f;
        return ret;
    }

    color operator +(const color& c) {
        color ret;
        ret.r = r + c.r;
        ret.g = g + c.g;
        ret.b = b + c.b;
        ret.a = a + c.a;
        return ret;
    }
};

struct sphere {
    vec3 center;
    float radius;
    color color;
    float specular;
    float reflective;
};

enum lightType {
    AMBIENT,
    POINT,
    DIRECTIONAL
};

struct light {
    lightType type;
    float intensity;
    vec3 position;
    vec3 direction;
};

struct triangle {
    //indecies to our verticies.
    int indexes[3];
    color c;
};

struct plane {
    vec3 normal;
    float distance;
};

struct model {
    std::vector<vec3> vertices;
    std::vector<triangle> triangles;
    vec4 boundsCenter;
    float boundsRadius;
};

struct instance {
    model model;
    vec3 position;
    matrix4 rotation;
    vec3 scale;
    matrix4 transform;
};



struct camera {
    vec3 position;
    matrix4 orientation;
    std::vector<plane> clippingPlanes;
};

/// <summary>
/// FACTORY
/// </summary>

inline vec3 Vec4To3(const vec4& v) {
    return vec3{ v.vec[0], v.vec[1] ,v.vec[2] };
}

inline vec4 Vec3To4(const vec3& v) {
    return vec4{ { v.x, v.y, v.z, 1 } };
}

inline matrix4 MakeIdentityMatrix() {
    return matrix4{ {  {1,0,0,0},
                      {0,1,0,0},
                      {0,0,1,0},
                      {0,0,0,1}  } };
}


inline matrix4 MakeTranslationMatrix(vec3 t) {
    return matrix4{{{1,0,0,t.x},
                    {0,1,0,t.y},
                    {0,0,1,t.z},
                    {0,0,0,1 }}};
}

inline matrix4 MakeScalingMatrix(vec3 scale) {
    return matrix4{{{scale.x,0 ,0 ,0},
                    {0 ,scale.y,0 ,0},
                    {0 ,0 ,scale.z,0},
                    {0 ,0 ,0 ,1}} };
}


/// <summary>
/// Common Values
/// </summary>

const float degreesToReadians = 0.0174533;

const color backgroundColor = color{ 128, 128, 128, 255 };

const matrix4 identMatrix = matrix4{ 1,0,0,0,
                                    0,1,0,0,
                                    0,0,1,0,
                                    0,0,0,1 };

const int canvasWidth = 1280;
const int canvasHeight = 720;

float depthBuffer[canvasHeight * canvasWidth];

const int viewportWidth = 16;
const int viewportHeight = 9;

const float zPlane = 16;

bool backfaceCullingEnabled = true;
bool depthBufferingEnabled = true;

SDL_Renderer* renderer;

/// <summary>
/// HELPER FUNCS
/// </summary>

inline matrix4 MatrixMultiply(const matrix4 a, const matrix4 b) {
    matrix4 ret = matrix4{0,0,0,0,
                        0,0,0,0,
                        0,0,0,0,
                        0,0,0,0};
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                ret.matrix[i][j] += a.matrix[i][k] * b.matrix[k][j];
            }
        }
    }
    return ret;
}

inline vec4 VecMatrixMultiply(const matrix4 &m, const vec4 &v) {
    vec4 ret = vec4{0,0,0,0};
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            ret.vec[i] += m.matrix[i][j] * v.vec[j];
        }
    }
    return ret;
}

inline matrix4 Transposed(const matrix4 &mat) {
    matrix4 result = matrix4{};
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            result.matrix[i][j] = mat.matrix[j][i];
        }
    }
    return result;
}


inline matrix4 MakeOYRotationMatrix(float degrees) {
    float c = cos(degrees * M_PI / 180.0);
    float s = sin(degrees * M_PI / 180.0);

    return matrix4{ c, 0, -s, 0,
                    0, 1, 0 , 0,
                    s, 0, c , 0,
                    0, 0, 0 , 1 };
}

inline matrix4 MakeRotationMatrix(float yaw, float pitch, float roll) {
    yaw *= degreesToReadians;
    pitch *= degreesToReadians;
    roll *= degreesToReadians;
    float ca = cos(yaw);
    float sa = sin(yaw);
    float cb = cos(pitch);
    float sb = sin(pitch);
    float cy = cos(roll);
    float sy = sin(roll);
    //matrix4 yawMatrix = matrix4{ ca, -sa, 0, 0,
    //                            sa, ca , 0, 0,
    //                            0 , 0  , 0, 0,
    //                            0 , 0  , 0, 1 };
    //matrix4 pitchMatrix = matrix4{ cb , 0 , sb, 0,
    //                                0  , 1 , 0, 0,
    //                                -sb, 0 , cb, 0,
    //                                0  , 0 , 0, 1 };
    //matrix4 rollMatrix = matrix4{ 1 , 0  , 0  , 0,
    //                              0 , cy , -sy, 0,
    //                              0 , sy , cy , 0,
    //                              0 , 0  , 0  , 1 };

    //return MatrixMultiply(MatrixMultiply(yawMatrix, pitchMatrix), rollMatrix);

    return matrix4{ ca * cb , ca * sb * sy - sa * cy   , ca * sb * cy + sa * sy   , 0,
    				  sa * cb , sa * sb * sy + ca * cy , sa * sb * cy - ca * sy , 0,
    				  -sb     , cb * sy                , cb * cy                , 0,
    				  0       ,0                       ,0                       , 1 };
}

inline vec3 CanvasToViewport(int x, int y) {
    return vec3({ (float)x * viewportWidth / canvasWidth , (float)y * viewportHeight / canvasHeight , zPlane });
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

inline float DotProduct(const vec4& a, const vec4& b) {
    //TODO for now ignore W -- is that correct?
    return a.vec[0] * b.vec[0] + a.vec[1] * b.vec[1] + a.vec[2] * b.vec[2];
}

inline vec3 Cross(vec3 v1,vec3 v2) {
    return vec3(
        v1.y * v2.z - v1.z * v2.y,
        v1.z * v2.x - v1.x * v2.z,
        v1.x * v2.y - v1.y * v2.x);
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


inline point2D ViewportToCanvas(float x, float y) {
    return point2D{ int(x * canvasWidth / viewportWidth),
       int(y * canvasHeight / viewportHeight)};
}

inline point2D ProjectVertex(const vec4 &v) {
    if (v.vec[2] == 0)
        return ViewportToCanvas(v.vec[0] * zPlane / .01, v.vec[1] * zPlane / .01);
    return ViewportToCanvas( v.vec[0] * zPlane / v.vec[2],
        v.vec[1] * zPlane / v.vec[2]);
}

/// <summary>
/// DrawFuncs
/// </summary>

inline void putPixel(int x, int y, const  color &c) {
    //transform x/y to our graph space
    x = canvasWidth / 2.0 + x;
    y = canvasHeight / 2.0 - y;
    //set color and draw a pixel
    SDL_SetRenderDrawColor(renderer, c.r, c.g, c.b, c.a);
    SDL_RenderDrawPoint(renderer, x, y);
}

inline void drawLine(point2D p0, point2D p1, color c) {
    float dx = p1.x - p0.x, dy = p1.y - p0.y;

    if (abs(dx) > abs(dy)) {
        // The line is horizontal-ish. Make sure it's left to right.
        if (dx < 0) { point2D swap = p0; p0 = p1; p1 = swap; }

        // Compute the Y values and draw.
        std::vector<float> ys = Interpolate(p0.x, p0.y, p1.x, p1.y);
        for (int x = p0.x; x <= p1.x; x++) {
            putPixel(x, ys[roundf(x - p0.x)], c);
        }
    }
    else {
        // The line is verical-ish. Make sure it's bottom to top.
        if (dy < 0) { point2D swap = p0; p0 = p1; p1 = swap; }

        // Compute the X values and draw.
        std::vector<float> xs = Interpolate(p0.y, p0.x, p1.y, p1.x);
        for (int y = p0.y; y <= p1.y; y++) {
            putPixel(xs[roundf(y - p0.y)], y, c);
        }
    }
}

inline void DrawWireframeTriangle(const point2D &p0, const  point2D  &p1, const  point2D  &p2, const  color &c) {
    drawLine(p0, p1, c);
    drawLine(p1, p2, c);
    drawLine(p0, p2, c);
}

inline std::vector<int> SortedVertexIndexes(triangle &t, const std::vector<point2D>& projected) {
    std::vector<int> indexes = { 0, 1, 2 };

    if (projected[t.indexes[indexes[1]]].y < projected[t.indexes[indexes[0]]].y) { int swap = indexes[0]; indexes[0] = indexes[1]; indexes[1] = swap; }
    if (projected[t.indexes[indexes[2]]].y < projected[t.indexes[indexes[0]]].y) { int swap = indexes[0]; indexes[0] = indexes[2]; indexes[2] = swap; }
    if (projected[t.indexes[indexes[2]]].y < projected[t.indexes[indexes[1]]].y) { int swap = indexes[1]; indexes[1] = indexes[2]; indexes[2] = swap; }

    return indexes;
}

inline vec3 ComputeTriangleNormal(vec3 v0,vec3  v1,vec3 v2) {
    vec3 v0v1 = v1 + (v0 * -1);
    vec3 v0v2 = v2 + (v0 * -1);
    return Cross(v0v1, v0v2);
}

inline std::tuple<std::vector<float>, std::vector<float>> EdgeInterpolate(float y0, float v0, float y1, float v1, float y2, float v2) {
    std::vector<float> v01 = Interpolate(y0, v0, y1, v1);
    std::vector<float> v12 = Interpolate(y1, v1, y2, v2);
    std::vector<float> v02 = Interpolate(y0, v0, y2, v2);
    v01.pop_back();
    v01.insert(v01.end(), v12.begin(), v12.end());
    return std::tuple<std::vector<float>, std::vector<float>> {v02, v01};
}

inline void ClearDepthBuffer() {
    for (int i = 0; i < sizeof(depthBuffer) / sizeof(depthBuffer[0]); i++) {
        depthBuffer[i] = -std::numeric_limits<float>::max();
    }
}

inline bool UpdateDepthBufferIfCloser(int x, int y, float inv_z) {
    x = canvasWidth / 2 + x;
    y = canvasHeight / 2 - y - 1;

    if (x < 0 || x >= canvasWidth || y < 0 || y >= canvasHeight) {
        return false;
    }

    int offset = x + canvasWidth * y;
    if (depthBuffer[offset] < inv_z) {
        depthBuffer[offset] = inv_z;
        return true;
    }
    return false;
}

inline void RenderTriangle(triangle &t, std::vector <vec3> vertices, std::vector<point2D> &projected) {
    // Sort by projected point Y.
    std::vector<int> indexes = SortedVertexIndexes(t, projected);
    //var[i0, i1, i2] = indexes;

    vec3 v0 = vertices[t.indexes[indexes[0]]];
    vec3 v1 = vertices[t.indexes[indexes[1]]];
    vec3 v2 = vertices[t.indexes[indexes[2]]];

    // Compute triangle normal. Use the unsorted vertices, otherwise the winding of the points may change.
    vec3 normal = ComputeTriangleNormal(vertices[t.indexes[0]], 
                                        vertices[t.indexes[1]], 
                                        vertices[t.indexes[2]]);

    // Backface culling.
    if (backfaceCullingEnabled) {
        vec3 vertex = vertices[t.indexes[0]];
        if (DotProduct(vertex, normal) <= 0) {
            return;
        }
    }

    // Get attribute values (X, 1/Z) at the vertices.
    point2D p0 = projected[t.indexes[indexes[0]]];
    point2D p1 = projected[t.indexes[indexes[1]]];
    point2D p2 = projected[t.indexes[indexes[2]]];

    // Compute attribute values at the edges.
    auto tuple = EdgeInterpolate(p0.y, p0.x, p1.y, p1.x, p2.y, p2.x);
    std::vector <float> x02 = std::get<0>(tuple);
    std::vector <float> x012 = std::get<1>(tuple);
    tuple = EdgeInterpolate(p0.y, 1.0 / v0.z, p1.y, 1.0 / v1.z, p2.y, 1.0 / v2.z);
    std::vector <float> iz02 = std::get<0>(tuple);
    std::vector <float> iz012 = std::get<1>(tuple);
    
    std::vector <float> xLeft, xRight, izLeft, izRight;

    // Determine which is left and which is right.
    int m = (x02.size() / 2);
    if (x02[m] < x012[m]) {
        xLeft = x02;
        xRight = x012;
        izLeft = iz02;
        izRight = iz012;
    }
    else {
        xLeft = x012;
        xRight = x02;
        izLeft = iz012;
        izRight = iz02;
    }

    // Draw horizontal segments.
    for (int y = p0.y; y <= p2.y; y++) {
        int xl = xLeft[y - p0.y];
        int xr = xRight[y - p0.y];

        // Interpolate attributes for this scanline.
        int zl = izLeft[y - p0.y];
        int zr = izRight[y - p0.y];
        std::vector<float> zscan = Interpolate(xl, zl, xr, zr);

        for (int x = xl; x <= xr; x++) {
            if (!depthBufferingEnabled || UpdateDepthBufferIfCloser(x, y, zscan[x - xl])) {
                putPixel(x, y, t.c);
            }
        }
    }

    //TODO do draw outlines
    /*if (drawOutlines) {
        var outline_color = MultiplyColor(0.75, triangle.color);
        DrawLine(p0, p1, outline_color);
        DrawLine(p0, p2, outline_color);
        DrawLine(p2, p1, outline_color);
    }*/
}

inline void RenderModel(model& model) {
    std::vector<point2D> projected;
    for (int i = 0; i < model.vertices.size(); i++) {
        projected.push_back(ProjectVertex(Vec3To4(model.vertices[i])));
    }
    for (int i = 0; i < model.triangles.size(); i++) {
        RenderTriangle(model.triangles[i], model.vertices, projected);
    }
}

void RenderModelWithTransform(model &model, matrix4 &transform) {
    std::vector<point2D> projected;
    for (int i = 0; i < model.vertices.size(); i++) {
        vec3 vertex = model.vertices[i];
        vec4 vertexH = vec4{ vertex.x, vertex.y, vertex.z, 1 };
        projected.push_back(ProjectVertex(VecMatrixMultiply(transform, vertexH)));
    }
    for (int i = 0; i < model.triangles.size(); i++) {
        RenderTriangle(model.triangles[i], model.vertices, projected);
    }
}

inline void ClipTriangle(triangle t, plane p, std::vector<triangle> &triangles_out, std::vector<vec3> vertices) {
    vec3 v0 = vertices[t.indexes[0]];
    vec3 v1 = vertices[t.indexes[1]];
    vec3 v2 = vertices[t.indexes[2]];

    float in0 = DotProduct(p.normal, v0) + p.distance > 0;
    float in1 = DotProduct(p.normal, v1) + p.distance > 0;
    float in2 = DotProduct(p.normal, v2) + p.distance > 0;

    float in_count = in0 + in1 + in2;
    if (in_count == 0) {
        //std::cout << "clip out triangle";
        // Nothing to do - the triangle is fully clipped out.
    }
    else if (in_count == 3) {
        // The triangle is fully in front of the plane.
        triangles_out.push_back(t);
    }
    else if (in_count == 1) {
        //std::cout << "clip triangle part";
        // The triangle has one vertex in. Output is one clipped triangle.
    }
    else if (in_count == 2) {
        //std::cout << "clip triangle part";
        // The triangle has two vertices in. Output is two clipped triangles.
    }
}

bool clipTriangle = true;

inline bool TransformAndClip(const std::vector<plane> &clippingPlanes, model& out, const model &modelToClip, const vec3 &scale, const matrix4 &transform) {
    vec4 center = VecMatrixMultiply(transform, modelToClip.boundsCenter) ;
    float radius = modelToClip.boundsRadius * std::max(std::max(scale.x, scale.y), scale.z);
    for (auto &p : clippingPlanes) {
        float distance = DotProduct(p.normal, Vec4To3(center)) + p.distance;
        if (distance < -radius) {
            return false;
        }
    }

    std::vector<vec3> vertices = {};
    for (int i = 0; i < modelToClip.vertices.size(); i++) {
        vertices.push_back(Vec4To3(VecMatrixMultiply(transform, Vec3To4(modelToClip.vertices[i]))));
    }
    if (clipTriangle) {
        std::vector<triangle> triangles = {};
        for (auto& p : clippingPlanes) {
            for (auto& t : modelToClip.triangles) {
                ClipTriangle(t, p, triangles, vertices);
            }
        }
        out = model{ vertices, triangles, center, modelToClip.boundsRadius };
    }
    else {
        out = model{ vertices, modelToClip.triangles, center, modelToClip.boundsRadius };
    }
    return true;

}

void RenderScene(camera &c, std::vector<instance> &instances) {
    matrix4 cameraMatrix = MatrixMultiply(Transposed(c.orientation), MakeTranslationMatrix(c.position * -1.0));

    for (int i = 0; i < instances.size(); i++) {
        matrix4 transform = MatrixMultiply(cameraMatrix, instances[i].transform);
        model clippedModel = model{};
        bool clipped = TransformAndClip(c.clippingPlanes, clippedModel, instances[i].model, instances[i].scale, transform);
        if (clipped) {
            RenderModel(clippedModel);
            //RenderModelWithTransform(instances[i].model, transform);
        }
    }
}

void clearScreen() {
    SDL_SetRenderDrawColor(renderer, backgroundColor.r, backgroundColor.g, backgroundColor.b, backgroundColor.a);
    SDL_RenderClear(renderer);
    ClearDepthBuffer();
}

/// <summary>
/// MAIN FUNCS
/// </summary>

std::vector<vec3> vertices = {
    vec3{1, 1, 1},
    vec3{-1, 1, 1},
    vec3{-1, -1, 1},
    vec3{1, -1, 1},
    vec3{1, 1, -1},
    vec3{-1, 1, -1},
    vec3{-1, -1, -1},
    vec3{1, -1, -1} };

color RED = color{ 255, 0, 0, 255 };
color GREEN = color{ 0, 255, 0, 255 };
color BLUE = color{ 0, 0, 255, 255 };
color YELLOW = color{ 255, 255, 0, 255 };
color PURPLE = color{ 255, 0, 255, 255 };
color CYAN = color{ 0, 255, 255, 255 };

std::vector<triangle> triangles = {
    triangle{0, 1, 2, RED},
    triangle{0, 2, 3, RED},
    triangle{4, 0, 3, GREEN},
    triangle{4, 3, 7, GREEN},
    triangle{5, 4, 7, BLUE},
    triangle{5, 7, 6, BLUE},
    triangle{1, 5, 6, YELLOW},
    triangle{1, 6, 2, YELLOW},
    triangle{4, 5, 1, PURPLE},
    triangle{4, 1, 0, PURPLE},
    triangle{2, 6, 7, CYAN},
    triangle{2, 7, 3, CYAN}
};

model cube = model{ vertices, triangles, vec4{0,0,0,1}, (float)sqrt(3.0)};

std::vector<instance> instances = { 
    instance{cube, vec3(6, 0, 16), identMatrix, vec3{1, 1, 1}},
    instance{cube, vec3(-6, 0, 16), identMatrix, vec3{1, 1, 1}},
    instance{cube, vec3(6, -2, -16), identMatrix, vec3{1, 1, 1}},
    instance{cube, vec3(-6, 2, -16), identMatrix, vec3{1, 1, 1}},
    //instance{cube, vec3(16, 0, 0), identMatrix, vec3{1, 1, 1}},
    //instance{cube, vec3(18, 0, 0), identMatrix, vec3{1, 1, 1}},
    //instance{cube, vec3(16, 0, 0), identMatrix, vec3{1, 1, 1}},
    //instance{cube, vec3(18, 0, 0), identMatrix, vec3{1, 1, 1}}
    //instance{cube, vec3(-2, 0, 9), MakeRotationMatrix(0,195 * degreesToReadians,0), vec3{1,1,1}}
};

void CalculateInstanceTransforms() {
    for (auto& inst : instances) {
        inst.transform = MatrixMultiply(MakeTranslationMatrix(inst.position), MatrixMultiply(inst.rotation, MakeScalingMatrix(inst.scale)));
    }
}

//float s2 = sqrt(2.0);
float s2 = 0;
//camera cam = camera{ vec3(-3, 1, 2), identMatrix};
//camera cam = camera{ vec3(-3, 1, 2),  MakeOYRotationMatrix(-30) };
camera cam = camera{ 
    vec3(0, 0, -8),
    MakeRotationMatrix(0, 0, 0), 
    {
        {{0,0,1}, 1},
        {{s2,0,s2},0},
        {{-s2,0,s2},0},
        {{0,-s2,s2},0},
        {{0,s2,s2},0}      
} 
};

float maxViewAngle = 45;

inline vec4 GetCameraForward(camera &c) {
    return VecMatrixMultiply(c.orientation, vec4{ 0,0,1, 0 });
}

inline void UpdateClipPlanes(camera& c) {
    vec4 forward = GetCameraForward(c);

    c.clippingPlanes[0].normal = Vec4To3(forward);
    c.clippingPlanes[0].distance = -1;    

    /*c.clippingPlanes[1].normal = Cross(Vec4To3(VecMatrixMultiply(MakeRotationMatrix(0, 0, -maxViewAngle), forward)), c.position);
    c.clippingPlanes[2].normal = Cross(Vec4To3(VecMatrixMultiply(MakeRotationMatrix(0, 0, maxViewAngle), forward)), c.position);
    c.clippingPlanes[3].normal = Cross(Vec4To3(VecMatrixMultiply(MakeRotationMatrix(0, 0, -maxViewAngle), forward)), c.position);
    c.clippingPlanes[4].normal = Cross(Vec4To3(VecMatrixMultiply(MakeRotationMatrix(0, 0, maxViewAngle), forward)), c.position);*/

    c.clippingPlanes[1].normal = Vec4To3(VecMatrixMultiply(MakeRotationMatrix(0, -maxViewAngle + 90, 0), forward));    
    c.clippingPlanes[2].normal = Vec4To3(VecMatrixMultiply(MakeRotationMatrix(0, maxViewAngle - 90, 0), forward));    
    c.clippingPlanes[3].normal = Vec4To3(VecMatrixMultiply(MakeRotationMatrix(0, 0, -maxViewAngle + 90), forward));    
    c.clippingPlanes[4].normal =  Vec4To3(VecMatrixMultiply(MakeRotationMatrix(0, 0, maxViewAngle - 90), forward));
    /*c.clippingPlanes[1].normal = Vec4To3(VecMatrixMultiply(c.orientation, vec4{ s2,0,s2,1 }));
    c.clippingPlanes[2].normal = Vec4To3(VecMatrixMultiply(c.orientation, vec4{ -s2,0,s2,1 }));
    c.clippingPlanes[3].normal = Vec4To3(VecMatrixMultiply(c.orientation, vec4{ 0,-s2,s2,1 }));
    c.clippingPlanes[4].normal = Vec4To3(VecMatrixMultiply(c.orientation, vec4{ 0,s2,s2,1 }));*/
}

inline void MoveCamera(camera& c, vec3 move) {
    cam.position = cam.position + move;
    UpdateClipPlanes(c);
}

inline void MoveCameraForward(camera& c, float speed) {    
    MoveCamera(c, Vec4To3(GetCameraForward(c)) * speed);
}

inline void RotateCamera(camera& c, float yaw, float pitch, float roll) {
    cam.orientation = MatrixMultiply(cam.orientation, MakeRotationMatrix(yaw, pitch, roll));
    UpdateClipPlanes(c);
}

int main(int argc, char* argv[]) {
    SDL_Event event;
    SDL_Window* window;
    int i = 0;

    SDL_Init(SDL_INIT_VIDEO);
    SDL_CreateWindowAndRenderer(canvasWidth, canvasHeight, 0, &window, &renderer);
    clearScreen();
    //drawAxis();

    CalculateInstanceTransforms();
    UpdateClipPlanes(cam);

    //Scene s = Scene::MakeTestScene();
    RenderScene(cam, instances);
    //drawSceneThreaded(s);

    //drawLine(point2D{ 100,100 }, point2D{ 200,200 }, RED);
    //DrawWireframeTriangle(point2D{ 100,0 }, point2D{ 100,100 }, point2D{ 200,100 }, RED);
    SDL_RenderPresent(renderer);

    //for (int i = 0; i < 3600; i++)    
    while(true)
    {
        high_resolution_clock::time_point t1 = high_resolution_clock::now();
        i++;
        if (i > 3600) {
            i = 0;
        }
        clearScreen();
        //cam.orientation = MakeRotationMatrix(0.0,0.0, i / 10.0);
        //cam.position.z -= 1;
        //cam.orientation = MakeRotationMatrix(i, 0, 0);
        instances[0].rotation = MakeRotationMatrix(0, i, 0);
        instances[1].rotation = MakeRotationMatrix(i*1.5 , i*.5, i);
        CalculateInstanceTransforms();
        RenderScene(cam, instances);
        while (SDL_PollEvent(&event)) {

            switch (event.type) {
                /* Keyboard event */
                /* Pass the event data onto PrintKeyInfo() */
            case SDL_KEYDOWN:

            case SDL_KEYUP:
                switch (event.key.keysym.sym) {
                case SDLK_w:
                    MoveCameraForward(cam, .2);
                    break;
                case SDLK_s:
                    cam.position = cam.position - Vec4To3(GetCameraForward(cam));
                    break;
                case SDLK_a:
                    RotateCamera(cam, 0, -1, 0);
                    //cam.orientation = MatrixMultiply(cam.orientation, MakeRotationMatrix(0, -1,0));
                    break;
                case SDLK_d:
                    RotateCamera(cam, 0, 1, 0);
                    //cam.orientation = MatrixMultiply(cam.orientation, MakeRotationMatrix(0, 1, 0));
                    break;

                case SDLK_q:
                    RotateCamera(cam, 0, 0, -1);
                    //cam.orientation = MatrixMultiply(cam.orientation, MakeRotationMatrix(0, 0, -1));
                    break;

                case SDLK_e:
                    RotateCamera(cam, 0, 0, 1);
                    //cam.orientation = MatrixMultiply(cam.orientation, MakeRotationMatrix(0, 0, 1));
                    break;

                default:
                    break;
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        SDL_RenderPresent(renderer);
        high_resolution_clock::time_point t2 = high_resolution_clock::now();
        duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
        std::cout << time_span.count() << " seconds" << std::endl;
    }

    while (1) {
        //drawSceneThreaded(s);
        if (SDL_PollEvent(&event) && event.type == SDL_QUIT)
            break;
    }
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return EXIT_SUCCESS;
}