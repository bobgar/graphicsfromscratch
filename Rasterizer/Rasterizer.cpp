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
    int v0;
    int v1;
    int v2;
    color c;
};

struct model {
    std::vector<vec3> vertices;
    std::vector<triangle> triangles;
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
};

/// <summary>
/// FACTORY
/// </summary>

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

const float canvasWidth = 1280;
const float canvasHeight = 720;

const float viewportWidth = 16;
const float viewportHeight = 9;

const float zPlane = 1.0;

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
    return vec3({ x * viewportWidth / canvasWidth , y * viewportHeight / canvasHeight , zPlane });
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

void DrawWireframeTriangle(const point2D &p0, const  point2D  &p1, const  point2D  &p2, const  color &c) {
    drawLine(p0, p1, c);
    drawLine(p1, p2, c);
    drawLine(p0, p2, c);
}


void RenderTriangle(triangle &t, std::vector<point2D> &projected) {
    DrawWireframeTriangle(projected[t.v0],
        projected[t.v1],
        projected[t.v2],
        t.c);
}

void RenderModel(model &model, matrix4 &transform) {
    std::vector<point2D> projected;
    for (int i = 0; i < model.vertices.size(); i++) {
        vec3 vertex = model.vertices[i];
        vec4 vertexH = vec4{ vertex.x, vertex.y, vertex.z, 1 };
        projected.push_back(ProjectVertex(VecMatrixMultiply(transform, vertexH)));
    }
    for (int i = 0; i < model.triangles.size(); i++) {
        RenderTriangle(model.triangles[i], projected);
    }
}

void RenderScene(camera &c, std::vector<instance> &instances) {
    matrix4 cameraMatrix = MatrixMultiply(Transposed(c.orientation), MakeTranslationMatrix(c.position * -1.0));

    for (int i = 0; i < instances.size(); i++) {
        matrix4 transform = MatrixMultiply(cameraMatrix, instances[i].transform);
        RenderModel(instances[i].model, transform);
    }
}

void clearScreen() {
    SDL_SetRenderDrawColor(renderer, backgroundColor.r, backgroundColor.g, backgroundColor.b, backgroundColor.a);
    SDL_RenderClear(renderer);
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

model cube = model{ vertices, triangles };

std::vector<instance> instances = { 
    instance{cube, vec3(-1.5, 0, 2), identMatrix, vec3{2, 3, 1}},
    instance{cube, vec3(1.25, 2.5, 2), identMatrix, vec3{.5, .25, 1}}
    //instance{cube, vec3(-2, 0, 9), MakeRotationMatrix(0,195 * degreesToReadians,0), vec3{1,1,1}}
};

void CalculateInstanceTransforms() {
    for (auto& inst : instances) {
        inst.transform = MatrixMultiply(MakeTranslationMatrix(inst.position), MatrixMultiply(inst.rotation, MakeScalingMatrix(inst.scale)));
    }
}


//camera cam = camera{ vec3(-3, 1, 2), identMatrix};
//camera cam = camera{ vec3(-3, 1, 2),  MakeOYRotationMatrix(-30) };
camera cam = camera{ vec3(-3, 1, -2),  MakeRotationMatrix( 0, -30, 0) };

int main(int argc, char* argv[]) {
    SDL_Event event;
    SDL_Window* window;
    int i;

    SDL_Init(SDL_INIT_VIDEO);
    SDL_CreateWindowAndRenderer(canvasWidth, canvasHeight, 0, &window, &renderer);
    clearScreen();
    //drawAxis();

    CalculateInstanceTransforms();

    //Scene s = Scene::MakeTestScene();
    RenderScene(cam, instances);
    //drawSceneThreaded(s);

    //drawLine(point2D{ 100,100 }, point2D{ 200,200 }, RED);
    //DrawWireframeTriangle(point2D{ 100,0 }, point2D{ 100,100 }, point2D{ 200,100 }, RED);
    SDL_RenderPresent(renderer);

    matrix4 rotTest1 = MakeRotationMatrix(0,30,0);
    matrix4 rotTest2 = MakeOYRotationMatrix(30);

    for (int i = 0; i < 3600; i++)
    {
        clearScreen();
        //cam.orientation = MakeRotationMatrix(0.0,0.0, i / 10.0);
        cam.position.z -= 0.01;
        instances[0].rotation = MakeRotationMatrix(0, i, 0);
        instances[1].rotation = MakeRotationMatrix(i*3 , i, i*2);
        CalculateInstanceTransforms();
        RenderScene(cam, instances);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        SDL_RenderPresent(renderer);
        
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