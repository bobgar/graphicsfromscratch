// RayTracer.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <chrono>
#include <limits>
#include <iostream>
#include <stdlib.h>
#include <vector>
#include <string>

#include <thread> 
#include <future>
#include <mutex>

#include <SDL.h>
#include "CommonStructs.h"
#include "Scene.h"


#undef main

const int canvasHeight = 800;
const int canvasWidth = 800;

const float viewportWidth = 1;
const float viewportHeight = 1;

const float zPlane = 1.0;

const color backgroundColor = color{ 128, 128, 128, 255 };

SDL_Renderer* renderer;

SDL_Texture* buffer;
uint32_t* pixels;

float Length(vec3 v) {
    return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

vec3 Normalize(vec3 v) {
    float length = Length(v);
    return v / length;
}

//std::mutex drawMutex;
//void putPixel(int x, int y, color c  ) {
//    drawMutex.lock();
//    //transform x/y to our graph space
//    x = canvasHeight / 2 + x;
//    y = canvasWidth / 2 - y;
//    //set color and draw a pixel
//    SDL_SetRenderDrawColor(renderer, c.r, c.g, c.b, c.a);
//    SDL_RenderDrawPoint(renderer, x, y);
//    drawMutex.unlock();
//}

void putPixel(int x, int y, color c) {
    x = canvasHeight / 2 + x;
    y = canvasWidth / 2 - y;
    /*Uint32 dst = (Uint32*)((Uint8*)pixels + y * canvasWidth);
    *dst++ = (0xFF000000 | (c.r << 16) | (c.g << 8) | c.b);*/
    int index = x + y * canvasWidth;
    int pixelValue = c.r * 0xff000000 + c.g * 0x00ff0000 + c.b * 0x0000ff00 + 0x000000ff;
    pixels[index] = pixelValue;
}

void clearScreen() {
    SDL_SetRenderDrawColor(renderer, backgroundColor.r, backgroundColor.g, backgroundColor.b, backgroundColor.a);
    SDL_RenderClear(renderer);
}

void drawAxis() {
    for (int x = -canvasHeight / 2; x < canvasHeight / 2; ++x)
        putPixel(x, 0, color{ 255, 100, 100, 255 });
    for (int y = -canvasWidth / 2; y < canvasWidth / 2; ++y)
        putPixel(0, y, color{ 100, 255, 100, 255 });
}

vec3 CanvasToViewport(int x,int y) {
    return vec3({ x * viewportWidth / canvasHeight , y * viewportHeight / canvasWidth , zPlane });
}

inline float DotProduct(const vec3& a, const vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

vec3 ReflectRay(vec3 ray, vec3 normal) {
    return normal * 2 * DotProduct(normal, ray) - ray;
}

std::vector<float> IntersectRaySphere( vec3 &origin, const vec3 &direction, const sphere &s) {
    vec3 oc = origin - s.center;

    float k1 = DotProduct(direction, direction);
    float k2 = 2 * DotProduct(oc, direction);
    float k3 = DotProduct(oc, oc) - s.radius * s.radius;

    float discriminant = k2 * k2 - 4 * k1 * k3;
    if (discriminant < 0) {
        return std::vector<float>{std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity()};
    }

    float t1 = (-k2 + sqrt(discriminant)) / (2 * k1);
    float t2 = (-k2 - sqrt(discriminant)) / (2 * k1);
    return std::vector<float>{ t1, t2};
}

std::tuple<sphere, float> ClosestIntersection(Scene &s, vec3 &origin, vec3 &direction, float min_t, float max_t) {

    float closest_t = std::numeric_limits<float>::infinity();
    sphere closest_sphere = sphere{ vec3{ 0,0,0 }, 0, backgroundColor, 0 };

    for (int i = 0; i < s.spheres.size(); i++) {
        std::vector<float> ts = IntersectRaySphere(origin, direction, s.spheres[i]);
        if (ts[0] < closest_t && min_t < ts[0] && ts[0] < max_t) {
            closest_t = ts[0];
            closest_sphere = s.spheres[i];
        }
        if (ts[1] < closest_t && min_t < ts[1] && ts[1] < max_t) {
            closest_t = ts[1];
            closest_sphere = s.spheres[i];
        }
    }
    return { closest_sphere, closest_t };
}

float ComputeLighting(Scene &s, vec3 &point, vec3 &normal, vec3 direction, float specular, float max_t) {
    float i = 0;
    float dot;
    vec3 l;
    for (auto& light : s.lights) {
        if (light.type == AMBIENT) {
            i += light.intensity;
        }
        else
        {
            if (light.type == POINT) {
                l = light.position - point;                
            }
            else if (light.type == DIRECTIONAL) {
                l = light.direction;
            }
            //Shadow
            auto [closest_sphere, closest_t] = ClosestIntersection(s, point, l, 0.001, max_t);
            if (closest_sphere.radius != 0)
                continue;
            //Diffuse
            dot = DotProduct(normal, l);
            if (dot > 0)
                i += light.intensity * dot / (Length(normal) * Length(l));//TODO length of normal will always be 1 -- should I remove this?
            //Specular
            if (specular != -1) {
                vec3 R = normal * 2 * DotProduct(normal, l) - l;    
                float r_dot_direction = DotProduct(R, direction);
                if (r_dot_direction > 0) {
                    i += light.intensity * powf(r_dot_direction / (Length(R) * Length(direction)), specular);
                }
            }
        }
    }
    if (i > 1) i = 1;
    return i;
}

const int MAX_RECURSION_DEPTH = 2;

// Traces a ray against the set of spheres in the scene.
color TraceRay(Scene &s, vec3 &origin, vec3 &direction, float min_t, float max_t, int recursionDepth) {
 
    auto[closest_sphere, closest_t] = ClosestIntersection(s,  origin,  direction,  min_t,  max_t);

    vec3 intersect_point = origin + direction * closest_t;
    vec3 intersect_normal = intersect_point - closest_sphere.center;
    intersect_normal = Normalize(intersect_normal);
    float i = ComputeLighting(s, intersect_point, intersect_normal, direction * -1, closest_sphere.specular, max_t);
    color localColor = closest_sphere.color * i;

    float reflectFactor = closest_sphere.reflective;
    if (recursionDepth > MAX_RECURSION_DEPTH || reflectFactor <= 0) {
        return localColor;
    }

    vec3 reflectDir = ReflectRay(direction * -1, intersect_normal);
    color reflectColor = TraceRay(s, intersect_point, reflectDir, 0.001, max_t, recursionDepth +1);
    return localColor * (1 - reflectFactor) + reflectColor * reflectFactor;
}



void drawCast(Scene &s, vec3 &origin, int x, int y) {
    vec3 direction = CanvasToViewport(x, y);
    color c = TraceRay(s, origin, direction, 1, std::numeric_limits<float>::infinity(), 0);    
    putPixel( x, y, c);    
}

void drawSection(Scene& s, vec3 &origin, int sectionStart, int sectionEnd) {
    for (int x = sectionStart; x < sectionEnd; ++x) {
        for (int y = -canvasHeight / 2; y < canvasHeight / 2; ++y) {
            drawCast(s, origin, x, y);            
        }
    }
}


std::vector<std::future<void>> futures;

const int MAX_THREADS = 3;

void drawSceneThreaded(Scene &s) {
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    int curWidth = canvasWidth;
    SDL_LockTexture(buffer, NULL, (void **) & pixels, &curWidth);
    vec3 origin{ 0,0,0 };
    int linesPerHeight = canvasWidth / MAX_THREADS + 1;
    for (int i = 0; i < MAX_THREADS; i++) {
        int sectionStart = i * linesPerHeight;
        int sectionEnd = std::min((i + 1) * linesPerHeight, canvasWidth);
        sectionStart -= canvasWidth / 2;
        sectionEnd -= canvasWidth / 2;
        std::cout << "section start: " << sectionStart << "  section end: " << sectionEnd << " \n";
        futures.emplace_back(std::async(std::launch::async, [&s, &origin, sectionStart, sectionEnd] { drawSection(s, origin, sectionStart, sectionEnd); }));
    }
    for (auto& elem : futures) {
        elem.wait();
    }
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds> (end - begin).count() << "[ms]" << std::endl;
    SDL_UnlockTexture(buffer);
    SDL_RenderCopy(renderer, buffer, NULL, NULL);
    /*std::string test;
    std::cin >> test;*/

    //vec3 origin{ 0,0,0 };
    //vec3 viewportVec;
    //for (int x = -canvasHeight / 2; x < canvasHeight / 2; ++x) {
    //    for (int y = -canvasWidth / 2; y < canvasWidth / 2; ++y) {
    //        //futures.emplace_back(std::async(std::launch::async, drawCast, s, origin, x, y ));
    //        futures.emplace_back(std::async(std::launch::async, [&s, &origin, x, y] { drawCast(s, origin, x, y); }));
    //    }
    //}
}

void drawScene(Scene& s) {
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    vec3 origin{ 0,0,0 };
    vec3 viewportVec;
    for (int x = -canvasHeight / 2; x < canvasHeight / 2; ++x) {
        for (int y = -canvasWidth / 2; y < canvasWidth / 2; ++y) {
            drawCast(s, origin, x, y);
            //putPixel(std::get<0>(result), std::get<1>(result), std::get<2>(result));
        }
    }
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds> (end - begin).count() << "[ms]" << std::endl;
    std::string test;
    std::cin >> test;
}

int main(void) {
    SDL_Event event;    
    SDL_Window* window;
    int i;
    
    buffer = SDL_CreateTexture(renderer,
        SDL_PIXELFORMAT_BGRA8888,
        SDL_TEXTUREACCESS_STREAMING,
        canvasWidth,
        canvasHeight);

    SDL_Init(SDL_INIT_VIDEO);
    SDL_CreateWindowAndRenderer(canvasWidth, canvasHeight, 0, &window, &renderer);
    //clearScreen();
    //drawAxis();

    Scene s = Scene::MakeTestScene();
    //drawScene(s);
    
    drawSceneThreaded(s);

    SDL_RenderPresent(renderer);
    while (1) {
        if (SDL_PollEvent(&event) && event.type == SDL_QUIT)
            break;
    }
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return EXIT_SUCCESS;
}