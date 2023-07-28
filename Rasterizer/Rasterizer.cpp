// Rasterizer.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <stdio.h>
#include "DrawFunctions.h"


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
    instance{cube, vec3(-1.5, 0, 7), identMatrix, 0.75},
    instance{cube, vec3(1.25, 2.5, 7.5), MakeRotationMatrix(195,0,0), 1}
};

int main(int argc, char* argv[]) {
    SDL_Event event;
    SDL_Window* window;
    int i;

    SDL_Init(SDL_INIT_VIDEO);
    SDL_CreateWindowAndRenderer(canvasWidth, canvasHeight, 0, &window, &renderer);
    //clearScreen();
    //drawAxis();

    //Scene s = Scene::MakeTestScene();
    //drawScene(s);
    //drawSceneThreaded(s);

    while (1) {
        SDL_RenderPresent(renderer);
        //drawSceneThreaded(s);
        if (SDL_PollEvent(&event) && event.type == SDL_QUIT)
            break;
    }
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return EXIT_SUCCESS;
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
