#pragma once
#include "Factory.h"

const color backgroundColor = color{ 128, 128, 128, 255 };

const matrix4 identMatrix = matrix4{ 1,0,0,0,
                                    0,1,0,0,
                                    0,0,1,0,
                                    0,0,0,1 };

const int canvasHeight = 400;
const int canvasWidth = 400;

const float viewportWidth = 1;
const float viewportHeight = 1;

const float zPlane = 1.0;

SDL_Renderer* renderer;
camera cam = camera{ vec3(-3, 1, 2), MakeRotationMatrix(-30,0,0) };