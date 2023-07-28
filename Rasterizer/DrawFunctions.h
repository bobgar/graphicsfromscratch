#pragma once
#include "HelperFunctions.h"

void putPixel(int x, int y, color c) {
    //transform x/y to our graph space
    x = canvasHeight / 2 + x;
    y = canvasWidth / 2 - y;
    //set color and draw a pixel
    SDL_SetRenderDrawColor(renderer, c.r, c.g, c.b, c.a);
    SDL_RenderDrawPoint(renderer, x, y);
}

void drawLine(point2D p0, point2D p1, color c) {
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

void DrawWireframeTriangle(point2D p0, point2D  p1, point2D  p2, color c) {
    drawLine(p0, p1, c);
    drawLine(p1, p2, c);
    drawLine(p0, p2, c);
}


void RenderTriangle(triangle t, std::vector<point2D> projected) {
    DrawWireframeTriangle(projected[t.v0],
        projected[t.v1],
        projected[t.v2],
        t.c);
}

void RenderModel(model model, matrix4 transform) {
    std::vector<point2D> projected;
    for (int i = 0; i < model.vertices.size(); i++) {
        vec3 vertex = model.vertices[i];
        vec4 vertexH = vec4{vertex.x, vertex.y, vertex.z, 1};
        projected.push_back(ProjectVertex(VecMatrixMultiply(transform, vertexH)));
    }
    for (int i = 0; i < model.triangles.size(); i++) {
        RenderTriangle(model.triangles[i], projected);
    }
}

void RenderScene(camera c, std::vector<instance> instances) {
    matrix4 cameraMatrix = MatrixMultiply(Transposed(c.orientation), MakeTranslationMatrix(c.position * -1));

    for (int i = 0; i < instances.size(); i++) {
        matrix4 transform = MatrixMultiply(cameraMatrix, instances[i].transform);
        RenderModel(instances[i].model, transform);
    }
}

void clearScreen() {
    SDL_SetRenderDrawColor(renderer, backgroundColor.r, backgroundColor.g, backgroundColor.b, backgroundColor.a);
    SDL_RenderClear(renderer);
}