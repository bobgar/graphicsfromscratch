#pragma once
#include <SDL.h>

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
