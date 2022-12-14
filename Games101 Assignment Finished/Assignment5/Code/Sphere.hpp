#pragma once

#include "Object.hpp"
#include "Vector.hpp"

class Sphere : public Object   //Derived Class
{
public:
    Sphere(const Vector3f& c, const float& r)
        : center(c)
        , radius(r)
        , radius2(r * r)
    {}

    bool intersect(const Vector3f& orig, const Vector3f& dir, float& tnear, uint32_t&, Vector2f&) const override   //Do Intersection
    {
        // analytic solution
        Vector3f L = orig - center;
        float a = dotProduct(dir, dir);
        float b = 2 * dotProduct(dir, L);
        float c = dotProduct(L, L) - radius2;
        float t0, t1;
        if (!solveQuadratic(a, b, c, t0, t1))   //Address Delivery
            return false;   //No Real Solution
        if (t0 < 0)   //t1>t0,t0<0,that the orig is in the sphere
            t0 = t1;   
        if (t0 < 0)
            return false;
        tnear = t0; //t1>t0>=0 , that the closer point is t0

        return true;
    }

    void getSurfaceProperties(const Vector3f& P, const Vector3f&, const uint32_t&, const Vector2f&,
                              Vector3f& N, Vector2f&) const override
    {
        N = normalize(P - center);   //dir
    }

    Vector3f center;
    float radius, radius2;
};
