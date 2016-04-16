#if defined(_MSC_VER)
#pragma once
#endif

#ifndef PBRT_SHAPES_DISTANCE_ESTIMATOR_H
#define PBRT_SHAPES_DISTANCE_ESTIMATOR_H

#include "shape.h"

class DistanceEstimator: public Shape {
  public:
    // Shape Interface
    DistanceEstimator (const Transform *o2w, const Transform *w2o, bool ro, float rad);
    BBox ObjectBound() const;
    bool Intersect(const Ray &ray, float *tHit,
        float *rayEpsilon, DifferentialGeometry *dg) const;
    bool IntersectP(const Ray &ray) const;
    float Area() const;

    virtual float Evaluate(const Point &p) const;
private:
    Vector CalculateNormal(const Point& pos, float eps) const;
    bool DoesIntersect(const Ray &r, float *tHit) const;

    // Sphere Private Data
    float radius;
    int maxIters; // Number of steps along the ray until we give up (default 1000)
    float hitEpsilon; // how close to the surface we must be before we say we "hit" it 
    float rayEpsilonMultiplier; // how much we multiply hitEpsilon by to get rayEpsilon 
    float normalEpsilon; // The epsilon we send to CalculateNormal()
};

DistanceEstimator *CreateDistanceEstimatorShape(const Transform *o2w, const Transform *w2o,
    bool reverseOrientation, const ParamSet &params);

#endif // PBRT_SHAPES_DISTANCE_ESTIMATOR_H
