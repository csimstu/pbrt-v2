#if defined(_MSC_VER)
#pragma once
#endif

#ifndef PBRT_SHAPES_DISTANCE_ESTIMATOR_H
#define PBRT_SHAPES_DISTANCE_ESTIMATOR_H

#include "shape.h"

class DistanceEstimator: public Shape {
  public:
    // Shape Interface
    DistanceEstimator (const Transform *o2w, const Transform *w2o, bool ro, float rad,
           float zmin, float zmax, float phiMax);
    BBox ObjectBound() const;
    bool Intersect(const Ray &ray, float *tHit,
        float *rayEpsilon, DifferentialGeometry *dg) const;
    bool IntersectP(const Ray &ray) const;
    float Area() const;
private:
    // Sphere Private Data
    float radius;
    float phiMax;
    float zmin, zmax;
    float thetaMin, thetaMax;
};

DistanceEstimator *CreateDistanceEstimatorShape(const Transform *o2w, const Transform *w2o,
        bool reverseOrientation, const ParamSet &params);

#endif // PBRT_SHAPES_DISTANCE_ESTIMATOR_H
