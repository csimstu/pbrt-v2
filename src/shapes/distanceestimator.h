#if defined(_MSC_VER)
#pragma once
#endif

#ifndef PBRT_SHAPES_DISTANCE_ESTIMATOR_H
#define PBRT_SHAPES_DISTANCE_ESTIMATOR_H

#include "shape.h"

struct DistanceEstimatorParams {
  DistanceEstimatorParams();
  DistanceEstimatorParams(const ParamSet &params);

  int maxIters; // Number of steps along the ray until we give up (default 1000)
  float hitEpsilon; // how close to the surface we must be before we say we "hit" it 
  float rayEpsilonMultiplier; // how much we multiply hitEpsilon by to get rayEpsilon 
  float normalEpsilon; // The epsilon we send to CalculateNormal()
};

class DistanceEstimator: public Shape {
  public:
    // Shape Interface
    DistanceEstimator (const Transform *o2w, const Transform *w2o, bool ro, const DistanceEstimatorParams &DE_params);
    bool Intersect(const Ray &ray, float *tHit, float *rayEpsilon, DifferentialGeometry *dg) const;
    bool IntersectP(const Ray &ray) const;

    virtual BBox ObjectBound() const = 0;
    virtual float Area() const = 0;
    virtual float Evaluate(const Point &p) const = 0;
  private:
    Vector CalculateNormal(const Point& pos, float eps) const;
    bool DoesIntersect(const Ray &r, float *tHit) const;
    DistanceEstimatorParams DE_params;
};

#endif // PBRT_SHAPES_DISTANCE_ESTIMATOR_H
