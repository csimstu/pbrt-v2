#if defined(_MSC_VER)
#pragma once
#endif

#ifndef PBRT_SHAPES_INFINITE_SPHERE_GRID_DE_H
#define PBRT_SHAPES_INFINITE_SPHERE_GRID_DE_H

#include "distanceestimator.h"

class InfiniteSphereGridDE: public DistanceEstimator {
  public:
    // Shape Interface
    InfiniteSphereGridDE (const Transform *o2w, const Transform *w2o, bool ro, const DistanceEstimatorParams &DE_params, float cellSize);
    virtual BBox ObjectBound() const;
    virtual float Area() const;
    virtual float Evaluate(const Point &p) const;
private:
    float cellSize;
};

InfiniteSphereGridDE *CreateInfiniteSphereGridDEShape(const Transform *o2w, const Transform *w2o, bool reverseOrientation, const ParamSet &params);

#endif // PBRT_SHAPES_INFINITE_SPHERE_GRID_DE_H
