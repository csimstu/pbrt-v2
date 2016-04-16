#if defined(_MSC_VER)
#pragma once
#endif

#ifndef PBRT_SHAPES_SPHERE_DE_H
#define PBRT_SHAPES_SPHERE_DE_H

#include "distanceestimator.h"

class SphereDE: public DistanceEstimator {
  public:
    // Shape Interface
    SphereDE (const Transform *o2w, const Transform *w2o, bool ro, const DistanceEstimatorParams &DE_params, float rad);
    virtual BBox ObjectBound() const;
    virtual float Area() const;
    virtual float Evaluate(const Point &p) const;
private:
    float radius;
};

SphereDE *CreateSphereDEShape(const Transform *o2w, const Transform *w2o, bool reverseOrientation, const ParamSet &params);

#endif // PBRT_SHAPES_SPHERE_DE_H
