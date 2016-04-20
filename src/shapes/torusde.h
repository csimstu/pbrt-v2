#if defined(_MSC_VER)
#pragma once
#endif

#ifndef PBRT_SHAPES_TORUS_DE_H
#define PBRT_SHAPES_TORUS_DE_H

#include "distanceestimator.h"

class TorusDE: public DistanceEstimator {
  public:
    // Shape Interface
    TorusDE (const Transform *o2w, const Transform *w2o, bool ro, const DistanceEstimatorParams &DE_params, float rad1, float rad2);
    virtual BBox ObjectBound() const;
    virtual float Area() const;
    virtual float Evaluate(const Point &p) const;
private:
    float rad1, rad2;
};

TorusDE *CreateTorusDEShape(const Transform *o2w, const Transform *w2o, bool reverseOrientation, const ParamSet &params);

#endif // PBRT_SHAPES_SPHERE_DE_H
