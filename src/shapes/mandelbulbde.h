#if defined(_MSC_VER)
#pragma once
#endif

#ifndef PBRT_MANDELBULB_DE_H
#define PBRT_MANDELBULB_DE_H

#include "distanceestimator.h"

class MandelbulbDE: public DistanceEstimator {
  public:
    // Shape Interface
    MandelbulbDE (const Transform *o2w, const Transform *w2o, bool ro, const DistanceEstimatorParams &DE_params, int fractalIters, int mandelbulbPower);
    virtual BBox ObjectBound() const;
    virtual float Area() const;
    virtual float Evaluate(const Point &p) const;
private:
    int fractalIters;
    int mandelbulbPower;
};

MandelbulbDE *CreateMandelbulbDEShape(const Transform *o2w, const Transform *w2o, bool reverseOrientation, const ParamSet &params);

#endif // PBRT_MANDELBULB_DE_H
