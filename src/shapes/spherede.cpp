#include "spherede.h"
#include "paramset.h"

SphereDE::SphereDE(const Transform *o2w, const Transform *w2o, bool ro, const DistanceEstimatorParams &DE_params, float rad)
  : DistanceEstimator(o2w, w2o, ro, DE_params), radius(rad){
}

BBox SphereDE::ObjectBound() const {
    return BBox(Point(-radius, -radius, -radius),
                Point( radius,  radius, radius));
}

float SphereDE::Area() const {
    return 4 * M_PI * radius * radius;
}

float SphereDE::Evaluate(const Point &p) const {
  return sqrt(p.x * p.x + p.y * p.y + p.z * p.z) - radius;

}
SphereDE *CreateSphereDEShape(const Transform *o2w, const Transform *w2o, bool reverseOrientation, const ParamSet &params) {
    float radius = params.FindOneFloat("radius", 1.f);
    return new SphereDE(o2w, w2o, reverseOrientation, DistanceEstimatorParams(), radius);
}
