#include "torusde.h"
#include "paramset.h"

TorusDE::TorusDE(const Transform *o2w, const Transform *w2o, bool ro, const DistanceEstimatorParams &DE_params, float rad1, float rad2)
  : DistanceEstimator(o2w, w2o, ro, DE_params), rad1(rad1), rad2(rad2) {
}

BBox TorusDE::ObjectBound() const {
    return BBox(Point(-1e10, -1e10, -1e10),
                Point(1e10, 1e10, 1e10));
}

float TorusDE::Area() const {
    return 2 * M_PI * rad1 * 2 * M_PI * rad2;
}

float TorusDE::Evaluate(const Point &p) const {
  float t = sqrt(p.x * p.x + p.y * p.y) - rad2;
  return sqrt(t * t + p.z * p.z) - rad1;

}
TorusDE *CreateTorusDEShape(const Transform *o2w, const Transform *w2o, bool reverseOrientation, const ParamSet &params) {
    float rad1 = params.FindOneFloat("r", 1.f);
    float rad2 = params.FindOneFloat("R", 1.f);
    return new TorusDE(o2w, w2o, reverseOrientation, DistanceEstimatorParams(params), rad1, rad2);
}
