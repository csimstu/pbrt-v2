#include "shapes/distanceestimator.h"
#include "geometry.h"
#include "paramset.h"

DistanceEstimator::DistanceEstimator (const Transform *o2w, const Transform *w2o, bool ro, const DistanceEstimatorParams &DE_params) 
    : Shape(o2w, w2o, ro), DE_params(DE_params) {
}

bool DistanceEstimator::Intersect(const Ray &r, float *tHit, float *rayEpsilon,
                       DifferentialGeometry *dg) const {
  bool succeed = DoesIntersect(r, tHit);
  if (!succeed) return false;

  Ray ray;
  (*WorldToObject)(r, &ray);
  Point p = ray(*tHit);
  *rayEpsilon = DE_params.hitEpsilon * DE_params.rayEpsilonMultiplier;
  Vector n = CalculateNormal(p, DE_params.normalEpsilon);
  Vector DPDU, DPDV;
  CoordinateSystem(n, &DPDU, &DPDV);
  *dg = DifferentialGeometry(p, DPDU, DPDV, Normal(), Normal(), 0, 0, this);
  return true;
}

bool DistanceEstimator::IntersectP(const Ray &r) const {
  return DoesIntersect(r, NULL);
}

Vector DistanceEstimator::CalculateNormal(const Point& pos, float eps) const {
  const Vector v1 = Vector( 1.0,-1.0,-1.0);
  const Vector v2 = Vector(-1.0,-1.0, 1.0);
  const Vector v3 = Vector(-1.0, 1.0,-1.0);
  const Vector v4 = Vector( 1.0, 1.0, 1.0);

  return Normalize( 
      v1 * Evaluate( pos + v1*eps ) +
      v2 * Evaluate( pos + v2*eps ) +
      v3 * Evaluate( pos + v3*eps ) +
      v4 * Evaluate( pos + v4*eps ) );
}

bool DistanceEstimator::DoesIntersect(const Ray &r, float *tHit) const {
  Ray ray;
  (*WorldToObject)(r, &ray);
  float t = Evaluate(ray(ray.mint));
  bool intersected = false;
  for (int itr = 0; ray.mint <= t && t <= ray.maxt && itr < DE_params.maxIters; itr++) {
    float dist = Evaluate(ray(t));
    if (fabs(dist) < DE_params.hitEpsilon) {
      intersected = true;
      break;
    }
    t += dist;
  }
  if (!intersected) 
    return false;
  if (tHit) *tHit = t;
  return true;
}
