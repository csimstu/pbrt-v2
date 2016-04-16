#include "shapes/distanceestimator.h"
#include "geometry.h"
#include "paramset.h"

DistanceEstimator::DistanceEstimator(const Transform *o2w, const Transform *w2o, bool ro, float rad)
    : Shape(o2w, w2o, ro) {
    radius = rad;
    maxIters = 1000;
    hitEpsilon = 1e-6f;
    rayEpsilonMultiplier = 2.0f;
    normalEpsilon = 1e-4f;
}

BBox DistanceEstimator::ObjectBound() const {
    return BBox(Point(-radius, -radius, -radius),
                Point( radius,  radius, radius));
}

bool DistanceEstimator::Intersect(const Ray &r, float *tHit, float *rayEpsilon,
                       DifferentialGeometry *dg) const {
  bool succeed = DoesIntersect(r, tHit);
  if (!succeed) return false;

  Ray ray;
  (*WorldToObject)(r, &ray);
  Point p = ray(*tHit);
  *rayEpsilon = hitEpsilon * rayEpsilonMultiplier;
  Vector n = CalculateNormal(p, normalEpsilon);
  Vector DPDU, DPDV;
  CoordinateSystem(n, &DPDU, &DPDV);
  *dg = DifferentialGeometry(p, DPDU, DPDV, Normal(), Normal(), 0, 0, this);
  return true;
}


bool DistanceEstimator::IntersectP(const Ray &r) const {
  return DoesIntersect(r, NULL);
}


float DistanceEstimator::Area() const {
    return 4 * M_PI * radius * radius;
}

DistanceEstimator *CreateDistanceEstimatorShape(const Transform *o2w, const Transform *w2o,
        bool reverseOrientation, const ParamSet &params) {
    float radius = params.FindOneFloat("radius", 1.f);
    return new DistanceEstimator(o2w, w2o, reverseOrientation, radius);
}

float DistanceEstimator::Evaluate(const Point &p) const {
  return sqrt(p.x * p.x + p.y * p.y + p.z * p.z) - radius;
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
  for (int itr = 0; ray.mint <= t && t <= ray.maxt && itr < maxIters; itr++) {
    float dist = Evaluate(ray(t));
    if (fabs(dist) < hitEpsilon) {
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
