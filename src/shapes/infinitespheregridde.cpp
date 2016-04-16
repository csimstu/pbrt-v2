#include "infinitespheregridde.h"
#include "paramset.h"

InfiniteSphereGridDE::InfiniteSphereGridDE(const Transform *o2w, const Transform *w2o, bool ro, const DistanceEstimatorParams &DE_params, float cellSize)
  : DistanceEstimator(o2w, w2o, ro, DE_params), cellSize(cellSize){
  }

BBox InfiniteSphereGridDE::ObjectBound() const {
  return BBox(Point(-1e10, -1e10, -1e10),
      Point(1e10, 1e10, 1e10));
}

float InfiniteSphereGridDE::Area() const {
  return 1e100;
}

float InfiniteSphereGridDE::Evaluate(const Point &p) const {
  float rx = remainder(p.x, cellSize); 
  float ry = remainder(p.y, cellSize); 
  float rz = remainder(p.z, cellSize); 
  return sqrt(rx * rx + ry * ry + rz * rz) - 1.0f;
}

InfiniteSphereGridDE *CreateInfiniteSphereGridDEShape(const Transform *o2w, const Transform *w2o, bool reverseOrientation, const ParamSet &params) {
  float cellSize = params.FindOneFloat("cellSize", 1.f);
  return new InfiniteSphereGridDE(o2w, w2o, reverseOrientation, DistanceEstimatorParams(params), cellSize);
}
