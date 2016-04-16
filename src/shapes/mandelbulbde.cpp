#include "mandelbulbde.h"
#include "paramset.h"

MandelbulbDE::MandelbulbDE (const Transform *o2w, const Transform *w2o, bool ro, const DistanceEstimatorParams &DE_params, int fractalIters, int mandelbulbPower)
  : DistanceEstimator(o2w, w2o, ro, DE_params), fractalIters(fractalIters), mandelbulbPower(mandelbulbPower) {
  }

BBox MandelbulbDE::ObjectBound() const {
  return BBox(Point(-1, -1, -1),
      Point(1, 1, 1));
}

float MandelbulbDE::Area() const {
  return 1e100;
}

float MandelbulbDE::Evaluate(const Point &c) const {
    const float Power = (float)mandelbulbPower;
    Point z = c;
    float dr = 1.0;
    float r = 0.0;
    for (int i = 0; i < fractalIters; ++i) {
        r = (z-Point(0,0,0)).Length();
        // Escaped orbit
        if (r > 2.0f) {
            break;
        }
        dr =  pow( r, Power-1.0)*Power*dr + 1.0;
        
        // In original Mandelbrot this is z = z^2 + c
        // Mandelbulb does this by analogy, replacing the square with
        // scaling and rotating
        
        // Polar coordinates
        float theta = acos(z.z/r);
        float phi = atan2(z.y,z.x);
        
        // "square" z (really scaling and rotating)
        float zr = pow(r,Power);
        theta = theta*Power;
        phi = phi*Power;
        
        // Cartesian coordinates
        z = zr * Point(sin(theta)*cos(phi), sin(phi)*sin(theta), cos(theta));
        z += c;
    }
    //
    return 0.5 * (r/dr) * log(r);
}

MandelbulbDE *CreateMandelbulbDEShape(const Transform *o2w, const Transform *w2o, bool reverseOrientation, const ParamSet &params) {
  int fractalIters = params.FindOneInt("fractalIters", 1000);
  int mandelbulbPower = params.FindOneInt("mandelbulbPower", 8);
  return new MandelbulbDE(o2w, w2o, reverseOrientation, DistanceEstimatorParams(params), fractalIters, mandelbulbPower);
}
