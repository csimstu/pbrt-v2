#include "stdafx.h"
#include "cameras/realistic.h"
#include "paramset.h"
#include "sampler.h"
#include "montecarlo.h"
#include <fstream>
#include <sstream>
#include <cstdio>

RealisticCamera::RealisticCamera(const AnimatedTransform &cam2world,
    float sopen, float sclose,
    float filmdistance, float fstop,
    string specfile, float filmdiag,
    Film *film)
  : Camera(cam2world, sopen, sclose, film), 
  filmdistance(filmdistance), 
  fstop(fstop), 
  specfile(specfile), 
  filmdiag(filmdiag) 
  {
    std::ifstream spec(specfile);
    if (spec.is_open()) {
      string line;
      float cur_z = 0;
      while (getline(spec, line)) {
        if (line[0] == '#') continue;
        std::stringstream ss(line);
        LensInterface li;
        ss >> li.radius >> li.dz >> li.n >> li.aperture;
        if (li.radius == 0) li.n = 1.0f;

        li.c = Point(0, 0, cur_z - li.radius);
        li.z = cur_z;
        cur_z -= li.dz;
        lensdata.push_back(li);
      }
    }

    filmZ = -filmdistance;
    for (LensInterface &I : lensdata)
      filmZ -= I.dz;
  }


float RealisticCamera::GenerateRay(const CameraSample &sample,
    Ray *ray) const {
  Point Pfilm(0, 0, filmZ);
  {
    Vector tv(film->xResolution, film->yResolution, 0);
    tv = Normalize(tv);
    Pfilm.x = tv.x * filmdiag * (-sample.imageX / film->xResolution + 0.5f);
    Pfilm.y = tv.y * filmdiag * (sample.imageY / film->yResolution - 0.5f);
  }


  float lensU, lensV;
  ConcentricSampleDisk(sample.lensU, sample.lensV, &lensU, &lensV);
  //printf("U,V=(%.3f,%.3f)\n", lensU, lensV);
  const LensInterface &backI = lensdata.back();
  lensU *= backI.aperture / 2;
  lensV *= backI.aperture / 2;
  float backZ = filmdistance + filmZ;
  //printf("before backZ=%.3f, backI.rad=%.3f,backI.aperture=%.3f\n", backZ, backI.radius, backI.aperture);
  if (backI.radius > 0) {
    backZ -= backI.radius - sqrt(backI.radius * backI.radius - backI.aperture * backI.aperture / 4);
  } else {
    backZ += -backI.radius - sqrt(backI.radius * backI.radius - backI.aperture * backI.aperture / 4);
  }
  //printf("after backZ=%.3f\n", backZ);
  Point PBackI = Point(lensU, lensV, backZ);
  Vector rayDir = Normalize(PBackI - Pfilm);
  *ray = Ray(Pfilm, rayDir, 0, INFINITY);
  float cosDelta = Dot(ray->d, Vector(0,0,1.f));
  float weight = pow(cosDelta, 4) * M_PI * (backI.aperture / 2) * (backI.aperture / 2) / ((backZ - filmZ) * (backZ - filmZ));
  
  float n_pre = 1.f; // Initially in air
  for (int i = (int)lensdata.size() - 1; i >= 0; i--) {
    const LensInterface &I = lensdata[i];
    Point P; // intersection on the surface
    Vector N; // norm of the surface
    if (fabs(I.radius) < 1e-6f) {
      //aperture stop
      float t = (I.z - ray->o.z) / ray->d.z;
      if (t < 0) {
        return 0.0f;
      }
      P = (*ray)(t);
      if (Distance(P, Point(0,0,I.z)) > fstop / 2) {
        return 0.0f;
      }
      N = Vector(0,0,-1.f);
    } else {
      // First test aperture

      // Then find intersections
      float a = ray->d.LengthSquared();
      float b = 2.f * Dot(ray->o - I.c, ray->d);
      float c = (ray->o - I.c).LengthSquared() - I.radius * I.radius;
      float delta = b*b - 4 * a * c;
      if (delta < 0) {
        return 0.0f;
      }
      float sqrtdelta = sqrt(delta);
      float t1 = (-b + sqrtdelta) / (2 * a);
      float t2 = (-b - sqrtdelta) / (2 * a);
      Point p1 = (*ray)(t1);
      Point p2 = (*ray)(t2);
      if (t1 > 0 && t2 > 0) {
        P = t1 < t2 ? p1 : p2;
        //else
        //  P = t1 > t2 ? p1 : p2;
      } else {
        if (t1 > 0) {
          P = p1;
        } else if (t2 > 0) {
          P = p2;
        } else {
          return 0.0f;
        }
      }

      if (P.x*P.x+P.y*P.y > I.aperture * I.aperture / 4)
        return 0.0f;
      if (I.radius > 0)
        N = Normalize(I.c - P);
      else
        N = Normalize(P - I.c);
    }

    float n_next = i > 0 ? lensdata[i-1].n : 1.f;
    if (fabs(I.radius) > 1e-6f) {
      float mu = n_pre / n_next;
      float tmp = 1 - mu * mu * (1 - Dot(ray->d, N) * Dot(ray->d, N));
      if (tmp < 0) {
        return 0.0f;
      }
      float gamma = -mu * Dot(ray->d, N) - sqrt(tmp);
      Vector T = Normalize(mu * ray->d + gamma * N);
      *ray = Ray(P, T, 0, INFINITY);
    }
    n_pre = n_next;
  }

  CameraToWorld(*ray, ray);
  ray->d = Normalize(ray->d);
  return weight;
}

RealisticCamera *CreateRealisticCamera(const ParamSet &params,
    const AnimatedTransform &cam2world, Film *film) {
  // Extract common camera parameters from \use{ParamSet}
  float shutteropen = params.FindOneFloat("shutteropen", -1);
  float shutterclose = params.FindOneFloat("shutterclose", -1);

  // Realistic camera-specific parameters
  string specfile = params.FindOneString("specfile", "");
  float filmdistance = params.FindOneFloat("filmdistance", 70.0); // about 70 mm default to film
  float fstop = params.FindOneFloat("aperture_diameter", 1.0);
  float filmdiag = params.FindOneFloat("filmdiag", 35.0);
  assert(shutteropen != -1 && shutterclose != -1 && filmdistance!= -1);
  if (specfile == "") {
    Severe( "No lens spec file supplied!\n" );
  }
  return new RealisticCamera(cam2world, shutteropen, shutterclose, filmdistance, fstop, specfile, filmdiag, film);
}
