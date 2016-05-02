#if defined(_MSC_VER)
#pragma once
#endif

#ifndef PBRT_CAMERAS_REALISTIC_H
#define PBRT_CAMERAS_REALISTIC_H

#include "pbrt.h"
#include "camera.h"
#include "film.h"

class RealisticCamera : public Camera {
public:
    RealisticCamera(const AnimatedTransform &cam2world,
        float sopen, float sclose,
        float filmdistance, float fstop,
        string specfile, float filmdiag,
        Film *film);
    float GenerateRay(const CameraSample &sample, Ray *) const;
private:
    struct LensInterface {
      float radius;
      float dz;
      float z;
      float n;
      float aperture;
      Point c;
    };

    float filmdistance;
    float fstop;
    string specfile;
    float filmdiag;
    vector<LensInterface> lensdata;

    float filmZ;
};


RealisticCamera *CreateRealisticCamera(const ParamSet &params,
        const AnimatedTransform &cam2world, Film *film);

#endif // PBRT_CAMERAS_REALISTIC_H
