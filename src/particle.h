#include "CGL/CGL.h"
#include "CGL/misc.h"
#include "CGL/vector3D.h"

using namespace CGL;


struct Particle {
  Particle(Vector3D position, double radius, double density)
      : start_position(position), position(position),
        last_position(position), radius(radius), density(density) {}

    double mass() {
        return density * (4.0/3.0) * PI * radius * radius * radius;
    }

    // static values
    Vector3D start_position;
    double density;

    // dynamic values
    Vector3D position;
    Vector3D last_position;
    Vector3D forces;
    Vector3D accel;
    Vector3D velocity;
    double radius;

};
