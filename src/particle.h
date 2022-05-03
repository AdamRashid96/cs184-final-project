#include "CGL/CGL.h"
#include "CGL/misc.h"
#include "CGL/vector3D.h"

using namespace CGL;

enum ParticleType { FUEL, SOOT };

struct Particle {
  Particle(Vector3D position, double radius, double density)
      : start_position(position), position(position),
        radius(radius), density(density) {}

    double mass() {
        return density * (4.0/3.0) * PI * radius * radius * radius;
    }

    double thermal_mass() {
        return 0; // TODO: fix
    }

    // static values
    Vector3D start_position;
    double density;

    // dynamic values
    Vector3D position;
    //Vector3D last_position; // TODO: remove
    Vector3D force;
    Vector3D accel;
    Vector3D velocity;
    double radius;
    double temperature;
    double heat_transfer;
    ParticleType type;
    double soot_mass;


};
