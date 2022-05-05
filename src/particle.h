#include "CGL/CGL.h"
#include "CGL/misc.h"
#include "CGL/vector3D.h"

using namespace CGL;

enum ParticleType { FUEL, SOOT };

struct Particle {
  Particle(ParticleType type, double density, 
           double specific_heat_capacity, Vector3D position, double radius, double temperature)
      : type(type), density(density), specific_heat_capacity(specific_heat_capacity), 
        position(position), radius(radius), temperature(temperature) {

    force = Vector3D(0.0);
    velocity = Vector3D(0.0);
    heat_transfer = 0.0;
    soot_mass = 0.0;
  }

  double mass() {
    return density * (4.0/3.0) * PI * radius * radius * radius;
  }

  double thermal_mass() {
    return mass() * specific_heat_capacity;
  }

  void setMass(double desiredMass) {
    radius = pow(desiredMass / (density * (4.0/3.0) * PI), 1.0 / 3.0);
  }

  // Static values
  ParticleType type;
  double density;
  double specific_heat_capacity;

  // dynamic values
  Vector3D position;
  Vector3D force;
  Vector3D velocity;
  double radius;
  double temperature;
  double heat_transfer;
  double soot_mass;
};
