#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Cloth::Cloth(double radius, int num_points) {
  this->radius = radius;
  this->num_points = num_points;

  buildGrid();
}

Cloth::~Cloth() {
  point_masses.clear();
  springs.clear();
}

void Cloth::buildFibonacciSphere() {
  // Translated from https://stackoverflow.com/a/26127012/5901346
  // Populates point_masses with samples number of points distributed in a sphere
  this->point_masses.clear();
  
  vector<PointMass> points = vector<PointMass>();
  float phi = PI * (3. - std::sqrt(5.));  // golden angle in radians

  for (int i = 0; i < this->num_points; i++) {
    float y = 1 - (i / float(this->num_points - 1)) * 2;  // y goes from 1 to -1
    float radius = sqrt(1 - y * y) * this->radius;  // radius at y

    float theta = phi * i; // golden angle increment

    float x = cos(theta) * radius;
    float z = sin(theta) * radius;

    point_masses.push_back(PointMass(Vector3D(x, y, z), false));
  }
}

void Cloth::buildGrid() {
  // TODO (Part 1): Build a grid of masses and springs.
  
  this->buildFibonacciSphere();
  
  // TODO add constraints
}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
//  double mass = width * height * cp->density / num_width_points / num_height_points;
//  double delta_t = 1.0f / frames_per_sec / simulation_steps;
//
//  // TODO (Part 2): Compute total force acting on each point mass.
//
//  Vector3D external_force;
//  for (int i = 0; i < external_accelerations.size(); i++) {
//    external_force += mass * external_accelerations[i];
//  }
//
//  // Set the external force on the point masses
//  for (int i = 0; i < num_height_points; i++) {
//    for (int j = 0; j < num_width_points; j++) {
//      int idx = (i * num_width_points) + j;
//      point_masses[idx].forces = external_force;
//    }
//  }
//
//  for (int i = 0; i < springs.size(); i++) {
//    if ((springs[i].spring_type == STRUCTURAL && cp->enable_structural_constraints) ||
//        (springs[i].spring_type == SHEARING && cp->enable_shearing_constraints) ||
//        (springs[i].spring_type == BENDING && cp->enable_bending_constraints)) {
//
//
//      Vector3D difference = springs[i].pm_a->position - springs[i].pm_b->position;
//      double dist = difference.norm();
//      double ks = springs[i].spring_type == BENDING ? 0.2 * cp->ks : cp->ks;
//      double force_magnitude = ks * (dist - springs[i].rest_length);
//
//      springs[i].pm_a->forces += -force_magnitude * difference.unit();
//      springs[i].pm_b->forces += force_magnitude * difference.unit();
//    }
//  }
//
//
//  // TODO (Part 2): Use Verlet integration to compute new point mass positions
//  for (int i = 0; i < num_height_points; i++) {
//    for (int j = 0; j < num_width_points; j++) {
//      int idx = (i * num_width_points) + j;
//      if (!point_masses[idx].pinned) {
//        Vector3D position = point_masses[idx].position;
//        Vector3D last_position = point_masses[idx].last_position;
//        point_masses[idx].position = position + (1.0 - cp->damping / 100.0) * (position - last_position) + point_masses[idx].forces / mass * delta_t * delta_t;
//        point_masses[idx].last_position = position;
//      }
//    }
//  }
//
//  // TODO (Part 4): Handle self-collisions.
//  build_spatial_map();
//  for (int i = 0; i < point_masses.size(); i++) {
//    self_collide(point_masses[i], simulation_steps);
//  }
//
//
//  // TODO (Part 3): Handle collisions with other primitives.
//  for (int k = 0; k < collision_objects->size(); k++) {
//    for (int i = 0; i < num_height_points; i++) {
//      for (int j = 0; j < num_width_points; j++) {
//        int idx = (i * num_width_points) + j;
//        (*collision_objects)[k]->collide(point_masses[idx]);
//      }
//    }
//  }
//
//
//  // TODO (Part 2): Constrain the changes to be such that the spring does not change
//  // in length more than 10% per timestep [Provot 1995].
//  for (int i = 0; i < springs.size(); i++) {
//    if ((springs[i].spring_type == STRUCTURAL && cp->enable_structural_constraints) ||
//        (springs[i].spring_type == SHEARING && cp->enable_shearing_constraints) ||
//        (springs[i].spring_type == BENDING && cp->enable_bending_constraints)) {
//
//        Vector3D difference = springs[i].pm_a->position - springs[i].pm_b->position;
//        double dist = difference.norm();
//        if (dist > springs[i].rest_length * 1.1) {
//          double offset = dist - springs[i].rest_length * 1.1;
//
//          if (!springs[i].pm_a->pinned && !springs[i].pm_b->pinned) {
//            springs[i].pm_a->position += -0.5 * offset * difference.unit();
//            springs[i].pm_b->position += 0.5 * offset * difference.unit();
//          } else if (springs[i].pm_a->pinned && !springs[i].pm_b->pinned) {
//            springs[i].pm_b->position += offset * difference.unit();
//          } else if (!springs[i].pm_a->pinned && springs[i].pm_b->pinned) {
//            springs[i].pm_a->position += -offset * difference.unit();
//          }
//        }
//
//    }
//  }

}

void Cloth::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.

  for (int i = 0; i < point_masses.size(); i++) {
    float id = hash_position(point_masses[i].position);
    if (map.count(id) == 0) {
      map.insert({id, new vector<PointMass *>()});
    }
    map[id]->push_back(&point_masses[i]);
  }
}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
  // TODO (Part 4): Handle self-collision for a given point mass.
//  float id = hash_position(pm.position);
//  if (map.count(id) == 0) {
//    return;
//  } else {
//    Vector3D correction;
//    int count = 0;
//    vector<PointMass *> *masses = map[id];
//    for (int i = 0; i < masses->size(); i++){
//      if ((*masses)[i] != &pm) {
//        float dist = (pm.position - (*masses)[i]->position).norm();
//        if (dist <= 2 * thickness){
//          count++;
//          correction += (pm.position - (*masses)[i]->position).unit() * (2 * thickness - dist);
//        }
//      }
//    }
//    if (count > 0) {
//      correction = correction / count / simulation_steps;
//      pm.position += correction;
//    }
//  }
}

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.
//  float w = 3.0 * width / num_width_points;
//  float h = 3.0 * height / num_height_points;
//  float t = max(w, h);
//
//  //Vector3D nearestBox((pos.x - fmod(pos.x, w)) / w, (pos.y - fmod(pos.y, h)) / h, (pos.z - fmod(pos.z, t)) / t);
//  Vector3D nearestBox(pos.x / w, pos.y / h, pos.z / t);
//  int id = (int)nearestBox.x * 100 + (int)nearestBox.y * 10 + (int)nearestBox.z;
//  return id;
  return 0;
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Cloth::reset() {
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}
