#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Cloth::Cloth(double width, double height, int num_width_points,
             int num_height_points, float thickness) {
  this->width = width;
  this->height = height;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->thickness = thickness;

  buildGrid();
  buildClothMesh();
}

Cloth::~Cloth() {
  point_masses.clear();
  springs.clear();

  if (clothMesh) {
    delete clothMesh;
  }
}

void Cloth::buildGrid() {
  // TODO (Part 1): Build a grid of masses and springs.
  Vector3D up, right, forward;
  if (orientation == HORIZONTAL) {
    up = Vector3D(0, 0, 1);
    right = Vector3D(1, 0, 0);
    forward = Vector3D(0, 1, 0);
  } else {
    up = Vector3D(0, 1, 0);
    right = Vector3D(1, 0, 0);
    forward = Vector3D(0, 0, 1);
  }


  for (int i = 0; i < num_height_points; i++) {
    for (int j = 0; j < num_width_points; j++) {
      int idx = (i * num_width_points) + j;
      Vector3D pos = (j * width / (num_width_points - 1)) * right + (i * height / (num_height_points - 1)) * up;
      if (orientation == VERTICAL) {
        pos += forward * ((double)rand() / (double)RAND_MAX - 0.5) * 2.0 / 1000.0;
      } else {
        pos += forward;
      }
      point_masses.emplace_back(pos, false);
    }
  }

  for (int i = 0; i < pinned.size(); i++) {
    int idx = (pinned[i][1] * num_width_points) + pinned[i][0];
    point_masses[idx].pinned = true;
  }

  // Add the springs
  for (int i = 0; i < num_height_points; i++) {
    for (int j = 0; j < num_width_points; j++) {

      PointMass* a = &point_masses[(i * num_width_points) + j];

      // Point mass to the left
      if (j > 0) {
        PointMass* b = &point_masses[(i * num_width_points) + j - 1];
        springs.emplace_back(a, b, STRUCTURAL);
      }
      // Point mass above
      if (i > 0) {
        PointMass* b = &point_masses[((i - 1) * num_width_points) + j];
        springs.emplace_back(a, b, STRUCTURAL);
      }

      // Point mass two to the left
      if (j > 1) {
        PointMass* b = &point_masses[(i * num_width_points) + j - 2];
        springs.emplace_back(a, b, BENDING);
      }
      // Point mass two above
      if (i > 1) {
        PointMass* b = &point_masses[((i - 2) * num_width_points) + j];
        springs.emplace_back(a, b, BENDING);
      }

      // Point mass diagonal 
      if (i > 0 && j > 0) {
        PointMass* b = &point_masses[((i - 1) * num_width_points) + j - 1];
        springs.emplace_back(a, b, SHEARING);
      }

      if (i > 0 && j < num_width_points - 1) {
        PointMass* b = &point_masses[((i - 1) * num_width_points) + j + 1];
        springs.emplace_back(a, b, SHEARING);
      }
    }
  }




}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
  double mass = width * height * cp->density / num_width_points / num_height_points;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

  // TODO (Part 2): Compute total force acting on each point mass.

  Vector3D external_force;
  for (int i = 0; i < external_accelerations.size(); i++) {
    external_force += mass * external_accelerations[i];
  }

  // Set the external force on the point masses
  for (int i = 0; i < num_height_points; i++) {
    for (int j = 0; j < num_width_points; j++) {
      int idx = (i * num_width_points) + j;
      point_masses[idx].forces = external_force;
    }
  }

  for (int i = 0; i < springs.size(); i++) {
    if ((springs[i].spring_type == STRUCTURAL && cp->enable_structural_constraints) ||
        (springs[i].spring_type == SHEARING && cp->enable_shearing_constraints) ||
        (springs[i].spring_type == BENDING && cp->enable_bending_constraints)) {

      
      Vector3D difference = springs[i].pm_a->position - springs[i].pm_b->position;
      double dist = difference.norm();
      double ks = springs[i].spring_type == BENDING ? 0.2 * cp->ks : cp->ks;
      double force_magnitude = ks * (dist - springs[i].rest_length);

      springs[i].pm_a->forces += -force_magnitude * difference.unit();
      springs[i].pm_b->forces += force_magnitude * difference.unit();
    }
  }


  // TODO (Part 2): Use Verlet integration to compute new point mass positions
  for (int i = 0; i < num_height_points; i++) {
    for (int j = 0; j < num_width_points; j++) {
      int idx = (i * num_width_points) + j;
      if (!point_masses[idx].pinned) {
        Vector3D position = point_masses[idx].position;
        Vector3D last_position = point_masses[idx].last_position;
        point_masses[idx].position = position + (1.0 - cp->damping / 100.0) * (position - last_position) + point_masses[idx].forces / mass * delta_t * delta_t;
        point_masses[idx].last_position = position;
      }
    }
  }

  // TODO (Part 4): Handle self-collisions.
  build_spatial_map();
  for (int i = 0; i < point_masses.size(); i++) {
    self_collide(point_masses[i], simulation_steps);
  }


  // TODO (Part 3): Handle collisions with other primitives.
  for (int k = 0; k < collision_objects->size(); k++) {
    for (int i = 0; i < num_height_points; i++) {
      for (int j = 0; j < num_width_points; j++) {
        int idx = (i * num_width_points) + j;
        (*collision_objects)[k]->collide(point_masses[idx]);
      }
    }
  }


  // TODO (Part 2): Constrain the changes to be such that the spring does not change
  // in length more than 10% per timestep [Provot 1995].
  for (int i = 0; i < springs.size(); i++) {
    if ((springs[i].spring_type == STRUCTURAL && cp->enable_structural_constraints) ||
        (springs[i].spring_type == SHEARING && cp->enable_shearing_constraints) ||
        (springs[i].spring_type == BENDING && cp->enable_bending_constraints)) {

        Vector3D difference = springs[i].pm_a->position - springs[i].pm_b->position;
        double dist = difference.norm();
        if (dist > springs[i].rest_length * 1.1) {
          double offset = dist - springs[i].rest_length * 1.1;

          if (!springs[i].pm_a->pinned && !springs[i].pm_b->pinned) {
            springs[i].pm_a->position += -0.5 * offset * difference.unit();
            springs[i].pm_b->position += 0.5 * offset * difference.unit();
          } else if (springs[i].pm_a->pinned && !springs[i].pm_b->pinned) {
            springs[i].pm_b->position += offset * difference.unit();
          } else if (!springs[i].pm_a->pinned && springs[i].pm_b->pinned) {
            springs[i].pm_a->position += -offset * difference.unit();
          }
        }

    }
  }

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
  float id = hash_position(pm.position);
  if (map.count(id) == 0) {
    return;
  } else {
    Vector3D correction;
    int count = 0;
    vector<PointMass *> *masses = map[id];
    for (int i = 0; i < masses->size(); i++){
      if ((*masses)[i] != &pm) {
        float dist = (pm.position - (*masses)[i]->position).norm();
        if (dist <= 2 * thickness){
          count++;
          correction += (pm.position - (*masses)[i]->position).unit() * (2 * thickness - dist);
        }
      }
    }
    if (count > 0) {
      correction = correction / count / simulation_steps;
      pm.position += correction;
    }
  }
}

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.
  float w = 3.0 * width / num_width_points;
  float h = 3.0 * height / num_height_points;
  float t = max(w, h);

  //Vector3D nearestBox((pos.x - fmod(pos.x, w)) / w, (pos.y - fmod(pos.y, h)) / h, (pos.z - fmod(pos.z, t)) / t);
  Vector3D nearestBox(pos.x / w, pos.y / h, pos.z / t);
  int id = (int)nearestBox.x * 100 + (int)nearestBox.y * 10 + (int)nearestBox.z;
  return id; 
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

void Cloth::buildClothMesh() {
  if (point_masses.size() == 0) return;

  ClothMesh *clothMesh = new ClothMesh();
  vector<Triangle *> triangles;

  // Create vector of triangles
  for (int y = 0; y < num_height_points - 1; y++) {
    for (int x = 0; x < num_width_points - 1; x++) {
      PointMass *pm = &point_masses[y * num_width_points + x];
      // Get neighboring point masses:
      /*                      *
       * pm_A -------- pm_B   *
       *             /        *
       *  |         /   |     *
       *  |        /    |     *
       *  |       /     |     *
       *  |      /      |     *
       *  |     /       |     *
       *  |    /        |     *
       *      /               *
       * pm_C -------- pm_D   *
       *                      *
       */
      
      float u_min = x;
      u_min /= num_width_points - 1;
      float u_max = x + 1;
      u_max /= num_width_points - 1;
      float v_min = y;
      v_min /= num_height_points - 1;
      float v_max = y + 1;
      v_max /= num_height_points - 1;
      
      PointMass *pm_A = pm                       ;
      PointMass *pm_B = pm                    + 1;
      PointMass *pm_C = pm + num_width_points    ;
      PointMass *pm_D = pm + num_width_points + 1;
      
      Vector3D uv_A = Vector3D(u_min, v_min, 0);
      Vector3D uv_B = Vector3D(u_max, v_min, 0);
      Vector3D uv_C = Vector3D(u_min, v_max, 0);
      Vector3D uv_D = Vector3D(u_max, v_max, 0);
      
      
      // Both triangles defined by vertices in counter-clockwise orientation
      triangles.push_back(new Triangle(pm_A, pm_C, pm_B, 
                                       uv_A, uv_C, uv_B));
      triangles.push_back(new Triangle(pm_B, pm_C, pm_D, 
                                       uv_B, uv_C, uv_D));
    }
  }

  // For each triangle in row-order, create 3 edges and 3 internal halfedges
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    // Allocate new halfedges on heap
    Halfedge *h1 = new Halfedge();
    Halfedge *h2 = new Halfedge();
    Halfedge *h3 = new Halfedge();

    // Allocate new edges on heap
    Edge *e1 = new Edge();
    Edge *e2 = new Edge();
    Edge *e3 = new Edge();

    // Assign a halfedge pointer to the triangle
    t->halfedge = h1;

    // Assign halfedge pointers to point masses
    t->pm1->halfedge = h1;
    t->pm2->halfedge = h2;
    t->pm3->halfedge = h3;

    // Update all halfedge pointers
    h1->edge = e1;
    h1->next = h2;
    h1->pm = t->pm1;
    h1->triangle = t;

    h2->edge = e2;
    h2->next = h3;
    h2->pm = t->pm2;
    h2->triangle = t;

    h3->edge = e3;
    h3->next = h1;
    h3->pm = t->pm3;
    h3->triangle = t;
  }

  // Go back through the cloth mesh and link triangles together using halfedge
  // twin pointers

  // Convenient variables for math
  int num_height_tris = (num_height_points - 1) * 2;
  int num_width_tris = (num_width_points - 1) * 2;

  bool topLeft = true;
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    if (topLeft) {
      // Get left triangle, if it exists
      if (i % num_width_tris != 0) { // Not a left-most triangle
        Triangle *temp = triangles[i - 1];
        t->pm1->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm1->halfedge->twin = nullptr;
      }

      // Get triangle above, if it exists
      if (i >= num_width_tris) { // Not a top-most triangle
        Triangle *temp = triangles[i - num_width_tris + 1];
        t->pm3->halfedge->twin = temp->pm2->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle to bottom right; guaranteed to exist
      Triangle *temp = triangles[i + 1];
      t->pm2->halfedge->twin = temp->pm1->halfedge;
    } else {
      // Get right triangle, if it exists
      if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
        Triangle *temp = triangles[i + 1];
        t->pm3->halfedge->twin = temp->pm1->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle below, if it exists
      if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
        Triangle *temp = triangles[i + num_width_tris - 1];
        t->pm2->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm2->halfedge->twin = nullptr;
      }

      // Get triangle to top left; guaranteed to exist
      Triangle *temp = triangles[i - 1];
      t->pm1->halfedge->twin = temp->pm2->halfedge;
    }

    topLeft = !topLeft;
  }

  clothMesh->triangles = triangles;
  this->clothMesh = clothMesh;
}
