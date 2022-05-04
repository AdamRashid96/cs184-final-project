#ifndef GAS_FIELD_H
#define GAS_FIELD_H

#include <vector>

#include "CGL/CGL.h"
#include "fieldCell.h"
#include "util/random_util.h"

using namespace CGL;
using namespace std;

class GasField {
public:
  GasField(int width, int height, int depth, double cell_size, double density,
           double ambient_temperature, double base_pressure, double initial_velocity) {
    this->width = width;
    this->height = height;
    this->depth = depth;
    this->cell_size = cell_size;
    this->density = density;
    this->size = (width + 2) * (height + 2) * (depth + 2);

    div = new double[size];
    p = new double[size];

    cells = new FieldCell[size];
    prev_cells = new FieldCell[size];

    // Initialize cell values
    for (int i = 0; i < width + 2; i++) {
      for (int j = 0; j < height + 2; j++) {
        for (int k = 0; k < depth + 2; k++) {
          FieldCell cell;
          if (i == 0 || i == width + 1 || j == 0 || j == height + 1 || k == 0 || k == depth + 1) {
              cell.isBoundary = true;
          } else {
              cell.isBoundary = false;
          }
          cell.temperature = ambient_temperature;
          cell.pressure = base_pressure;
          cell.phi = 0;
          cell.velocity = get_sample() * random_uniform() * initial_velocity;

          // Write to both arrays
          *CellAt(cells, i, j, k) = cell;
          *CellAt(prev_cells, i, j, k) = cell;
        }
      }
    }
  }

  ~GasField() {
    delete [] cells;
    delete [] prev_cells;
    delete [] div;
    delete [] p;
  }

  void apply_force(double delta_time) {
    for (int i = 1; i <= width; i++) {
      for (int j = 1; j <= height; j++) {
        for (int k = 1; k <= depth; k++) {
          CellAt(i, j, k)->velocity += CellAt(i, j, k)->force / density * delta_time;
        }
      }
    }
  }

  // Unused for now
  void diffuse_vel(double diff, double delta_time) {
    swap();

    double a = delta_time * diff * width * height * depth;

    for (int t = 0; t < diffuse_relax_steps; t++) {
      for (int i = 1; i <= width; i++) {
        for (int j = 1; j <= height; j++) {
          for (int k = 1; k <= depth; k++) {
            FieldCell* up = CellAt(prev_cells, i, j + 1, k);
            FieldCell* down = CellAt(prev_cells, i, j - 1, k);
            FieldCell* left = CellAt(prev_cells, i - 1, j, k);
            FieldCell* right = CellAt(prev_cells, i + 1, j, k);
            FieldCell* in = CellAt(prev_cells, i, j, k - 1);
            FieldCell* out = CellAt(prev_cells, i, j, k + 1);

            FieldCell* assignment = CellAt(cells, i, j, k);
            *assignment = *CellAt(cells, i, j, k);
            assignment->velocity = (assignment->velocity + a * (up->velocity + down->velocity + left->velocity + right->velocity + in->velocity + out->velocity)) / (1 + 6 * a);
            
            
            // (CellAt(prev_cells, i, j, k)->velocity + a * (CellAt(cells, i - 1, j, k)->velocity +
            // CellAt(cells, i + 1, j, k)->velocity + CellAt(cells, i, j - 1, k)->velocity + CellAt(cells, i, j + 1, k)->velocity + 
            // CellAt(cells, i, j, k - 1)->velocity + CellAt(cells, i, j, k + 1)->velocity)) / (1 + 6 * a);
          }
        }
      }
      set_bnd(cells);
    }
  }

  void advect(double delta_time) {
    swap();

    double dt0 = delta_time / cell_size;

    for (int i = 1; i <= width; i++) {
      for (int j = 1; j <= height; j++) {
        for (int k = 1; k <= depth; k++) {
          FieldCell* prev_cell = CellAt(prev_cells, i, j, k);

          // Perform the back trace in field space
          double x = i - dt0 * prev_cell->velocity.x;
          double y = j - dt0 * prev_cell->velocity.y;
          double z = k - dt0 * prev_cell->velocity.z;

          // There is a difference in the way we clamp
          if (x < 0.5) {
              x = 0.5;
          }
          if (x > width + 0.5) {
            x = width + 0.5;
          }
          if (y < 0.5) {
              y = 0.5;
          }
          if (y > height + 0.5) {
            y = height + 0.5;
          }
          if (z < 0.5) {
              z = 0.5;
          }
          if (z > depth + 0.5) {
            z = depth + 0.5;
          }

          int left = static_cast<int>(x);
          int right = static_cast<int>(x) + 1;
          int lower = static_cast<int>(y);
          int upper = static_cast<int>(y) + 1;
          int front = static_cast<int>(z);
          int back = static_cast<int>(z) + 1;

          // Left or Right
          // Upper or lower
          // Front or Back
          FieldCell* luf = CellAt(prev_cells, left, upper, front);
          FieldCell* ruf = CellAt(prev_cells, right, upper, front);
          FieldCell* llf = CellAt(prev_cells, left, lower, front);
          FieldCell* rlf = CellAt(prev_cells, right, lower, front);
          FieldCell* lub = CellAt(prev_cells, left, upper, back);
          FieldCell* rub = CellAt(prev_cells, right, upper, back);
          FieldCell* llb = CellAt(prev_cells, left, lower, back);
          FieldCell* rlb = CellAt(prev_cells, right, lower, back);

          double horizontal = x - left;
          double vertical = y - lower;
          double forward = z - front;

          FieldCell left_upper = InterpCell(*luf, *lub, forward);
          FieldCell right_upper = InterpCell(*ruf, *rub, forward);
          FieldCell left_lower = InterpCell(*llf, *llb, forward);
          FieldCell right_lower = InterpCell(*rlf, *rlb, forward);

          FieldCell upper_cell = InterpCell(left_upper, right_upper, horizontal);
          FieldCell lower_cell = InterpCell(left_lower, right_lower, horizontal);

          FieldCell interpolatedCell = InterpCell(lower_cell, upper_cell, vertical);




          //FieldCell interpolatedCell = CellAtInterpolatedFieldSpace(prev_cells, x, y, z);
          FieldCell* cell = CellAt(cells, i, j, k);
          *cell = *prev_cell;
          cell->velocity = interpolatedCell.velocity;
        }
      }
    }
    set_bnd(cells);
  }

  /* Calculates the divergence of the velocity */
  void calculate_divergence(FieldCell* cell_array) {
    for (int i = 1; i <= width; i++) {
      for (int j = 1; j <= height; j++) {
        for (int k = 1; k <= depth; k++) {
          FieldCell* up = CellAt(cell_array, i, j + 1, k);
          FieldCell* down = CellAt(cell_array, i, j - 1, k);
          FieldCell* left = CellAt(cell_array, i - 1, j, k);
          FieldCell* right = CellAt(cell_array, i + 1, j, k);
          FieldCell* in = CellAt(cell_array, i, j, k - 1);
          FieldCell* out = CellAt(cell_array, i, j, k + 1);

          div[i + j * width + k * height * width] = 0.5 * ((right->velocity.x - left->velocity.x) / cell_size + 
                                                           (up->velocity.y - down->velocity.y) / cell_size +
                                                           (out->velocity.z - in->velocity.z) / cell_size);
        }
      }
    }
  }

  void pressure_step(double delta_time) {
    // Swap
    swap();

    // Calculate divergence
    calculate_divergence(prev_cells);

    double h3 = cell_size * cell_size * cell_size;

    for (int t = 0; t < pressure_relax_steps; t++) {
      for (int i = 1; i <= width; i++) {
        for (int j = 1; j <= height; j++) {
          for (int k = 1; k <= depth; k++) {
            FieldCell* center = CellAt(prev_cells, i, j, k);
            FieldCell* up = CellAt(prev_cells, i, j + 1, k);
            FieldCell* down = CellAt(prev_cells, i, j - 1, k);
            FieldCell* left = CellAt(prev_cells, i - 1, j, k);
            FieldCell* right = CellAt(prev_cells, i + 1, j, k);
            FieldCell* in = CellAt(prev_cells, i, j, k - 1);
            FieldCell* out = CellAt(prev_cells, i, j, k + 1);

            double f = density / delta_time * (div[i + j * width + k * height * width] - center->phi);

            FieldCell* assignment = CellAt(cells, i, j, k);
            *assignment = *center;
            assignment->pressure = 1 / 6 * (up->pressure + down->pressure + left->pressure + right->pressure + in->pressure + out->pressure - h3 * f); 
          }
        }
      }
    }
  }

  void vel_pressure_step() {
    swap();

    for (int i = 1; i <= width; i++) {
      for (int j = 1; j <= height; j++) {
        for (int k = 1; k <= depth; k++) {
          FieldCell* center = CellAt(prev_cells, i, j, k);
          FieldCell* up = CellAt(prev_cells, i, j + 1, k);
          FieldCell* down = CellAt(prev_cells, i, j - 1, k);
          FieldCell* left = CellAt(prev_cells, i - 1, j, k);
          FieldCell* right = CellAt(prev_cells, i + 1, j, k);
          FieldCell* in = CellAt(prev_cells, i, j, k - 1);
          FieldCell* out = CellAt(prev_cells, i, j, k + 1);

          FieldCell* assignment = CellAt(cells, i, j, k);
          *assignment = *center;
          assignment->velocity.x -= (right->pressure - left->pressure) / (2 * cell_size);
          assignment->velocity.y -= (up->pressure - down->pressure) / (2 * cell_size);
          assignment->velocity.z -= (out->pressure - in->pressure) / (2 * cell_size);
        }
      }
    }
  }

  void project() {  // int N, float * u, float * v, float * p, float * div project ( N, u, v, u0, v0 );
    for (int i = 0; i < size; i++) {
      div[i] = 0;
      p[i] = 0;
    }

    int i, j, k;
    double hx = cell_size;
    double hy = cell_size;
    double hz = cell_size;
    for ( i=1 ; i<=width ; i++ ) {
      for ( j=1 ; j<=height ; j++ ) {
        for ( k=1 ; k<=depth ; k++ ) {
          div[IDX(i, j, k)] = -0.5 * ( hx * (CellAt(cells, i+1, j, k)->velocity[0] - CellAt(cells, i-1, j, k)->velocity[0]) +
                                          hy * (CellAt(cells, i, j+1, k)->velocity[1] - CellAt(cells, i, j - 1, k)->velocity[1]) + hz * (CellAt(cells, i, j, k+1)->velocity[2] - CellAt(cells, i, j, k-1)->velocity[2]));
          p[IDX(i, j, k)] = 0;
        }
      }
    }
    set_bnd_vec(div);
    for (int t = 0; t < 20; t++) {
      for ( i=1 ; i<=width ; i++ ) {
        for ( j=1 ; j<=height ; j++ ) {
          for (k = 1; k <= depth; k++) {
            p[IDX(i, j, k)] =
                (div[IDX(i, j, k)] + p[IDX(i - 1, j, k)] + p[IDX(i + 1, j, k)] +
                p[IDX(i, j - 1, k)] + p[IDX(i, j + 1, k)] + p[IDX(i, j, k - 1)] + p[IDX(i, j, k + 1)]) / 6;
          }
        }
      }
      set_bnd_vec(p);
    }
    for ( i=1 ; i<=width ; i++ ) {
      for ( j=1 ; j<=height ; j++ ) {
        for (k = 1; k <= depth; k++) {
          CellAt(cells, i, j, k)->velocity[0] -= 0.5 * (p[IDX(i + 1, j, k)] - p[IDX(i - 1, j, k)]) / hx;
          CellAt(cells, i, j, k)->velocity[1] -= 0.5 * (p[IDX(i, j + 1, k)] - p[IDX(i, j - 1, k)]) / hy;
          CellAt(cells, i, j, k)->velocity[2] -= 0.5 * (p[IDX(i, j, k + 1)] - p[IDX(i, j, k - 1)]) / hz;
        }
      }
    }
    set_bnd(cells);
  }

  int IDX(int x, int y, int z) {
    return x + y * (width + 2) + z * (height + 2) * (width + 2);
  }

  void set_bnd_vec(double *p){
    // In and out bounds
    for (int i = 1; i <= width; i++) {
      for (int j = 1; j <= height; j++) {
        p[IDX(i, j, 0)] = p[IDX(i, j, 1)];
        p[IDX(i, j, depth + 1)] = p[IDX(i, j, depth)];
      }
    }

    // Top and bottom bounds
    for (int i = 1; i <= width; i++) {
      for (int k = 1; k <= depth; k++) {
        p[IDX(i, 0, k)] = p[IDX(i, 1, k)];
        p[IDX(i, height + 1, k)] = p[IDX(i, height + 1, k)];
      }
    }

    // Left and right bounds
    for (int j = 1; j <= height; j++) {
      for (int k = 1; k <= depth; k++) {
        p[IDX(0, j, k)] = p[IDX(1, j, k)];
        p[IDX(width + 1, j, k)] = p[IDX(width, j, k)];
      }
    }

    // Horizontal edges
    for (int i = 1; i <= width; i++) {
        p[IDX(i, 0, 0)] = (p[IDX(i, 1, 0)] + p[IDX(i, 0, 1)]) / 2;
        p[IDX(i, height+1, 0)] = (p[IDX(i, height, 0)] + p[IDX(i, height + 1, 1)]) / 2;
        p[IDX(i, 0, depth + 1)] = (p[IDX(i, 0, depth)] + p[IDX(i, 1, depth + 1)]) / 2;
        p[IDX(i, height+1, depth+1)] = (p[IDX(i, height, depth+1)] + p[IDX(i, height+1, depth)]) / 2;
    }

    // Vertical edges
    for (int j = 1; j <= height; j++) {
      p[IDX(0, j, 0)] = (p[IDX(1, j, 0)] + p[IDX(0, j, 1)]) / 2;
      p[IDX(width + 1, j, 0)] = (p[IDX(width, j, 0)] + p[IDX(width + 1, j, 1)]) / 2;
      p[IDX(0, j, depth + 1)] = (p[IDX(1, j, depth + 1)] + p[IDX(0, j, depth)]) / 2;
      p[IDX(width + 1, j, depth + 1)] = (p[IDX(width, j, depth + 1)] + p[IDX(width + 1, j, depth)]) / 2;
    }

    // Outward edges
    for (int k = 1; k <= depth; k++) {
      p[IDX(0, 0, k)] = (p[IDX(1, 0, k)] + p[IDX(0, 1, k)]) / 2;
      p[IDX(0, height + 1, k)] = (p[IDX(1, height + 1, k)] + p[IDX(0, height, k)]) / 2;
      p[IDX(width + 1, 0, k)] = (p[IDX(width, 0, k)]  + p[IDX(width + 1, 1, k)]) / 2;
      p[IDX(width + 1, height + 1, k)] = (p[IDX(width, height + 1, k)] + p[IDX(width + 1, height, k)]) / 2;
    }

    // The corners
    p[IDX(0, 0, 0)] = (p[IDX(1, 0, 0)] + p[IDX(0, 1, 0)] + p[IDX(0, 0, 1)]) / 3.0;
    p[IDX(width + 1, 0, 0)] = (p[IDX(width, 0, 0)] + p[IDX(width + 1, 1, 0)] + p[IDX(width + 1, 0, 1)]) / 3.0;
    p[IDX(0, height + 1, 0)] = (p[IDX(1, height + 1, 0)] + p[IDX(0, height, 0)] + p[IDX(0, height + 1, 1)]) / 3.0;
    p[IDX(0, 0, depth + 1)] = (p[IDX(1, 0, depth + 1)] + p[IDX(0, 1, depth + 1)] + p[IDX(0, 0, depth )]) / 3.0;
    p[IDX(width + 1, height+1, 0)] = (p[IDX(width, height+1, 0)] + p[IDX(width + 1, height, 0)] + p[IDX(width + 1, height+1, 1)]) / 3.0;
    p[IDX(width + 1, 0, depth + 1)] = (p[IDX(width, 0, depth + 1)] + p[IDX(width + 1, 1, depth + 1)] + p[IDX(width + 1, 0, depth)]) / 3.0;
    p[IDX(0, height + 1, depth + 1)] = (p[IDX(1, height + 1, depth + 1)] + p[IDX(0, height, depth + 1)] + p[IDX(0, height + 1, depth)]) / 3.0;
    p[IDX(width + 1, height + 1, depth + 1)] = (p[IDX(width, height + 1, depth + 1)] + p[IDX(width + 1, height, depth + 1)] + p[IDX(width + 1, height + 1, depth )]) / 3.0;
  }

  void set_bnd(FieldCell* cells) {
    // In and out bounds
    for (int i = 1; i <= width; i++) {
      for (int j = 1; j <= height; j++) {
        Vector3D inVelocity = CellAt(cells, i, j, 1)->velocity;
        CellAt(cells, i, j, 0)->velocity = Vector3D(inVelocity.x, inVelocity.y, -inVelocity.z);

        Vector3D outVelocity = CellAt(cells, i, j, depth)->velocity;
        CellAt(cells, i, j, depth + 1)->velocity = Vector3D(outVelocity.x, outVelocity.y, -outVelocity.z);
      }
    }

    // Top and bottom bounds
    for (int i = 1; i <= width; i++) {
      for (int k = 1; k <= depth; k++) {
        Vector3D bottomVelocity = CellAt(cells, i, 1, k)->velocity;
        CellAt(cells, i, 0, k)->velocity = Vector3D(bottomVelocity.x, -bottomVelocity.y, bottomVelocity.z);

        Vector3D topVelocity = CellAt(cells, i, height, k)->velocity;
        CellAt(cells, i, height + 1, k)->velocity = Vector3D(topVelocity.x, -topVelocity.y, topVelocity.z);
      }
    }

    // Left and right bounds
    for (int j = 1; j <= height; j++) {
      for (int k = 1; k <= depth; k++) {
        Vector3D leftVelocity = CellAt(cells, 1, j, k)->velocity;
        CellAt(cells, 0, j, k)->velocity = Vector3D(-leftVelocity.x, leftVelocity.y, leftVelocity.z);

        Vector3D rightVelocity = CellAt(cells, width, j, k)->velocity;
        CellAt(cells, width + 1, j, k)->velocity = Vector3D(-rightVelocity.x, rightVelocity.y, rightVelocity.z);
      }
    }

    // Horizontal edges
    for (int i = 1; i <= width; i++) {
      CellAt(cells, i, 0, 0)->velocity = (CellAt(cells, i, 1, 0)->velocity + CellAt(cells, i, 0, 1)->velocity) / 2;
      CellAt(cells, i, height+1, 0)->velocity = (CellAt(cells, i, height, 0)->velocity + CellAt(cells, i, height+1, 1)->velocity) / 2;
      CellAt(cells, i, 0, depth+1)->velocity = (CellAt(cells, i, 0, depth)->velocity + CellAt(cells, i, 1, depth+1)->velocity) / 2;
      CellAt(cells, i, height+1, depth+1)->velocity = (CellAt(cells, i, height, depth+1)->velocity + CellAt(cells, i, height+1, depth)->velocity) / 2;
    }

    // Vertical edges
    for (int j = 1; j <= height; j++) {
      CellAt(cells, 0, j, 0)->velocity = (CellAt(cells, 1, j, 0)->velocity + CellAt(cells, 0, j, 1)->velocity) / 2;
      CellAt(cells, width+1, j, 0)->velocity = (CellAt(cells, width, j, 0)->velocity + CellAt(cells, width+1, j, 1)->velocity) / 2;
      CellAt(cells, 0, j, depth+1)->velocity = (CellAt(cells, 1, j, depth+1)->velocity + CellAt(cells, 0, j, depth)->velocity) / 2;
      CellAt(cells, width+1, j, depth+1)->velocity = (CellAt(cells, width, j, depth+1)->velocity + CellAt(cells, width+1, j, depth)->velocity) / 2;
    }

    // Outward edges
    for (int k = 1; k <= depth; k++) {
      CellAt(cells, 0, 0, k)->velocity = (CellAt(cells, 1, 0, k)->velocity + CellAt(cells, 0, 1, k)->velocity) / 2;
      CellAt(cells, 0, height+1, k)->velocity = (CellAt(cells, 1, height+1, k)->velocity + CellAt(cells, 0, height, k)->velocity) / 2;
      CellAt(cells, width+1, 0, k)->velocity = (CellAt(cells, width, 0, k)->velocity + CellAt(cells, width+1, 1, k)->velocity) / 2;
      CellAt(cells, width+1, height+1, k)->velocity = (CellAt(cells, width, height+1, k)->velocity + CellAt(cells, width+1, height, k)->velocity) / 2;
    }

    // The corners
    CellAt(cells, 0, 0, 0)->velocity = (CellAt(cells, 1, 0, 0)->velocity + CellAt(cells, 0, 1, 0)->velocity + CellAt(cells, 0, 0, 1)->velocity) / 3.0;
    CellAt(cells, width+1, 0, 0)->velocity = (CellAt(cells, width, 0, 0)->velocity + CellAt(cells, width+1, 1, 0)->velocity + CellAt(cells, width+1, 0, 1)->velocity) / 3.0;
    CellAt(cells, 0, height+1, 0)->velocity = (CellAt(cells, 1, height+1, 0)->velocity + CellAt(cells, 0, height, 0)->velocity + CellAt(cells, 0, height+1, 1)->velocity) / 3.0;
    CellAt(cells, 0, 0, depth+1)->velocity = (CellAt(cells, 1, 0, depth+1)->velocity + CellAt(cells, 0, 1, depth+1)->velocity + CellAt(cells, 0, 0, depth)->velocity) / 3.0;
    CellAt(cells, width+1, height+1, 0)->velocity = (CellAt(cells, width, height+1, 0)->velocity + CellAt(cells, width+1, height, 0)->velocity + CellAt(cells, width+1, height+1, 1)->velocity) / 3.0;
    CellAt(cells, width+1, 0, depth+1)->velocity = (CellAt(cells, width, 0, depth+1)->velocity + CellAt(cells, width+1, 1, depth+1)->velocity + CellAt(cells, width+1, 0, depth)->velocity) / 3.0;
    CellAt(cells, 0, height+1, depth+1)->velocity = (CellAt(cells, 1, height+1, depth+1)->velocity + CellAt(cells, 0, height, depth+1)->velocity + CellAt(cells, 0, height+1, depth)->velocity) / 3.0;
    CellAt(cells, width+1, height+1, depth+1)->velocity = (CellAt(cells, width, height+1, depth+1)->velocity + CellAt(cells, width+1, height, depth+1)->velocity + CellAt(cells, width+1, height+1, depth)->velocity) / 3.0;
  }

  /* Return the cell at the given indices. */
  FieldCell* CellAt(FieldCell* cell_array, int x, int y, int z) {
    return &cell_array[x + y * (width + 2) + z * (height + 2) * (width + 2)];
  }

  /* Return the cell at the given indices. */
  FieldCell* CellAt(int x, int y, int z) {
    return &cells[x + y * (width + 2) + z * (height + 2) * (width + 2)];
  }

  /* Return the closest cell to the given world position. */
  FieldCell* CellAt(Vector3D position) {
    int x = (position.x - origin.x) / cell_size + 0.5 * (width + 2);
    int y = (position.y - origin.y) / cell_size + 0.5 * (height + 2);
    int z = (position.z - origin.z) / cell_size + 0.5 * (depth + 2);

    if (x < 0 || x >= width + 2 || y < 0 || y >= height + 2 || z < 0 || z >= depth + 2) {
      return NULL;
    }

    return CellAt(x, y, z);
  }

  /* Interpolate between two cells. */
  FieldCell InterpCell(FieldCell a, FieldCell b, double t) {
    FieldCell result;
    result.temperature = a.temperature + t * b.temperature  + (-t) * a.temperature;
    result.velocity = a.velocity + t * b.velocity  + (-t) * a.velocity;
    return result;
  }

  /* Return a cell with interpolated values for the given field space position. */
  FieldCell CellAtInterpolatedFieldSpace(FieldCell* cell_array, double x, double y, double z) {
    int left = clamp(static_cast<int>(x - 0.5), 0, width + 1);
    int right = clamp(static_cast<int>(x + 0.5), 0, width + 1);
    int lower = clamp(static_cast<int>(y - 0.5), 0, height + 1);
    int upper = clamp(static_cast<int>(y + 0.5), 0, height + 1);
    int front = clamp(static_cast<int>(z - 0.5), 0, depth + 1);
    int back = clamp(static_cast<int>(z + 0.5), 0, depth + 1);

    // Left or Right
    // Upper or lower
    // Front or Back
    FieldCell* luf = CellAt(cell_array, left, upper, front);
    FieldCell* ruf = CellAt(cell_array, right, upper, front);
    FieldCell* llf = CellAt(cell_array, left, lower, front);
    FieldCell* rlf = CellAt(cell_array, right, lower, front);
    FieldCell* lub = CellAt(cell_array, left, upper, back);
    FieldCell* rub = CellAt(cell_array, right, upper, back);
    FieldCell* llb = CellAt(cell_array, left, lower, back);
    FieldCell* rlb = CellAt(cell_array, right, lower, back);

    double horizontal = x - static_cast<int>(x);
    double vertical = y - static_cast<int>(y);
    double forward = z - static_cast<int>(z);

    FieldCell left_upper = InterpCell(*luf, *lub, forward);
    FieldCell right_upper = InterpCell(*ruf, *rub, forward);
    FieldCell left_lower = InterpCell(*llf, *llb, forward);
    FieldCell right_lower = InterpCell(*rlf, *rlb, forward);

    FieldCell upper_cell = InterpCell(left_upper, right_upper, horizontal);
    FieldCell lower_cell = InterpCell(left_lower, right_lower, horizontal);

    return InterpCell(lower_cell, upper_cell, vertical);
  }

  /* Return a cell with interpolated values for the given world space position. */
  FieldCell CellAtInterpolated(Vector3D position) {
    double x = (position.x - origin.x) / cell_size + 0.5 * (width + 2);
    double y = (position.y - origin.y) / cell_size + 0.5 * (height + 2);
    double z = (position.z - origin.z) / cell_size + 0.5 * (depth + 2);

    return CellAtInterpolatedFieldSpace(cells, x, y, z);
  }

  /* Swap the previous and current cell values. */
  void swap() {
    FieldCell *temp = cells;
    cells = prev_cells;
    prev_cells = temp;
  }

  /* Return the world space position of the cell at the given indices. */
  Vector3D CellPos(int x, int y, int z) {
    return origin - cell_size / 2 * Vector3D(width + 2, height + 2, depth + 2) + cell_size * Vector3D(x, y, z);
  }

  Vector3D origin; // Center of the grid field
  int width, height, depth;
  int size;
  double cell_size;
  double density;

  FieldCell *cells;
  FieldCell *prev_cells;

  int pressure_relax_steps = 20;
  int diffuse_relax_steps = 20;

  double *div;
  double *p;

}; // class GasField

#endif // GAS_FIELD_H
