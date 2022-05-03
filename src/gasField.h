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
    GasField(int width, int height, int depth, double cell_size,
                double ambient_temperature, double base_pressure, double initial_velocity) {
        this->width = width;
        this->height = height;
        this->depth = depth;
        this->cell_size = cell_size;
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
                    cell.divergence = 0;
                    cell.velocity = get_sample() * random_uniform() * initial_velocity;

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
    
    void add_vel(Vector3D *s, double dt) {
        int i;
        for (i=0 ; i<size ; i++) { 
            cells[i].velocity += dt*s[i];
        }
    }

    void diffuse_vel(double diff, double dt) {
        int i, j, k, t;
        double a=dt*diff*width*height*depth;

        for (t = 0; t < 20; t++) {
            for (i = 1; i <= width; i++) {
                for (j = 1; j <= height; j++) {
                    for (k = 1; k <= depth; k++) {
                        CellAt(cells, i, j, k)->velocity = (CellAt(prev_cells, i, j, k)->velocity + a * (CellAt(cells, i-1, j, k)->velocity +
                                                            CellAt(cells, i+1, j, k)->velocity + CellAt(cells, i, j-1, k)->velocity + CellAt(cells, i, j+1, k)->velocity + 
                                                            CellAt(cells, i, j, k-1)->velocity + CellAt(cells, i, j, k+1)->velocity)) / (1 + 6*a);
                    }
                }
            }
            set_bnd(cells);
        }
    }

    void advect(double dt)  // int N, int b, float * d, float * d0, float * u, float * v, float dt
    {
      int i, j, k, i0, j0, k0, i1, j1, k1;
      double x, y, z, s0, t0, u0, s1, t1, u1, dtx0, dty0, dtz0;
      dtx0 = dt*width;
      dty0 = dt*height;
      dtz0 = dt*depth;

      for (int dim = 0; dim < 3; dim++) {
        for (i = 1; i <= width; i++) {
          for (j = 1; j <= height; j++) {
            for (k = 1; k <= depth; k++) {

              x = i - dtx0 * CellAt(prev_cells, i, j, k)->velocity[0];
              y = j - dty0 * CellAt(prev_cells, i, j, k)->velocity[1];
              z = k - dtz0 * CellAt(prev_cells, i, j, k)->velocity[2];

              if (x < 0.5) x = 0.5;
              if (x > width + 0.5) x = width + 0.5;
              i0 = (int) x;
              i1 = i0 + 1;
              if (y < 0.5) y = 0.5;
              if (y > height + 0.5) y = height + 0.5;
              j0 = (int) y;
              j1 = j0 + 1;
              if (z < 0.5) z = 0.5;
              if (z > depth + 0.5) z = depth + 0.5;
              k0 = (int) k;
              k1 = k0 + 1;

              s1 = x - i0;
              s0 = 1 - s1;
              t1 = y - j0;
              t0 = 1 - t1;
              u1 = z - k0;
              u0 = 1 - u1;

              double s0t0 = u0 * CellAt(prev_cells, i0, j0, k0)->velocity[dim] + u1 * CellAt(prev_cells, i0, j0, k1)->velocity[dim];
              double s0t1 = u0 * CellAt(prev_cells, i0, j1, k0)->velocity[dim] + u1 * CellAt(prev_cells, i0, j1, k1)->velocity[dim];
              double s1t0 = u0 * CellAt(prev_cells, i1, j0, k0)->velocity[dim] + u1 * CellAt(prev_cells, i1, j0, k1)->velocity[dim];
              double s1t1 = u0 * CellAt(prev_cells, i1, j1, k0)->velocity[dim] + u1 * CellAt(prev_cells, i1, j1, k1)->velocity[dim];
              CellAt(cells, i, j, k)->velocity[dim] = s0 * (t0 * s0t0 + t1 * s0t1)+ s1 * (t0 * s1t0 + t1 * s1t1);
//              CellAt(cells, i, j, k)->velocity[dim] = s0 * (t0 * d0[IDX(i0, j0)] + t1 * d0[IDX(i0, j1)])+ s1 * (t0 * d0[IDX(i1, j0)] + t1 * d0[IDX(i1, j1)]);
            }
          }
        }
        set_bnd(cells);
      }

    }

    void project() {  // int N, float * u, float * v, float * p, float * div project ( N, u, v, u0, v0 );
        for (int i = 0; i < size; i++) {
            div[i] = 0;
            p[i] = 0;
        }

        int i, j, k;
        double hx = 1.0/width;
        double hy = 1.0/height;
        double hz = 1.0/depth;
        for ( i=1 ; i<=width ; i++ ) {
            for ( j=1 ; j<=height ; j++ ) {
                for ( k=1 ; k<=depth ; k++ ) {
                div[i + j * width + k * height * width] = -0.5 * ( hx * (CellAt(cells, i+1, j, k)->velocity[0] - CellAt(cells, i-1, j, k)->velocity[0]) +
                                                hy * (CellAt(cells, i, j+1, k)->velocity[1] - CellAt(cells, i, j - 1, k)->velocity[1]) + hz * (CellAt(cells, i, j, k+1)->velocity[2] - CellAt(cells, i, j, k-1)->velocity[2]));
                p[i + j * width + k * height * width] = 0;
                }
            }
        }
        set_bnd_vec (div);
         for (int t = 0; t < 20; t++) {
            for ( i=1 ; i<=width ; i++ ) {
                for ( j=1 ; j<=height ; j++ ) {
                    for (k = 1; k <= depth; k++) {
                        p[i + j * width + k * height * width] =
                            (div[i + j * width + k * height * width] + p[(i - 1) + j * width + k * height * width] +
                            p[(i + 1) + j * width + k * height * width] +
                            p[i + (j - 1) * width + k * height * width] + p[i + (j + 1) * width + k * height * width] +
                            p[i + j * width + (k - 1) * height * width] + p[i + (j + 1) * width + (k + 1) * height * width]) / 6;
                    }
                }
            }
            set_bnd_vec (p);
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
        set_bnd ( cells );
    }

    int IDX(int x, int y, int z) {
      return x + y * width + z * height * width;
    }

  void set_bnd_vec (double *p){
    int i, j, k;
    for (i=1 ; i<=width; i++) {
      p[IDX(i, 0, 0)] = p[IDX(i, 1, 1)];
      p[IDX(i, height+1, 0)] = p[IDX(i, height, 1)];
      p[IDX(i, 0, depth+1)] = p[IDX(i, 1, depth)];
      p[IDX(i, height+1, depth+1)] = p[IDX(i, height, depth)];
    }

    for (j=1 ; j<=height; j++) {
      p[IDX(0, j, 0)] = p[IDX(1, j, 1)];
      p[IDX(width+1, j, 0)] = p[IDX(width, j, 1)];
      p[IDX(0, j, depth+1)] = p[IDX(1, j, depth)];
      p[IDX(width+1, j, depth+1)] = p[IDX(width, j, depth)];
    }

    for (k=1 ; k<=depth; k++) {
      p[IDX(0, 0, k)] = p[IDX(1, 1, k)];
      p[IDX(0, height+1, k)] = p[IDX(i, height, k)];
      p[IDX(width+1, 0, k)] = p[IDX(width, 1, k)];
      p[IDX(width+1, height+1, k)] = p[IDX(width, height, k)];
    }


    p[IDX(0, 0, 0)] = (p[IDX(1, 0, 0)] + p[IDX(0, 1, 0)] + p[IDX(0, 0, 1)]) / 3.0;
    p[IDX(width+1, 0, 0)] = (p[IDX(width, 0, 0)] + p[IDX(width+1, 1, 0)] + p[IDX(width+1, 0, 1)]) / 3.0;
    p[IDX(0, height+1, 0)] = (p[IDX(1, height+1, 0)] + p[IDX(0, height, 0)] + p[IDX(0, height+1, 1)]) / 3.0;
    p[IDX(0, 0, depth+1)] = (p[IDX(1, 0, depth+1)] + p[IDX(0, 1, depth+1)] + p[IDX(0, 0, depth)]) / 3.0;
    p[IDX(width+1, height+1, 0)] = (p[IDX(width, height+1, 0)] + p[IDX(width+1, height, 0)] + p[IDX(width+1, height+1, 1)]) / 3.0;
    p[IDX(width+1, 0, depth+1)] = (p[IDX(width, 0, depth+1)] + p[IDX(width+1, 1, depth+1)] + p[IDX(width+1, 0, depth)]) / 3.0;
    p[IDX(0, height+1, depth+1)] = (p[IDX(1, height+1, depth+1)] + p[IDX(0, height, depth+1)] + p[IDX(0, height+1, depth)]) / 3.0;
    p[IDX(width+1, height+1, depth+1)] = (p[IDX(width, height+1, depth+1)] + p[IDX(width+1, height, depth+1)] + p[IDX(width+1, height+1, depth)]) / 3.0;
  }

    void set_bnd (FieldCell* cells){
        int i, j, k;
        for (i=1 ; i<=width; i++) {
            CellAt(cells, i, 0, 0)->velocity = -CellAt(cells, i, 1, 1)->velocity;
            CellAt(cells, i, height+1, 0)->velocity = -CellAt(cells, i, height, 1)->velocity;
            CellAt(cells, i, 0, depth+1)->velocity = -CellAt(cells, i, 1, depth)->velocity;
            CellAt(cells, i, height+1, depth+1)->velocity = -CellAt(cells, i, height, depth)->velocity;
        }

        for (j=1 ; j<=height; j++) {
            CellAt(cells, 0, j, 0)->velocity = -CellAt(cells, 1, j, 1)->velocity;
            CellAt(cells, width+1, j, 0)->velocity = -CellAt(cells, width, j, 1)->velocity;
            CellAt(cells, 0, j, depth+1)->velocity = -CellAt(cells, 1, j, depth)->velocity;
            CellAt(cells, width+1, j, depth+1)->velocity = -CellAt(cells, width, j, depth)->velocity;
        }

        for (k=1 ; k<=depth; k++) {
            CellAt(cells, 0, 0, k)->velocity = -CellAt(cells, 1, 1, k)->velocity;
            CellAt(cells, 0, height+1, k)->velocity = -CellAt(cells, i, height, k)->velocity;
            CellAt(cells, width+1, 0, k)->velocity = -CellAt(cells, width, 1, k)->velocity;
            CellAt(cells, width+1, height+1, k)->velocity = -CellAt(cells, width, height, k)->velocity;
        }


        CellAt(cells, 0, 0, 0)->velocity = (CellAt(cells, 1, 0, 0)->velocity + CellAt(cells, 0, 1, 0)->velocity + CellAt(cells, 0, 0, 1)->velocity) / 3.0;
        CellAt(cells, width+1, 0, 0)->velocity = (CellAt(cells, width, 0, 0)->velocity + CellAt(cells, width+1, 1, 0)->velocity + CellAt(cells, width+1, 0, 1)->velocity) / 3.0;
        CellAt(cells, 0, height+1, 0)->velocity = (CellAt(cells, 1, height+1, 0)->velocity + CellAt(cells, 0, height, 0)->velocity + CellAt(cells, 0, height+1, 1)->velocity) / 3.0;
        CellAt(cells, 0, 0, depth+1)->velocity = (CellAt(cells, 1, 0, depth+1)->velocity + CellAt(cells, 0, 1, depth+1)->velocity + CellAt(cells, 0, 0, depth)->velocity) / 3.0;
        CellAt(cells, width+1, height+1, 0)->velocity = (CellAt(cells, width, height+1, 0)->velocity + CellAt(cells, width+1, height, 0)->velocity + CellAt(cells, width+1, height+1, 1)->velocity) / 3.0;
        CellAt(cells, width+1, 0, depth+1)->velocity = (CellAt(cells, width, 0, depth+1)->velocity + CellAt(cells, width+1, 1, depth+1)->velocity + CellAt(cells, width+1, 0, depth)->velocity) / 3.0;
        CellAt(cells, 0, height+1, depth+1)->velocity = (CellAt(cells, 1, height+1, depth+1)->velocity + CellAt(cells, 0, height, depth+1)->velocity + CellAt(cells, 0, height+1, depth)->velocity) / 3.0;
        CellAt(cells, width+1, height+1, depth+1)->velocity = (CellAt(cells, width, height+1, depth+1)->velocity + CellAt(cells, width+1, height, depth+1)->velocity + CellAt(cells, width+1, height+1, depth)->velocity) / 3.0;
    }


    FieldCell* CellAt(FieldCell *cells, int x, int y, int z) {
        return &cells[x + y * width + z * height * width];
    }

    FieldCell* CellAt(Vector3D position) {
        int x = (position.x - origin.x) / cell_size + 0.5 * cell_size * width;
        int y = (position.y - origin.y) / cell_size + 0.5 * cell_size * height;
        int z = (position.z - origin.z) / cell_size + 0.5 * cell_size * depth;

        if (x < 0 || x >= width || y < 0 || y >= height || z < 0 || z >= depth) {
            return NULL;
        }

        return CellAt(cells, x, y, z);
    }

    FieldCell InterpCell(FieldCell a, FieldCell b, double t) {
        FieldCell result;
        result.temperature = a.temperature + t * b.temperature  + (-t) * a.temperature;
        result.velocity = a.velocity + t * b.velocity  + (-t) * a.velocity;
        return result;
    }

    FieldCell CellAtInterpolated(Vector3D position) {
        double x = (position.x - origin.x) / cell_size + 0.5 * cell_size * width;
        double y = (position.y - origin.y) / cell_size + 0.5 * cell_size * height;
        double z = (position.z - origin.z) / cell_size + 0.5 * cell_size * depth;

        int left = clamp(static_cast<int>(x), 0, width - 1);
        int right = clamp(static_cast<int>(x + 0.5), 0, width - 1);
        int lower = clamp(static_cast<int>(y), 0, height - 1);
        int upper = clamp(static_cast<int>(y + 0.5), 0, height - 1);
        int front = clamp(static_cast<int>(z), 0, depth - 1);
        int back = clamp(static_cast<int>(z + 0.5), 0, depth - 1);

        // Left or Right
        // Upper or lower
        // Front or Back
        FieldCell* luf = CellAt(cells, left, upper, front);
        FieldCell* ruf = CellAt(cells, right, upper, front);
        FieldCell* llf = CellAt(cells, left, lower, front);
        FieldCell* rlf = CellAt(cells, right, lower, front);
        FieldCell* lub = CellAt(cells, left, upper, back);
        FieldCell* rub = CellAt(cells, right, upper, back);
        FieldCell* llb = CellAt(cells, left, lower, back);
        FieldCell* rlb = CellAt(cells, right, lower, back);

        double horizontal = x - static_cast<int>(x);
        double vertical = y - static_cast<int>(y);
        double forward = z - static_cast<int>(z);

        FieldCell left_upper = InterpCell(*luf, *lub, forward);
        FieldCell right_upper = InterpCell(*ruf, *rub, forward);
        FieldCell left_lower = InterpCell(*llf, *llb, forward);
        FieldCell right_lower = InterpCell(*rlf, *rlb, forward);

        FieldCell upper_cell = InterpCell(left_upper, right_upper, horizontal);
        FieldCell lower_cell = InterpCell(left_lower, right_lower, horizontal);

        return InterpCell(upper_cell, lower_cell, vertical);
    }

    void swap() {
        FieldCell *temp = cells;
        cells = prev_cells;
        prev_cells = temp;
    }

    Vector3D CellPos(int x, int y, int z) {
        return origin + cell_size * (Vector3D(x, y, z) - 0.5 * Vector3D(width, height, depth));
    }

    Vector3D origin; // Center of the grid field
    int width, height, depth;
    int size;
    double cell_size;
    FieldCell *cells;
    FieldCell *prev_cells;

    double *div;
    double *p;

}; // class GasField

#endif // GAS_FIELD_H
