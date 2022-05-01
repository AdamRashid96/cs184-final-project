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

        cells = new FieldCell[size];

        // Initialize cell values
        for (int i = 0; i < width + 2; i++) {
            for (int j = 0; j < height + 2; j++) {
                for (int k = 0; k < depth + 2; k++) {
                    FieldCell* cell = CellAt(cells, i, j, k);
                    if (i == 0 || i == width + 1 || j == 0 || j == height + 1 || k == 0 || k == depth + 1) {
                        cell->isBoundary = true;
                    } else {
                        cell->isBoundary = false;
                    }
                    cell->temperature = ambient_temperature;
                    cell->pressure = base_pressure;
                    cell->divergence = 0;
                    cell->velocity = get_sample() * random_uniform() * initial_velocity;
                }
            }
        }
    }

    ~GasField() {
        delete [] cells;
    }
    
    void add_vel(Vector3D *s, float dt) {
        int i;
        for (i=0 ; i<size ; i++) { 
            cells[i].velocity += dt*s[i];
        }
    }

    void diffuse_vel(float diff, float dt )
    {
        int i, j, k, t;
        float a=dt*diff*width*height*depth;

        for (t=0; t<20; t++) {
            for (i=1; i<=width; i++) {
                for (j=1; j<=height; j++) {
                    for (k = 1; k <=depth; k++) {
                        CellAt(cells, i, j, k)->velocity = (CellAt(prev_cells, i, j, k)->velocity + a * (CellAt(cells, i-1, j, k)->velocity +
                                                            CellAt(cells, i+1, j, k)->velocity + CellAt(cells, i, j-1, k)->velocity + CellAt(cells, i, j+1, k)->velocity + 
                                                            CellAt(cells, i, j, k-1)->velocity + CellAt(cells, i, j, k+1)->velocity)) / (1 + 6*a);
                    }
                }
            }
        set_bnd (cells);
        }
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

    int width, height, depth;
    int size;
    double cell_size;
    FieldCell *cells;
    FieldCell *prev_cells;

}; // class GasField

#endif // GAS_FIELD_H
