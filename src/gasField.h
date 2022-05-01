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

        cells = new FieldCell[width * height * depth];

        // Initialize cell values
        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {
                for (int k = 0; k < depth; k++) {
                    FieldCell* cell = CellAt(i, j, k);
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

    FieldCell* CellAt(int x, int y, int z) {
        return &cells[x + y * width + z * height * width];
    }

    int width, height, depth;
    double cell_size;
    FieldCell *cells;

}; // class GasField

#endif // GAS_FIELD_H
