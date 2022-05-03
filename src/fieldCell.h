#ifndef FIELD_CELL_H
#define FIELD_CELL_H

#include <vector>

#include "CGL/CGL.h"

using namespace CGL;
using namespace std;

struct FieldCell {
public:
    double pressure;
    Vector3D velocity;
    double temperature;
    double divergence;
    bool isBoundary;
    Vector3D force;
    double heat_transfer;
}; // struct FieldCell

#endif // FIELD_CELL_H
