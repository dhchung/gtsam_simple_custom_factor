#include "calculate_transformation.h"

Vector3 CalculateTransformation::inGlobal(Vector3 in, Vector3 dstate){
    double x = in(0);
    double y = in(1);
    double psi = in(2);

    Matrix R = Matrix::Zero(2,2);
    R(0,0) = cos(psi);
    R(0,1) = -sin(psi);
    R(1,0) = sin(psi);
    R(1,1) = cos(psi);

    Vector2 dstate_global = R*dstate.segment(0,1);
    Vector3 out = Vector3(dstate_global(0), dstate_global(1), psi + dstate(2));
    return out;
}