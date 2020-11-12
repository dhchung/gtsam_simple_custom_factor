#include "odom_factor.h"

OdomFactor::OdomFactor(Key key1, Key key2,
                       const Vector3 measured, const SharedNoiseModel & model):
                             NoiseModelFactor2(model, key1, key2),
                             measured_(measured){
}

Vector OdomFactor::evaluateError(const State& s1, const State& s2,
                                 boost::optional<Matrix&> H1,
                                 boost::optional<Matrix&> H2) const{

    if(H1){
        *H1 = -Matrix::Identity(3,3);
    }
    if(H2){
        *H2 = Matrix::Identity(3,3);
    }

    Vector3 result;

    result(0) = s2.x - (s1.x + measured_(0));
    result(1) = s2.y - (s1.y + measured_(1));
    result(2) = s2.z - (s1.z + measured_(2));


    return result;
}