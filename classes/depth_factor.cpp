#include "depth_factor.h"

DepthFactor::DepthFactor(Key key1, Key key2,
                         const Vector1 measured, const SharedNoiseModel & model):
                             NoiseModelFactor2(model, key1, key2),
                             measured_(measured){
}

Vector DepthFactor::evaluateError(const State& s1, const State& s2,
                                 boost::optional<Matrix&> H1,
                                 boost::optional<Matrix&> H2) const{

    if(H1){
        Matrix H1_ = Matrix::Zero(1,3);
        H1_(0,2) = -1;
        *H1 = H1_;
    }
    if(H2){
        Matrix H2_ = Matrix::Zero(1,3);
        H2_(0,2) = 1;
        *H2 = H2_;
    }

    Vector1 result;

    result(0) = s2.z - (s1.z + measured_(0));


    return result;
}