#include "odom_factor2d.h"

OdomFactor2D::OdomFactor2D(Key key1, Key key2,
                           const Vector2 measured, const SharedNoiseModel & model):
                             NoiseModelFactor2(model, key1, key2),
                             measured_(measured){
}

Vector OdomFactor2D::evaluateError(const State& s1, const State& s2,
                                   boost::optional<Matrix&> H1,
                                   boost::optional<Matrix&> H2) const{

    if(H1){
        Matrix H1_ = Matrix::Zero(2,3);
        H1_(0,0) = -1.0;
        H1_(1,1) = -1.0;
        *H1 = H1_;
    }
    if(H2){
        Matrix H2_ = Matrix::Zero(2,3);
        H2_(0,0) = 1.0;
        H2_(1,1) = 1.0;
        *H2 = H2_;
    }

    Vector2 result;

    result(0) = s2.x - (s1.x + measured_(0));
    result(1) = s2.y - (s1.y + measured_(1));

    return result;
}