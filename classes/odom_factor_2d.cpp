#include "odom_factor_2d.h"

OdomFactor2D::OdomFactor2D(Key key1, Key key2,
                           const Vector3 measured, const SharedNoiseModel & model):
                             NoiseModelFactor2(model, key1, key2),
                             measured_(measured){
}

Vector OdomFactor2D::evaluateError(const State2D& s1, const State2D& s2,
                                   boost::optional<Matrix&> H1,
                                   boost::optional<Matrix&> H2) const{

    if(H1){
        Matrix H1_ = Matrix::Zero(3,6);
        H1_.block(0,0,2,1) = rotation_2d(s1.yaw).transpose() * Vector2(-1.0, 0.0);
        H1_.block(0,1,2,1) = rotation_2d(s1.yaw).transpose() * Vector2(0.0, -1.0);
        H1_.block(0,2,2,1) = rotation_jacobian(s1.yaw).transpose() * Vector2(s2.x - s1.x, s2.y - s1.y);
        H1_(2,2) = -1;
        *H1 = H1_;
    }
    if(H2){
        Matrix H2_ = Matrix::Zero(3,6);
        H2_.block(0,0,2,1) = rotation_2d(s1.yaw).transpose() * Vector2(1.0, 0.0);
        H2_.block(0,1,2,1) = rotation_2d(s1.yaw).transpose() * Vector2(0.0, 1.0);
        H2_(2,2) = 1;
        *H2 = H2_;
    }

    Vector2 estimation = rotation_2d(s1.yaw)*measured_.segment(0,1);

    Vector3 result;
    result(0) = s2.x - (s1.x + estimation(0));
    result(1) = s2.y - (s1.y + estimation(1));
    result(2) = s2.yaw - (s1.yaw + measured_(2));

    return result;
}

Matrix OdomFactor2D::rotation_2d(double psi) const{
    Matrix R = Matrix::Zero(2,2);
    R(0,0) = cos(psi);
    R(0,1) = -sin(psi);
    R(1,0) = sin(psi);
    R(1,1) = cos(psi);
    return R;
}
Matrix OdomFactor2D::rotation_jacobian(double psi) const{
    Matrix J = Matrix::Zero(2,2);
    J(0,0) = -sin(psi);
    J(0,1) = -cos(psi);
    J(1,0) = cos(psi);
    J(1,1) = -sin(psi);
    return J;
}