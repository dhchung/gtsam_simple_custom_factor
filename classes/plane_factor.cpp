#include "plane_factor.h"

PlaneFactor::PlaneFactor(Key key1, Key key2,
                           const Vector3 measured, const SharedNoiseModel & model):
                             NoiseModelFactor2(model, key1, key2),
                             measured_(measured){
}

Vector PlaneFactor::evaluateError(const State2D& s1, const State2D& s2,
                                   boost::optional<Matrix&> H1,
                                   boost::optional<Matrix&> H2) const{

    if(H1){
        Matrix H1_ = Matrix::Zero(3,6);
        H1_.block(0,2,2,1) = rotation_2d(s2.yaw).transpose()*rotation_jacobian(s1.yaw)*Vector2(s1.nx, s1.ny);
        H1_.block(0,3,2,1) = rotation_2d(s2.yaw).transpose()*rotation_2d(s1.yaw)*Vector2(1.0, 0.0);
        H1_.block(0,4,2,1) = rotation_2d(s2.yaw).transpose()*rotation_2d(s1.yaw)*Vector2(0.0, 1.0);
        H1_(2,0) = Vector2(-1.0, 0.0).transpose()*rotation_2d(s1.yaw).transpose()*Vector2(s1.nx, s1.ny);
        H1_(2,1) = Vector2(0.0, -1.0).transpose()*rotation_2d(s1.yaw).transpose()*Vector2(s1.nx, s1.ny);
        H1_(2,2) = Vector2(s2.x - s1.x, s2.y - s1.y).transpose()*rotation_jacobian(s1.yaw)*Vector2(s1.nx, s1.ny);
        H1_(2,3) = Vector2(s2.x - s1.x, s2.y - s1.y).transpose()*rotation_2d(s1.yaw)*Vector2(1.0, 0.0);
        H1_(2,4) = Vector2(s2.x - s1.x, s2.y - s1.y).transpose()*rotation_2d(s1.yaw)*Vector2(0.0, 1.0);
        H1_(2,5) = 1.0;
        *H1 = H1_;
    }
    if(H2){
        Matrix H2_ = Matrix::Zero(3,6);
        H2_.block(0,2,2,1) = rotation_jacobian(s2.yaw).transpose()*rotation_2d(s1.yaw)*Vector2(s1.nx, s1.ny);
        H2_(2,0) = Vector2(1.0, 0.0).transpose()*rotation_2d(s1.yaw).transpose()*Vector2(s1.nx, s1.ny);
        H2_(2,1) = Vector2(0.0, 1.0).transpose()*rotation_2d(s1.yaw).transpose()*Vector2(s1.nx, s1.ny);
        *H2 = H2_;
    }

    Vector2 estimation_n = rotation_2d(s2.yaw).transpose()*rotation_2d(s1.yaw)*Vector2(s1.nx, s1.ny);
    double estimation_d = Vector2(s2.x - s1.x, s2.y - s1.y).transpose()*rotation_2d(s1.yaw)*Vector2(s1.nx, s1.ny) + s1.d;


    Vector3 result;
    result.segment(0,2) = estimation_n - measured_.segment(0,2);
    result(2) = estimation_d - measured_(2);
    
    return result;
}

Matrix PlaneFactor::rotation_2d(double psi) const{
    Matrix R = Matrix::Zero(2,2);
    R(0,0) = cos(psi);
    R(0,1) = -sin(psi);
    R(1,0) = sin(psi);
    R(1,1) = cos(psi);
    return R;
}
Matrix PlaneFactor::rotation_jacobian(double psi) const{
    Matrix J = Matrix::Zero(2,2);
    J(0,0) = -sin(psi);
    J(0,1) = -cos(psi);
    J(1,0) = cos(psi);
    J(1,1) = -sin(psi);
    return J;
}