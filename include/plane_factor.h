#pragma once
#include <gtsam/base/Testable.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include "custom_node_2d.h"
#include "math.h"

using namespace gtsam;
using namespace custom_node_2d;

class PlaneFactor : public NoiseModelFactor2<State2D, State2D>{

private:
    Vector3 measured_;

public:
    PlaneFactor(Key key1, Key key2,
                 const Vector3 measured, const SharedNoiseModel & model = nullptr);
    Vector evaluateError(const State2D& s1, const State2D& s2,
                         boost::optional<Matrix&> H1 = boost::none,
                         boost::optional<Matrix&> H2 = boost::none) const;

    Matrix rotation_2d(double psi) const;
    Matrix rotation_jacobian(double psi) const;
};