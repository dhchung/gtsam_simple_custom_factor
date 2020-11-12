#pragma once
#include <gtsam/base/Testable.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include "custom_node.h"
#include "math.h"

using namespace gtsam;
using namespace custom_node;

class OdomFactor2D : public NoiseModelFactor2<State, State>{

private:
    Vector2 measured_;

public:
    OdomFactor2D(Key key1, Key key2,
                 const Vector2 measured, const SharedNoiseModel & model = nullptr);
    Vector evaluateError(const State& s1, const State& s2,
                         boost::optional<Matrix&> H1 = boost::none,
                         boost::optional<Matrix&> H2 = boost::none) const;
};