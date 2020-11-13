#pragma once
#include <gtsam/base/Testable.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include "custom_node.h"
#include "math.h"

using namespace gtsam;
using namespace custom_node;

class DepthFactor : public NoiseModelFactor2<State, State>{

private:
    Vector1 measured_;

public:
    DepthFactor(Key key1, Key key2,
                const Vector1 measured, const SharedNoiseModel & model = nullptr);
    Vector evaluateError(const State& s1, const State& s2,
                         boost::optional<Matrix&> H1 = boost::none,
                         boost::optional<Matrix&> H2 = boost::none) const;
};