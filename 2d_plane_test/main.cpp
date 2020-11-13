#include <iostream>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/PriorFactor.h>

#include <random>
#include <iterator>
#include <math.h>

#include "custom_node_2d.h"
#include "odom_factor2d.h"
#include "plane_factor.h"

using namespace std;
using namespace gtsam;
using namespace custom_node_2d;

int main(){
    NonlinearFactorGraph graph;

    default_random_engine generator;
    const float stddev = 0.1;
    const float mean = 0.0f;
    normal_distribution<float> dist(mean, stddev);

    vector<Vector2> odom_measure;

    odom_measure.push_back(Vector2(0.0f + dist(generator), -1.0f + dist(generator)));
    odom_measure.push_back(Vector2(1.0f + dist(generator), 0.0f + dist(generator)));
    odom_measure.push_back(Vector2(0.0f + dist(generator), 1.0f + dist(generator)));

    for(int i = 0; i < odom_measure.size(); ++i){
        std::cout<<odom_measure[i]<<std::endl<<std::endl;
    }

    Vector1 depth = Vector1(0.0);


    State priorState = State(1.0, 2.0, 0.0);
    noiseModel::Diagonal::shared_ptr priorNoise =
        noiseModel::Diagonal::Sigmas(Vector3(0.01,0.01,0.01));
    
    noiseModel::Diagonal::shared_ptr odomNoise = 
        noiseModel::Diagonal::Sigmas(Vector2(0.01,0.01));

    noiseModel::Diagonal::shared_ptr measureNoise = 
        noiseModel::Diagonal::Sigmas(Vector2(0.0001, 0.0001));

    noiseModel::Diagonal::shared_ptr depthNoise = 
        noiseModel::Diagonal::Sigmas(Vector1(0.1));

    // int idx = 0;

    State for_initials = priorState;

    Values initials;
    graph.add(PriorFactor<State>(0, priorState, priorNoise));
    initials.insert(0, for_initials);
    graph.add(boost::make_shared<OdomFactor2D>(0, 1, odom_measure[0], odomNoise));
    graph.add(boost::make_shared<DepthFactor>(0,1, depth, depthNoise));

    for_initials = State(for_initials.x + odom_measure[0](0),
                         for_initials.y + odom_measure[0](1),
                         for_initials.z + 0);
    initials.insert(1, for_initials);
    // initials.insert(1, State(0,0,0));

    graph.add(boost::make_shared<OdomFactor2D>(1, 2, odom_measure[1], odomNoise));
    graph.add(boost::make_shared<DepthFactor>(1, 2, depth, depthNoise));
    for_initials = State(for_initials.x + odom_measure[1](0),
                         for_initials.y + odom_measure[1](1),
                         for_initials.z + 0);
    initials.insert(2, for_initials);
    // initials.insert(2, State(0,0,0));

    graph.add(boost::make_shared<OdomFactor2D>(2, 3, odom_measure[2], odomNoise));
    graph.add(boost::make_shared<DepthFactor>(2, 3, depth, depthNoise));

    for_initials = State(for_initials.x + odom_measure[2](0),
                         for_initials.y + odom_measure[2](1),
                         for_initials.z + 0);
    initials.insert(3, for_initials);
    // initials.insert(3, State(0,0,0));

    graph.add(boost::make_shared<OdomFactor2D>(3, 0, Vector2(-1.0, 0.0), measureNoise));
    graph.add(boost::make_shared<DepthFactor>(3, 0, depth, depthNoise));


    initials.print("Initial Results\n");

    Values results = LevenbergMarquardtOptimizer(graph, initials).optimize();
    // Values results = GaussNewtonOptimizer(graph, initials).optimize();

    results.print("Final Results\n");

    Marginals marginals(graph, results);

    cout << "x1 covariance:\n" << marginals.marginalCovariance(1) << endl;
    cout << "x2 covariance:\n" << marginals.marginalCovariance(2) << endl;
    cout << "x3 covariance:\n" << marginals.marginalCovariance(3) << endl;


    return 0;

}