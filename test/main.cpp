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
#include "odom_factor_2d.h"
#include "plane_factor.h"
#include "calculate_transformation.h"

using namespace std;
using namespace gtsam;
using namespace custom_node_2d;

CalculateTransformation cal_t;

int main(){
    NonlinearFactorGraph graph;

    default_random_engine generator;
    const float stddev = 1;
    const float mean = 0.0f;
    normal_distribution<float> dist(mean, stddev);


    Vector3 gt_plane_model = Vector3(-1.0, 0.0, 1.0);

    vector<Vector3> odom_measure;
    odom_measure.push_back(Vector3(0.0 + dist(generator)/10.0, 0.5 + dist(generator)/10.0, 0.0 + dist(generator)*M_PI/180.0));
    odom_measure.push_back(Vector3(0.0 + dist(generator)/10.0, 0.5 + dist(generator)/10.0, 0.0 + dist(generator)*M_PI/180.0));
    odom_measure.push_back(Vector3(0.0 + dist(generator)/10.0, 0.5 + dist(generator)/10.0, 0.0 + dist(generator)*M_PI/180.0));

    const float stddev_norm = 2*M_PI/180.0f;

    vector<Vector3> plane_measure;
    plane_measure.push_back(Vector3(-cos(dist(generator)*2*M_PI/180.0), -sin(dist(generator)*2*M_PI/180.0), 1.0 + dist(generator)/20.0));
    plane_measure.push_back(Vector3(-cos(dist(generator)*2*M_PI/180.0), -sin(dist(generator)*2*M_PI/180.0), 1.0 + dist(generator)/20.0));
    plane_measure.push_back(Vector3(-cos(dist(generator)*2*M_PI/180.0), -sin(dist(generator)*2*M_PI/180.0), 1.0 + dist(generator)/20.0));

    State2D priorState = State2D(0.0, 0.0, 0.0, -1.0, 0.0, 1.0);

    noiseModel::Diagonal::shared_ptr priorNoise =
        noiseModel::Diagonal::Sigmas((Vector(6)<<0.01, 0.01, 0.01, 0.01, 0.01, 0.01).finished());
    
    noiseModel::Diagonal::shared_ptr odomNoise = 
        noiseModel::Diagonal::Sigmas(Vector3(0.1,0.1, 1*M_PI/180.0));

    noiseModel::Diagonal::shared_ptr measureNoise = 
        noiseModel::Diagonal::Sigmas(Vector3(0.0001, 0.0001, 0.05));

    // int idx = 0;

    Vector3 temp;

    State2D for_initials = priorState;

    Values initials;
    graph.add(PriorFactor<State2D>(0, priorState, priorNoise));
    initials.insert(0, for_initials);
    graph.add(boost::make_shared<OdomFactor2D>(0, 1, odom_measure[0], odomNoise));
    graph.add(boost::make_shared<PlaneFactor>(0, 1, plane_measure[0], measureNoise));

    temp = cal_t.inGlobal(Vector3(for_initials.x, for_initials.y, for_initials.yaw), odom_measure[0]);

    for_initials = State2D(temp(0),temp(1),temp(2),
                           plane_measure[0](0), plane_measure[0](1), plane_measure[0](2));

    initials.insert(1, for_initials);

    graph.add(boost::make_shared<OdomFactor2D>(1, 2, odom_measure[1], odomNoise));
    graph.add(boost::make_shared<PlaneFactor>(1, 2, plane_measure[1], measureNoise));

    temp = cal_t.inGlobal(Vector3(for_initials.x, for_initials.y, for_initials.yaw), odom_measure[1]);
    for_initials = State2D(temp(0),temp(1),temp(2),
                           plane_measure[1](0), plane_measure[1](1), plane_measure[1](2));

    initials.insert(2, for_initials);

    graph.add(boost::make_shared<OdomFactor2D>(2, 3, odom_measure[2], odomNoise));
    graph.add(boost::make_shared<PlaneFactor>(2, 3, plane_measure[2], measureNoise));

    temp = cal_t.inGlobal(Vector3(for_initials.x, for_initials.y, for_initials.yaw), odom_measure[2]);

    for_initials = State2D(temp(0),temp(1),temp(2),
                           plane_measure[2](0), plane_measure[2](1), plane_measure[2](2));

    initials.insert(3, for_initials);

    initials.print("Initial Results\n");

    Values results = LevenbergMarquardtOptimizer(graph, initials).optimize();
    // Values results = GaussNewtonOptimizer(graph, initials).optimize();

    // results.print("Final Results\n");

    for(int i=0; i<results.size(); ++i){
        State2D optimized_result = results.at<State2D>(i);
        std::cout<<"No. "<<to_string(i)<<": "<<std::endl;
        std::cout<<"X:   "<<optimized_result.x<<std::endl;
        std::cout<<"Y:   "<<optimized_result.y<<std::endl;
        std::cout<<"PSI: "<<optimized_result.yaw<<std::endl;
        std::cout<<"Nx:  "<<optimized_result.nx<<std::endl;
        std::cout<<"Ny:  "<<optimized_result.ny<<std::endl;
        std::cout<<"d:   "<<optimized_result.d<<std::endl<<std::endl;
    }



    return 0;

}