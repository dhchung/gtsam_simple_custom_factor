#pragma once
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <iostream>

namespace custom_node{
    struct State{
        double x;
        double y;
        double z;

    State(double xi, double yi, double zi):
               x(xi), y(yi), z(zi){}
    };



}

namespace gtsam{
    template<>
    struct traits<custom_node::State>{

        static void Print(const custom_node::State & m, const std::string & str = ""){
            std::cout<<str<<"("<<m.x<<", "<<m.y<<", "<<m.z<<std::endl;
        }

        static bool Equals(const custom_node::State &m1, const custom_node::State &m2, double tol = 1e-8){
            if(fabs(m1.x-m2.x)<tol &&
               fabs(m1.y-m2.y)<tol &&
               fabs(m1.z-m2.z)<tol){
                return true;
            }else{
                return false;
            }
        }

        enum{dimension = 3};
        static int GetDimension(const custom_node::State&) {return dimension;}

        typedef custom_node::State ManifoldType;
        typedef Eigen::Matrix<double, dimension, 1> TangentVector;

        static TangentVector Local(const custom_node::State& origin,
                                   const custom_node::State& other){

            Vector3 result;
            result(0) = other.x-origin.x;
            result(1) = other.y-origin.y;
            result(2) = other.z-origin.z;

            return result;
        }

        static custom_node::State Retract(const custom_node::State& origin,
                                                    const TangentVector& v){
            return custom_node::State(origin.x+v(0),
                                            origin.y+v(1),
                                            origin.z+v(2));
        }
    };
}

