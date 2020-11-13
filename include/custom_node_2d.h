#pragma once
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <iostream>

namespace custom_node_2d{
    struct State2D{
        double x;
        double y;
        double yaw;
        double nx;
        double ny;
        double d;

    State2D(double xi, double yi, double yawi, double nxi, double nyi, double di):
               x(xi), y(yi), yaw(yawi), nx(nxi), ny(nyi), d(di){}
    };
}

namespace gtsam{
    template<>
    struct traits<custom_node_2d::State2D>{

        static void Print(const custom_node_2d::State2D & m, const std::string & str = ""){
            std::cout<<str<<"("<<m.x<<", "<<m.y<<", "<<m.yaw<<", "<<m.nx<<", "<<m.ny<<", "<<m.d<<std::endl;
        }

        static bool Equals(const custom_node_2d::State2D &m1, const custom_node_2d::State2D &m2, double tol = 1e-8){
            if(fabs(m1.x - m2.x) < tol &&
               fabs(m1.y - m2.y) < tol &&
               fabs(m1.yaw - m2.yaw) < tol &&
               fabs(m1.nx - m2.nx) < tol &&
               fabs(m1.ny - m2.ny) < tol &&
               fabs(m1.d - m2.d) < tol){
                return true;
            }else{
                return false;
            }
        }

        enum{dimension = 6};
        static int GetDimension(const custom_node_2d::State2D&) {return dimension;}

        typedef custom_node_2d::State2D ManifoldType;
        typedef Eigen::Matrix<double, dimension, 1> TangentVector;

        static TangentVector Local(const custom_node_2d::State2D& origin,
                                   const custom_node_2d::State2D& other){

            Vector6 result;
            result(0) = other.x-origin.x;
            result(1) = other.y-origin.y;
            result(2) = other.yaw-origin.yaw;
            result(3) = other.nx - origin.nx;
            result(4) = other.ny - origin.ny;
            result(5) = other.d - origin.d;

            return result;
        }

        static custom_node_2d::State2D Retract(const custom_node_2d::State2D& origin,
                                                    const TangentVector& v){
            return custom_node_2d::State2D(origin.x + v(0),
                                      origin.y + v(1),
                                      origin.yaw + v(2),
                                      origin.nx + v(3),
                                      origin.ny + v(4),
                                      origin.d + v(5));
        }
    };
}