//
// Created by steve on 17-4-25.
//

#include "g2o/stuff/sampler.h"
#include "g2o/stuff/command_args.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/solvers/dense/linear_solver_dense.h"


#include "OwnEdge/Line2D.h"
#include "OwnEdge/Line2D.cpp"


#ifndef MAGNETOMETERFEATUREAUGUMENTATIONLOCATION_POINT2LINE2D_H
#define MAGNETOMETERFEATUREAUGUMENTATIONLOCATION_POINT2LINE2D_H


class Point2Line2D : public g2o::BaseBinaryEdge<1, double, Line2D, g2o::VertexSE3> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    Point2Line2D() {

    }

    virtual bool read(std::istream &is) {
        return true;
    }

    virtual bool write(std::ostream &os) const {
        return true;
    }


    void computeError();


};


#endif //MAGNETOMETERFEATUREAUGUMENTATIONLOCATION_POINT2LINE2D_H
