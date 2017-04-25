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


class Point2Line2D : public g2o::BaseBinaryEdge<1, g2o::VertexSE3, Line3D> {
public:
    EIGEN_MAKE_ALIGEND_OPERATOR_NEW;

    Point2Line2D() {

    }

    virtual bool read(std::istream &is) {
        return true;
    }

    virtual bool write(std::ostream &os) {
        return true;
    }

    void computeErros() {
        double p[10] = {0};
        double l[10] = {0};


        const auto *Line = static_cast<const Line2D *>(vertices()[0]);
        Line->getEstimateData(l);
        auto *point = static_cast<g2o::VertexSE3 *>(vertices()[1]);
        point->getEstimateData(p);


        _error(0, 0) = std::abs(l[0] * p[0] + l[1] * p[1] + 1.0) /
                       std::sqrt(l[0] * l[0] + l[1] * l[1]);
    }


};


#endif //MAGNETOMETERFEATUREAUGUMENTATIONLOCATION_POINT2LINE2D_H
