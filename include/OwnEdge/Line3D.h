//
// Created by steve on 17-4-21.
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

#ifndef MAGNETOMETERFEATUREAUGUMENTATIONLOCATION_LINE3D_H
#define MAGNETOMETERFEATUREAUGUMENTATIONLOCATION_LINE3D_H


class Line3D : public g2o::BaseVertex<6, Eigen::Matrix<double, 1, 6>> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    Line3D() {

    }

    virtual bool read(std::istream &is) {
        std::cerr << __PRETTY_FUNCTION__ << "not implemented yet" << endl;
        return true;
    }

    virtual bool write(std::ostream &os) {

        std::cerr << __PRETTY_FUNCTION__ << "not implemented yet" << endl;
        return true;
    }

    virtual void setToOriginImpl() {
        std::cerr << __PRETTY_FUNCTION__ << "not implemented yet" << endl;
    }

    virtual void oplusImpl(const double *update) {
        for (int i(0); i < 6; ++i) {
            _estimate(i) += update[i];
        }
    }


};


#endif //MAGNETOMETERFEATUREAUGUMENTATIONLOCATION_LINE3D_H
