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

#ifndef MAGNETOMETERFEATUREAUGUMENTATIONLOCATION_LINE2D_H
#define MAGNETOMETERFEATUREAUGUMENTATIONLOCATION_LINE2D_H


class Line2D : public g2o::BaseVertex<2, Eigen::Vector2d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    Line2D() {

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
        for (int i(0); i < 2; ++i) {
            _estimate(i) += update[i];
        }
    }


    virtual bool setEstimateDataImpl(const double *est) {
//        for(int i(0);i<6;++i)
//        {
//            est[i] = _estimate(i);
//        }

        Eigen::Map<const T> _est(est);
        _estimate = _est;
        return true;
    }

    virtual bool getEstimateData(double *est) const {
        Eigen::Map<T> _est(est);
        _est = _estimate;
        return true;
    }

    virtual int estimateDimension() const {
        return 2;
    }


    virtual bool setMinimalEstimateDataImpl(const double *est) {
        _estimate = Eigen::Map<T>(est);
        return true;
    }

    virtual bool getMinimalEstimateData(double *est) const {
        Eigen::Map<T> v(est);
        v = _estimate;
        return true;
    }


    virtual int minimalEstimateDimension() const {
        return 2;
    }

};


#endif //MAGNETOMETERFEATUREAUGUMENTATIONLOCATION_LINE2D_H
