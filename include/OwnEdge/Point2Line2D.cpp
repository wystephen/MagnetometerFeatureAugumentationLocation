//
// Created by steve on 17-4-25.
//

#include "Point2Line2D.h"

void Point2Line2D::computeError() {

    double p[10] = {0};
    double l[10] = {0};

    std::cout << "compute error" << std::endl;

    auto *Line = static_cast<g2o::VertexSE3 *>(vertices()[0]);
    Line->getEstimateData(l);
    auto *point = static_cast<Line2D *>(vertices()[1]);
    point->getEstimateData(p);

    std::cout << "after load data " << std::endl;

    _error(0, 0) = std::abs(l[0] * p[0] + l[1] * p[1] + 1.0) /
                   std::sqrt(l[0] * l[0] + l[1] * l[1]);

}


bool Point2Line2D::setMeasurementFromState() {
    setMeasurement(0.0f);
    return true;
}

void Point2Line2D::initialEstimate(const g2o::OptimizableGraph::VertexSet &from,
                                   g2o::OptimizableGraph::Vertex *to) {
    /**
     * Do nothing.
     */
}