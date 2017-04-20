//
// Created by steve on 17-4-15.
//

#include "DistanceEdge.h"

DistanceEdge::DistanceEdge() :
        g2o::BaseBinaryEdge<1, double, g2o::VertexSE3, g2o::VertexSE3>() {
    /**
     *
     */
    information().setIdentity();
    _information(0, 0) = 10.0f;
}

bool DistanceEdge::read(std::istream &is) {
    /**
     * TODO: ACHIVE IT!!!
     */
    return true;
}

bool DistanceEdge::write(std::ostream &os) const {
    return os.good();
}

void DistanceEdge::computeError() {
    g2o::VertexSE3 *from = static_cast<g2o::VertexSE3 *>(_vertices[0]);
    g2o::VertexSE3 *to = static_cast<g2o::VertexSE3 *>(_vertices[1]);

    double p1[10], p2[10];
    from->getEstimateData(p1);
    to->getEstimateData(p2);
    _error(0, 0) = std::pow(std::sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) +
                             (p1[1] - p2[1]) * (p1[1] - p2[1]) +
                                      (p1[2] - p2[2]) * (p1[2] - p2[2])) - (_measurement), 2.0);
}

bool DistanceEdge::setMeasurementFromState() {
    setMeasurement(0.0f);
    return true;
}


void DistanceEdge::linearizeOplus() {
    g2o::VertexSE3 *from = static_cast<g2o::VertexSE3 *>(_vertices[0]);
    g2o::VertexSE3 *to = static_cast<g2o::VertexSE3 *>(_vertices[1]);

    double p1[10], p2[10];
    from->getEstimateData(p1);
    to->getEstimateData(p2);

    _jacobianOplusXi.setZero();
    _jacobianOplusXj.setZero();
    double dis = std::sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) +
                           (p1[1] - p2[1]) * (p1[1] - p2[1]) +
                           (p1[2] - p2[2]) * (p1[2] - p2[2]));

    for (int i(0); i < 3; ++i) {
        if (std::abs(p1[i] - p2[i]) - _measurement < 0.0000001) {
            continue;
        }
        _jacobianOplusXi(0, i) = -(p1[i] - p2[i]) / dis * 2.0 * (dis - _measurement);
        _jacobianOplusXj(0, i) = -(p1[i] - p2[i]) / dis * 2.0 * (dis - _measurement);
    }

}


void DistanceEdge::initialEstimate(const g2o::OptimizableGraph::VertexSet &from,
                                   g2o::OptimizableGraph::Vertex *to) {

    /**
     * Do nothing
     */
}