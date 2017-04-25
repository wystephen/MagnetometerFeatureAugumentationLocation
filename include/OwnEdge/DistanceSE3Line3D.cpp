//
// Created by steve on 17-4-25.
//

#include "DistanceSE3Line3D.h"

DistanceSE3Line3D::DistanceSE3Line3D() :
        g2o::BaseBinaryEdge<1, double, g2o::VertexSE3, g2o::VertexLine3D>() {
    information().setIdentity();
    _information(0, 0) = 10.0f;
}

bool DistanceSE3Line3D::read(std::istream &is) {
    return true;
}


bool DistanceSE3Line3D::write(std::ostream &os) const {
    return true;
}


void DistanceSE3Line3D::computeError() {
    /// Wait
    g2o::VertexSE3 *from = static_cast<g2o::VertexSE3 *>(_vertices[0]);
    g2o::VertexLine3D *to = static_cast<g2o::VertexLine3D *>(_vertices[1]);
    double p[10] = {0};
    from->getEstimateData(p);

    auto line = to->estimate();
    line.normalize();
    auto direct = line.d();
    auto npoint = line.d().cross(line.w());

    Eigen::Vector3d pose(p[0], p[1], p[2]);

    auto u = pose - npoint;

//    _error(0,0) = std::sqrt(std::pow(u.norm(),2.0)-
//                                    std::pow((u.dot(direct)).cross(dir;
    _error(0, 0) = double((u.cross(direct)).norm() / (direct.norm()));

}


bool DistanceSE3Line3D::setMeasurementFromState() {
    setMeasurement(0.0f);
    return true;
}


void DistanceSE3Line3D::initialEstimate(const g2o::OptimizableGraph::VertexSet &from,
                                        g2o::OptimizableGraph::Vertex *to) {

}

