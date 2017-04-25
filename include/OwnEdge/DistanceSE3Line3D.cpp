//
// Created by steve on 17-4-25.
//

#include "DistanceSE3Line3D.h"

DistanceSE3Line3D::DistanceSE3Line3D() :
        g2o::BaseBinaryEdge<1, double, g2o::VertexSE3, g2o::VertexLine3D>() {
    information().setIdentity();
    _information *= 10.0f;
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


}


bool DistanceSE3Line3D::setMeasurementFromState() {
    setMeasurement(0.0f);
    return true;
}


void DistanceSE3Line3D::initialEstimate(const g2o::OptimizableGraph::VertexSet &from,
                                        g2o::OptimizableGraph::Vertex *to) {

}

