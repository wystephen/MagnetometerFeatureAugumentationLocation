//
// Created by steve on 17-4-18.
//
#include <iostream>
#include <thread>

#include <fstream>


//#include "Eigen/Dense"
//#include "Eigen/Geometry"
#include<Eigen/Dense>
#include <Eigen/Geometry>

#include "Extent/CSVReader.h"
#include "ZUPT/MYEKF.h"
#include "ZUPT/EKF.hpp"

#include "Extent/time_stamp.h"
#include "Extent/matplotlib_interface.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam3d_addons/types_slam3d_addons.h"

#include "g2o/core/robust_kernel.h"
#include "g2o/core/robust_kernel_factory.h"

#include "OwnEdge/ZoEdge.h"
#include "OwnEdge/ZoEdge.cpp"
#include "OwnEdge/DistanceEdge.h"
#include "OwnEdge/DistanceEdge.cpp"

G2O_USE_TYPE_GROUP(slam3d)


int main(int argc, char *argv[]) {

    double first_info(1000), second_info(1000), distance_info(0.001), corner_ratio(10.0);
    double zero_info(1000);

    /// parameters
    std::cout << "para num :" << argc << std::endl;
    if (argc > 3) {
        first_info = std::stod(argv[1]);
        second_info = std::stod(argv[2]);
        distance_info = std::stod(argv[3]);

    }

    if (argc >= 5) {
        corner_ratio = std::stod(argv[4]);
    }

    if (argc >= 6) {
        zero_info = std::stod(argv[5]);
    }

    std::cout << "first info :" << first_info
              << "second info :" << second_info
              << "distance info :" << distance_info
              << " corner ratio :" << corner_ratio
              << " zero info :" << zero_info
              << std::endl;


    /// Load Data
    CSVReader vertex_pose_file("./TMP_DATA/vertex_pose.csv");
    CSVReader vertex_quat_file("./TMP_DATA/vertex_quat.csv");
    CSVReader close_id_file("./TMP_DATA/close_vetices_num_full.csv");

    auto vertex_pose(vertex_pose_file.GetMatrix());
    auto vertex_quat(vertex_quat_file.GetMatrix());
    auto close_id(close_id_file.GetMatrix());


    /// Initial graph
    g2o::SparseOptimizer globalOptimizer;


    typedef g2o::BlockSolver_6_3 SlamBlockSolver;
    typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

    // Initial solver
    SlamLinearSolver *linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver *blockSolver = new SlamBlockSolver(linearSolver);
    g2o::OptimizationAlgorithmLevenberg *solver =
            new g2o::OptimizationAlgorithmLevenberg(blockSolver);
    globalOptimizer.setAlgorithm(solver);


    /// read and push all id of corners , id of first vertex and last vertex

    std::vector<int> key_id;

    key_id.push_back(0);
    for (int i(0); i < close_id.GetRows(); ++i) {
        for (int j(0); j < close_id.GetCols(); ++j) {
            int tmp = *(close_id(i, j));
//            if(key_id.)
            if (std::find(key_id.begin(), key_id.end(), tmp) == key_id.end()) {
                key_id.push_back(tmp);
            }

        }
    }

    std::sort(key_id.begin(), key_id.end());


    /// Build base graph


    Eigen::Isometry3d last_t = (Eigen::Isometry3d::Identity());

    for (int index(0); index < vertex_pose.GetRows(); ++index) {
        /// build transfrom matrix

        Eigen::Isometry3d transform = (Eigen::Isometry3d::Identity());

        Eigen::Quaterniond the_quat;
        the_quat.x() = *vertex_quat(index, 0);
        the_quat.y() = *vertex_quat(index, 1);
        the_quat.z() = *vertex_quat(index, 2);
        the_quat.w() = *vertex_quat(index, 3);

//        std::cout << "vertex quat :" << *vertex_quat(index,0),*vertex_quat(index,1)
//                ,*vertex_quat(index,2),*vertex_pose()

        Eigen::Matrix3d rotation_matrix = the_quat.toRotationMatrix();


        Eigen::Vector3d offset(
                *vertex_pose(index, 0),
                *vertex_pose(index, 1),
                *vertex_pose(index, 2)
        );

        for (int ix(0); ix < 3; ++ix) {
            for (int iy(0); iy < 3; ++iy) {
                transform(ix, iy) = rotation_matrix(ix, iy);
            }
        }

        for (int ix(0); ix < 3; ++ix) {
            transform(ix, 3) = offset(ix);
        }

        /// add vertex

        auto *vertex = new g2o::VertexSE3();
        vertex->setId(index);
//        vertex->setEstimate
//        vertex->setEstimate(transform);
        if (index == 0) {
            vertex->setFixed(true);
        }
        globalOptimizer.addVertex(vertex);


        /// add edge
        if (index > 0) {

            auto *edge = new g2o::EdgeSE3();

            edge->vertices()[0] = globalOptimizer.vertex(index - 1);
            edge->vertices()[1] = globalOptimizer.vertex(index);

            Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();


            information(0, 0) = information(1, 1) = information(2, 2) = first_info;
            information(3, 3) = information(4, 4) = information(5, 5) = second_info;
            if (std::find(key_id.begin(), key_id.end(), index) != key_id.end()) {
                information(0, 0) = information(1, 1) = information(2, 2) = first_info / corner_ratio;
                information(3, 3) = information(4, 4) = information(5, 5) = second_info / corner_ratio;
            }
            edge->setInformation(information);

            edge->setInformation(information);

            edge->setMeasurement(last_t.inverse() * transform);
//            std::cout << " index : \n" << (transform).matrix() << std::endl;

            globalOptimizer.addEdge(edge);
        }

        // Z equal zero .
        if (index > 0) {
            auto *edge = new Z0Edge();
            edge->vertices()[0] = globalOptimizer.vertex(index - 1);
            edge->vertices()[1] = globalOptimizer.vertex(index);

            Eigen::Matrix<double, 1, 1> information;
            information(0, 0) = zero_info;
            edge->setInformation(information);

            edge->setMeasurement(0.0);
            globalOptimizer.addEdge(edge);
        }

        last_t = transform;


    }

    /// Add distance caonstraint
    for (int k(0); k < close_id.GetRows(); ++k) {
        auto distanceEdge = new DistanceEdge();

//        std::cout << "vertex id : " << int(*close_id(k, 0)) << " to " << int(*close_id(k, 1)) << std::endl;
        distanceEdge->vertices()[0] = globalOptimizer.vertex(int(*close_id(k, 0)));
        distanceEdge->vertices()[1] = globalOptimizer.vertex(int(*close_id(k, 1)));
        distanceEdge->setMeasurement(0.0);
        Eigen::Matrix<double, 1, 1> information;
        information(0, 0) = distance_info;

        distanceEdge->setInformation(information);

        globalOptimizer.addEdge(distanceEdge);
    }
    std::cout << "close id count:" << close_id.GetRows() << std::endl;

    /// Optimizer
    double start_optimize_time = TimeStamp::now();
//    globalOptimizer.initializeOptimization();
    globalOptimizer.initializeOptimization();
    globalOptimizer.optimize(2000);
    globalOptimizer.optimize(1000);
    globalOptimizer.optimize(1000);
    std::cout << " optimize waste time :" << TimeStamp::now() - start_optimize_time << std::endl;


    ///Save result
    std::ofstream trace_file("./TMP_DATA/save_trace.txt");

    std::vector<double> rx, ry, rz;
    int index = 0;
    while (true) {
        auto t_vertex = globalOptimizer.vertex(index);
        if (t_vertex > 0) {
            double data[10] = {0};
            t_vertex->getEstimateData(data);

            rx.push_back(data[0]);
            ry.push_back(data[1]);
            rz.push_back(data[2]);
            trace_file << data[0] << " " << data[1] << " " << data[2] << std::endl;
            index++;
        } else {
            break;
        }
//        delete t_vertex;
    }

    std::cout << "after compute vertex : " << rx.size() << std::endl;
    trace_file.close();

    matplotlibcpp::plot(rx, ry, "r-*");
    double sum_dis(0.0);
    for (int i(0); i < close_id.GetRows(); ++i) {
        std::vector<double> tx, ty;

//        tx.push_back(*close_id)
        for (int j(0); j < 2; ++j) {
            double tmp_data[10] = {0};
            auto *tv = globalOptimizer.vertex(*close_id(i, j));
            if (tv > 0) {
                tv->getEstimateData(tmp_data);
            }
            tx.push_back(tmp_data[0]);
            ty.push_back(tmp_data[1]);
        }
        matplotlibcpp::plot(tx, ty, "-b");
        sum_dis += std::sqrt(std::pow(tx[0] - tx[1], 2.0) + std::pow(ty[1] - ty[0], 2.0));


    }
    std::cout << "sum of distance is :" << sum_dis << " raw distance is :" << 1969.1 << std::endl;
    matplotlibcpp::show();
//    matplotlibcpp::save("/home/steve/Data/tmpimg" + std::to_string(first_info) + "-" + std::to_string(second_info) + "-"
//                        + std::to_string(distance_info) + ".jpg");
}