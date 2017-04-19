//
// Created by steve on 17-4-19.
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
    double first_info(1000), second_info(1000), distance_info(0.001);


    /// parameters
    std::cout << "para num :" << argc << std::endl;
    if (argc == 4) {
        first_info = std::stod(argv[1]);
        second_info = std::stod(argv[2]);
        distance_info = std::stod(argv[3]);

        std::cout << "first info :" << first_info
                  << "second info :" << second_info
                  << "distance info :" << distance_info
                  << std::endl;

    }

    /// Load Data
    CSVReader vertex_pose_file("./TMP_DATA/vertex_pose.csv");
    CSVReader vertex_quat_file("./TMP_DATA/vertex_quat.csv");
    CSVReader close_id_file("./TMP_DATA/close_vetices_num.csv");

    auto vertex_pose(vertex_pose_file.GetMatrix());
    auto vertex_quat(vertex_quat_file.GetMatrix());
    auto close_id(close_id_file.GetMatrix());


    /// Initial graph
    g2o::SparseOptimizer globalOptimizer;


    typedef g2o::BlockSolver_6_3 SlamBlockSolver;
    typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

    /// Initial solver
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

        }
    }







    /**
     *
     */
    /// Add distance constraint
    for (int k(0); k < close_id.GetRows(); ++k) {
        auto distanceEdge = new DistanceEdge();

        std::cout << "vertex id : " << int(*close_id(k, 0)) << " to " << int(*close_id(k, 1)) << std::endl;
        distanceEdge->vertices()[0] = globalOptimizer.vertex(int(*close_id(k, 0)));
        distanceEdge->vertices()[1] = globalOptimizer.vertex(int(*close_id(k, 1)));
        distanceEdge->setMeasurement(0.0);
        Eigen::Matrix<double, 1, 1> information;
        information(0, 0) = distance_info;

        distanceEdge->setInformation(information);

        globalOptimizer.addEdge(distanceEdge);
    }

    /// Optimizer
    double start_optimize_time = TimeStamp::now();
//    globalOptimizer.initializeOptimization();
    globalOptimizer.initializeOptimization();
    globalOptimizer.optimize(2000);
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
    matplotlibcpp::show();

}

