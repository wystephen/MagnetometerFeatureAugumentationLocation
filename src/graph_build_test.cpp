//
// Created by steve on 17-4-17.
//
# include <iostream>
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
    /// Load Data
    int k(0);
    CSVReader raw_data_file("./TMP_DATA/preprocess_data.csv");
//    std::cin >> k;
    CSVReader zv_data_file("./TMP_DATA/zupt_result.csv");

//    std::cin >> k;
    CSVReader close_id_file("./TMP_DATA/close_vetices_num.csv");


    auto raw_data(raw_data_file.GetMatrix());
    auto zv_data(zv_data_file.GetMatrix());
    auto close_id(close_id_file.GetMatrix());

//    std::cin >> k;

    if (zv_data.GetRows() != raw_data.GetRows()) {
        MYERROR("rows of raw data and zv_data is not equal");
    }

    Eigen::MatrixXd u(raw_data.GetRows(), raw_data.GetCols() - 1);
    u.setZero();

    std::cout << u.rows() << " : " << u.cols() << std::endl;
//    std::cin >> k;

    for (int i(0); i < u.rows(); ++i) {
        for (int j(1); j < u.cols() - 1; ++j) {
            u(i, j - 1) = *raw_data(i, j);
//            std::cout << *raw_data.GetMatrix()(i, j) << std::endl;
        }
    }


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

    /// Initial EKF

    SettingPara settingPara(true);
    settingPara.Ts_ = 1.0 / 200.0;

    settingPara.sigma_acc_ *= 6.0;
    settingPara.sigma_gyro_ *= 6.0;

    Ekf myekf(settingPara);
    myekf.InitNavEq(u.block(0, 0, 40, 6));

    int vertex_id = 0;

    for (int index(0); index < u.rows(); ++index) {
//        std::cout << "before GetPosition" << std::endl;
        if (index > 0) {
            settingPara.Ts_ = (*raw_data(index, 0) - *raw_data(index - 1, 0));
        }
        auto tmp = myekf.GetPosition(u.block(index, 0, 1, 6).transpose(),
                                     *zv_data(index, 0));

//        std::cout << "after GetPosition" << std::endl;

        if (vertex_id == 337) {
//            break;
        }
        if (index == 0 ||
            (*zv_data(index - 1, 0) < 0.5 && *zv_data(index, 0) > 0.5)) {

            Eigen::MatrixXd P(9, 9);
            P.setZero();
            Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
            Eigen::Isometry3d abs_transform = Eigen::Isometry3d::Identity();
//            std::cout << "before Get Transform" << std::endl;

            std::cout << "index :" << index << "vertex id : " << vertex_id << std::endl;
            myekf.GetTransform(P, transform, abs_transform);
//            std::cout << "after Get Transform" << std::endl;

            auto *v = new g2o::VertexSE3();
            v->setId(vertex_id);
//            v->setEstimate(abs_transform);
            if (vertex_id == 0) {
                v->setFixed(true);
            }
            globalOptimizer.addVertex(v);

            // add vertex
            if (vertex_id > 0) {
                auto *edge = new g2o::EdgeSE3();
                edge->vertices()[0] = globalOptimizer.vertex(vertex_id - 1);
                edge->vertices()[1] = globalOptimizer.vertex(vertex_id);

//                std::cout << "P rows :" << P.rows() << " cols : " << P.cols() << std::endl;

                Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
                information(0, 0) = information(1, 1) = information(2, 2) = 50;
                information(3, 3) = information(4, 4) = information(5, 5) = 100;
                edge->setInformation(information);

                edge->setMeasurement(transform);
//                edge->setMeasurementFromState();
//                edge->set
                globalOptimizer.addEdge(edge);

            }


            vertex_id += 1;
        }
    }


    /// Add distance caonstraint
    for (int k(0); k < close_id.GetRows(); ++k) {
        auto edge = new DistanceEdge();

        edge->setMeasurement(0.0);
        Eigen::Matrix<double, 1, 1> information;
        information(0, 0) = 1.0 / 50.0;

//        if(*close_id(k,0)>337||*close_id(k,1)>337)
//        {
//            continue;
//        }

        edge->setInformation(information);
        edge->vertices()[0] = globalOptimizer.vertex(int(*close_id(k, 0)));
        edge->vertices()[1] = globalOptimizer.vertex(int(*close_id(k, 1)));

        globalOptimizer.addEdge(edge);
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