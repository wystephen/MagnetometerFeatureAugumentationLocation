//
// Created by steve on 17-4-17.
//
# include <iostream>
#include <thread>


//#include "Eigen/Dense"
//#include "Eigen/Geometry"
#include<Eigen/Dense>
#include <Eigen/Geometry>

#include "Extent/CSVReader.h"
#include "ZUPT/MYEKF.h"

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
    CSVReader raw_data("./TMP_DATA/all_data2.csv");
//    std::cin >> k;
    CSVReader zv_data("./TMP_DATA/zupt_result.csv");

//    std::cin >> k;
    CSVReader close_id("./TMP_DATA/close_vetices_num.csv");

//    std::cin >> k;

    if (zv_data.GetMatrix().GetRows() != raw_data.GetMatrix().GetRows()) {
        MYERROR("rows of raw data and zv_data is not equal");
    }

    Eigen::MatrixXd u(raw_data.GetMatrix().GetRows(), raw_data.GetMatrix().GetCols() - 1);

    std::cout << u.rows() << " : " << u.cols() << std::endl;
    std::cin >> k;

    for (int i(0); i < raw_data.GetMatrix().GetRows(); ++i) {
        for (int j(1); j < raw_data.GetMatrix().GetCols(); ++j) {
//            u(i, j - 1) = raw_data.GetMatrix()(i, j);
            std::cout << *raw_data.GetMatrix()(i, j) << std::endl;
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

    MyEkf myekf(settingPara);
    myekf.InitNavEq(u.block(0, 0, 40, 6));

    int vertex_id = 0;

    for (int index(0); index < u.rows(); ++index) {
        auto tmp = myekf.GetPosition(u.block(index, 0, 1, 6),
                                     *zv_data.GetMatrix()(index, 0));

        if (index == 0 ||
            (*zv_data.GetMatrix()(index - 1, 0) < 0.5 && *zv_data.GetMatrix()(index, 0) > 0.5)) {

            Eigen::MatrixXd P;
            Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
            Eigen::Isometry3d abs_transform = Eigen::Isometry3d::Identity();

            myekf.GetTransform(P, transform, abs_transform);

            auto *v = new g2o::VertexSE3();
            v->setId(vertex_id);
            v->setEstimate(abs_transform);
            if (index == 0) {
                v->setFixed(true);
            }
            globalOptimizer.addVertex(v);

            // add vertex
            if (index > 0) {
                auto *edge = new g2o::EdgeSE3();
                edge->vertices()[0] = globalOptimizer.vertex(index - 1);
                edge->vertices()[1] = globalOptimizer.vertex(index);

                std::cout << "P rows :" << P.rows() << " cols : " << P.cols() << std::endl;
            }


            vertex_id += 1;
        }
    }


    /// Add distance caonstraint



    /// Optimizer


    ///Save result

}