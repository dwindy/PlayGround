#include <iostream>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <opencv2/core/core.hpp>
#include <Eigen/Dense>

using namespace std;

int main() {
    std::cout << "Hello, World!" << std::endl;

    //with FAKE data first
    cv::Mat pose0 = (cv::Mat_<float>(4, 4) << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1);

    ///Init the G2O parameters
    g2o::SparseOptimizer myoptimizer;
    g2o::BlockSolver_6_3::LinearSolverType *mylinearSolver;
    mylinearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
    g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(mylinearSolver);

    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    myoptimizer.setAlgorithm(solver);
    myoptimizer.setVerbose(false);

    ///Add Data for camera pose Vertex
    int maxVertexID = 0;

    //matrix 2 quat
    Eigen::Matrix<double, 3, 3> R;
    R << pose0.at<float>(0, 0), pose0.at<float>(0, 1), pose0.at<float>(0, 2),
            pose0.at<float>(1, 0), pose0.at<float>(1, 1), pose0.at<float>(1, 2),
            pose0.at<float>(2, 0), pose0.at<float>(2, 1), pose0.at<float>(2, 2);
    Eigen::Matrix<double, 3, 1> t(pose0.at<float>(0, 3), pose0.at<float>(1, 3), pose0.at<float>(2, 3));
    g2o::SE3Quat pose0Quat = g2o::SE3Quat(R, t);

    g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate(pose0Quat);
    vSE3->setId(maxVertexID);
    myoptimizer.addVertex(vSE3);
    maxVertexID++;
    std::cout<<"pose vertex added, maxVertexId "<<maxVertexID<<endl;
    ///Add data for mappoint vertex
    int maxMapPointNum = 10;
    for (int i = 0; i < maxMapPointNum; i++) {
        Eigen::Matrix<double,3,1> location;
        location<<i,i,i;
        g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(location);
        vPoint->setId(i+maxVertexID);
        vPoint->setMarginalized(true);
        myoptimizer.addVertex(vPoint);
    }
    maxVertexID +=maxMapPointNum;
    std::cout<<"mappoint vertex added, maxVertexId "<<maxVertexID<<endl;

    ///Add observation edge
    int maxEdgeNum = 10;
    for(int i =0; i < maxEdgeNum;i++){
        Eigen::Matrix<double,3,1> obs;
        obs<<i+100,i+100,i+100;
//        g2o::EdgeSt
    }

    return 0;
}
