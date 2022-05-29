/*
 * This project is modifed from SLAMbook2 bundle_adjustment project
 */

//#include <Eigen/StdVector>
#include <opencv2/core/core.hpp>
#include <iostream>
//#include <fstream>
//#include "sophus/se3.hpp"
#include "Converter.h"
//#include <g2o/core/base_vertex.h>
//#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>


using namespace std;
using namespace Eigen;
//using namespace Sophus;

struct point {
    double x, y, z;
    int id;
    int observedFrameID;
};
struct mapPoint {
    double x, y, z;
    int id;
};

const double thHuberStereo = sqrt(7.815);
const float fx = 535.4;
const float fy = 539.2;
const float cx = 320.1;
const float cy = 247.6;
const float mbf = 40.0;

int main() {
    std::cout << "Hello, World!" << std::endl;

    ///Read data--------------------------------------------------------------------------------------
    ///Read observed points' XYZ, point ID and observed Frame Id.
    ifstream pointReader, mapPointReader;
    pointReader.open("../localBundlePoints.txt");
    string line;
    int count = 0;
    vector<point> points;
    while (getline(pointReader, line)) {
        stringstream ss(line);
        point p;
        if (count % 5 == 0)
            ss >> p.x;
        else if (count % 5 == 1)
            ss >> p.y;
        else if (count % 5 == 2)
            ss >> p.z;
        else if (count % 5 == 3)
            ss >> p.id;
        else if (count % 5 == 4)
            ss >> p.observedFrameID;
        if (count % 5 == 4) {
            //cout<<p.x<<" "<<p.y<<" "<<p.z<<" "<<p.id<<" "<<p.observedFrameID<<endl;
            points.push_back(p);
        }
        count++;
    }
    cout << "load " << points.size() << " observed points" << endl;
    pointReader.close();

    ///Read Map Points XYZ and its point ID
    mapPointReader.open("../localBundleMapPoints.txt");
    string line2;
    int count2 = 0;
    vector<mapPoint> mapPoints;
    while (getline(mapPointReader, line2)) {
        stringstream ss(line2);
        mapPoint p;
        if (count2 % 4 == 0)
            ss >> p.x;
        else if (count2 % 4 == 1)
            ss >> p.y;
        else if (count2 % 4 == 2)
            ss >> p.z;
        else if (count2 % 4 == 3)
            ss >> p.id;
        if (count2 % 4 == 3) {
            //cout << p.x << " " << p.y << " " << p.z << " " << p.id << " " << endl;
            mapPoints.push_back(p);
        }
        count2++;
    }
    cout << "Load " << mapPoints.size() << " mappoints " << endl;
    mapPointReader.close();

    ///Load camera poses at Frame 1,2 and 3
    cv::Mat pose2 = (cv::Mat_<float>(4, 4) << 0.99241, 0.0144781, -0.122115, 0.202981,
            -0.0136932, 0.99988, 0.00726406, -0.0262754,
            0.122205, -0.00553679, 0.992489, 0.0460141,
            0, 0, 0, 1);
    cv::Mat pose1 = (cv::Mat_<float>(4, 4) << 0.991981, 0.0239371, -0.124101, 0.201774,
            -0.0225897, 0.99967, 0.0122531, -0.0314431,
            0.124353, -0.00935147, 0.992194, 0.0487706,
            0, 0, 0, 1);
    cv::Mat pose0 = (cv::Mat_<float>(4, 4) << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1);

    ///Set Up G2O--------------------------------------------------------------------------------------
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> DefBlockSolverType;
    //typedef g2o::LinearSolverCSparse<DefBlockSolverType::PoseMatrixType> DefLinearSolverType;
    typedef g2o::LinearSolverCholmod<DefBlockSolverType::PoseMatrixType> DefLinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<DefBlockSolverType>(g2o::make_unique<DefLinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);

//    g2o::BlockSolver_6_3::LinearSolverType *linearSolver;
//    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
//    g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(
//            std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType>(linearSolver));
//    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(
//            std::unique_ptr<g2o::BlockSolver_6_3>(solver_ptr));
//    g2o::SparseOptimizer optimizer;
//    optimizer.setAlgorithm(solver);
//    optimizer.setVerbose(false);

    int maxVertexID = 0;
    //Add vertex for pose
//    vector<g2o::VertexSE3Expmap *> vertexs_poses;
//    g2o::VertexSE3Expmap* v_se3 = new g2o::VertexSE3Expmap();
    //g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
//    vSE3->setEstimate(ORB_SLAM2::Converter::toSE3Quat(pose0));
//    vSE3->setId(0);
//    vSE3->setFixed(true);
//    optimizer.addVertex(vSE3);
//    for(int i=0;i<3;i++){
//        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
//        if (i == 0)
//            vSE3->setEstimate(ORB_SLAM2::Converter::toSE3Quat(pose0));
//        if (i == 1)
//            vSE3->setEstimate(ORB_SLAM2::Converter::toSE3Quat(pose1));
//        if (i == 2)
//            vSE3->setEstimate(ORB_SLAM2::Converter::toSE3Quat(pose2));
//        vSE3->setId(i);
//        if (i == 0)
//            vSE3->setFixed(true);
//        optimizer.addVertex(vSE3);
//        vertexs_poses.push_back(vSE3);
//        cout << "add vertex " << i << " with pose " << endl << vSE3->estimate() << endl;
//        if (i >maxVertexID)
//            maxVertexID = i;
//    }
//    //Add vertex for mappoints
//    vector<g2o::VertexSBAPointXYZ *> vertexs_mappoints;
//    for(size_t i = 0; i < mapPoints.size(); i++){
//        cv::Mat mapPointXYZ = (cv::Mat_<float>(3, 1) << mapPoints[i].x, mapPoints[i].y, mapPoints[i].z);
//        Eigen::Matrix<double, 3, 1> pointXYZ;
//        pointXYZ << mapPoints[i].x, mapPoints[i].y, mapPoints[i].z;
//        g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
//        vPoint->setEstimate(ORB_SLAM2::Converter::toVector3d(mapPointXYZ));
//        //vPoint->setEstimate(pointXYZ);
//        int id = mapPoints[i].id + maxVertexID; ///G2O ID
//        vPoint->setId(id);
//        //Linearsolvertype 所以可以边缘化
//        vPoint->setMarginalized(true);
//        if (mapPoints[i].id < 10) { ///Just show first 10 mappoints
//            cout << "add mappoint vertex " << id << " (point id " << mapPoints[i].id << ") with"
//                 << vPoint->estimate().transpose() << endl;
//        }
//        optimizer.addVertex(vPoint);
//        vertexs_mappoints.push_back(vPoint);
//    }
//
//    ///load observation edges
//    int edgeNum = 0;
//    for (int i = 0; i < points.size(); i++) {
//        //cout << points[i].x << " " << points[i].y << " " << points[i].z << " " << points[i].id << " "<< points[i].observedFrameID << endl;
//        Eigen::Matrix<double, 3, 1> obs;
//        obs << points[i].x, points[i].y, points[i].z;
//        g2o::EdgeStereoSE3ProjectXYZ *e = new g2o::EdgeStereoSE3ProjectXYZ();
//        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(points[i].id + maxVertexID)));
//        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(points[i].observedFrameID)));
//        cout << "add edge with points vertex ID " << points[i].id + maxVertexID << " and frame id "
//             << points[i].observedFrameID << endl;
//        e->setMeasurement(obs);
//        cout << "measurement " << e->measurement().transpose() << endl;
//        Eigen::Matrix3d Info = Eigen::Matrix3d::Identity();
//        e->setInformation(Info);
//        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
//        e->setRobustKernel(rk);
//        rk->setDelta(thHuberStereo);
//        e->fx = fx;
//        e->fy = fy;
//        e->cx = cx;
//        e->cy = cy;
//        e->bf = mbf;
//        optimizer.addEdge(e);
//
//        if (points[i].id < 10) {///Just showed first 10 edges
//            cout << "add new edge " << e->measurement().transpose() << " linked vertex0: "
//                 <<points[i].id + maxVertexID  << " vertex1: " << points[i].observedFrameID << endl;
//        }
//        edgeNum++;
//    }
//    cout << "store edge num " << edgeNum << endl;

    //*Step 9 开始优化
    optimizer.initializeOptimization();
    //先优化五次
    optimizer.optimize(5);
    cout<<"g2o done"<<endl;

cout<<"system return"<<endl;
    return 0;
}
