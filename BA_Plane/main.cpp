#include <iostream>
#include <fstream>
#include "vector"
#include <Eigen/Eigen>
#include "g2o/g2o/core/block_solver.h"
#include "g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/g2o/solvers/linear_solver_eigen.h"
#include "g2o/g2o/types/types_six_dof_expmap.h"
#include "g2o/g2o/core/robust_kernel_impl.h"
#include "g2o/g2o/solvers/linear_solver_dense.h"
#include "g2o/g2o/types/types_seven_dof_expmap.h"

using namespace std;

//Define the Plane vertex
class VertexPlane : public g2o::BaseVertex<4, Eigen::Vector4d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VertexPlane() {}

    virtual void setToOriginImpl() override {
        _estimate = Eigen::Vector4d(0, 0, 0, 0);
    };

    ///Todo is this the way of update plane?
    virtual void oplusImpl(const double *update) override {
        _estimate += Eigen::Vector4d(update[0], update[1], update[2], update[3]);
    }

    virtual bool read(istream &in) {}

    virtual bool write(ostream &out) const{}
};

//Define the Plane-3DPoint edge
class EdgePlanePoint : public g2o::BaseBinaryEdge<1, double , g2o::VertexSBAPointXYZ, VertexPlane> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual void computeError() override {
        auto vPoint = (g2o::VertexSBAPointXYZ *) _vertices[0];
        auto vPlane = (VertexPlane *) _vertices[1];
        Eigen::Vector4d planeABCD = vPlane->estimate();
        Eigen::Vector3d pointXYZ = vPoint->estimate();
        double devided = abs(
                planeABCD[0] * pointXYZ[0] + planeABCD[1] * pointXYZ[1] + planeABCD[2] * pointXYZ[2] + planeABCD[3]);
        double devidor = sqrt(planeABCD[0] * planeABCD[0] + planeABCD[1] * planeABCD[1] + planeABCD[2] * planeABCD[2]);
        Eigen::Matrix<double, 1, 1> myError;
        myError << (devided / devidor);
        _error = myError;
    }

    virtual bool read(istream &in) {}

    virtual bool write(ostream &out) const {}
};

////Define the Plane-3DPoint edge
//class EdgePlanePoint : public g2o::BaseBinaryEdge<1, double, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap> {
//public:
//    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
//
//    virtual void computeError() override {
//        auto vPoint = (g2o::VertexSBAPointXYZ *) _vertices[0];
//        auto vPose = (g2o::VertexSE3Expmap *) _vertices[1];
//        Eigen::Vector4d obserPlane = _measurement;
//        Eigen::Vector3d pointXYZ = vPoint->estimate();
//        Eigen::Vector3d PointW = vPose->estimate().map(vPoint->estimate());
//        double devided = abs(
//                obserPlane[0] * PointW[0] + obserPlane[1] * PointW[1] + obserPlane[2] * PointW[2] + obserPlane[3]);
//        double devidor = sqrt(
//                obserPlane[0] * obserPlane[0] + obserPlane[1] * obserPlane[1] + obserPlane[2] * obserPlane[2]);
//        Eigen::Matrix<double, 1, 1> myError;
//        myError << (devided / devidor);
//        _error = myError;
//    }
//
//    virtual bool read(istream &in) {}
//
//    virtual bool write(ostream &out) const {}
//};

int main() {
    std::cout << "Hello, World!" << std::endl;
    ///Load data
    ifstream reader;
    reader.open("data/3Dpoints.txt", ios::in);
    vector<std::vector<double>> point3D;
    vector<int> point2Plane;
    while (!reader.eof()) {
        vector<double> thisPoint;
        double x, y, z, ID;
        reader >> x >> y >> z >> ID;
        thisPoint.push_back(x);
        thisPoint.push_back(y);
        thisPoint.push_back(z);
        point2Plane.push_back(ID);
        point3D.push_back(thisPoint);
    }
    reader.close();
    int index = 15;
    cout << "3D point " << point3D[index][0] << " " << point3D[index][1] << " " << point3D[index][2] << endl;
    cout << "close to plane " << point2Plane[index] << endl;
    cout << "load " << point3D.size() << " 3D points" << " with " << point2Plane.size() << " to plane labels" << endl;
    reader.open("data/observations.txt", ios::in);
    vector<std::vector<double>> point2D;
    vector<pair<int, int>> PointFrameIndex;
    while (!reader.eof()) {
        vector<double> thisPoint;
        double x, y, indexP, KF;
        reader >> x >> y >> indexP >> KF;
        thisPoint.push_back(x);
        thisPoint.push_back(y);
        point2D.push_back(thisPoint);
        PointFrameIndex.push_back(pair<int, int>(indexP, KF));
    }
    reader.close();
    cout << "2D observation " << point2D[index][0] << " " << point2D[index][1] << endl;
    cout << "is related to 3D point " << PointFrameIndex[index].first << " of frame " << PointFrameIndex[index].second
         << endl;
    cout << "load " << point2D.size() << " 2d observations " << endl;
    reader.open("data/poses.txt");
    Eigen::Matrix4d pose1;
    Eigen::Matrix4d pose2;
    reader >> pose1(0, 0) >> pose1(0, 1) >> pose1(0, 2) >> pose1(0, 3);
    reader >> pose1(1, 0) >> pose1(1, 1) >> pose1(1, 2) >> pose1(1, 3);
    reader >> pose1(2, 0) >> pose1(2, 1) >> pose1(2, 2) >> pose1(2, 3);
    reader >> pose1(3, 0) >> pose1(3, 1) >> pose1(3, 2) >> pose1(3, 3);
    reader >> pose2(0, 0) >> pose2(0, 1) >> pose2(0, 2) >> pose2(0, 3);
    reader >> pose2(1, 0) >> pose2(1, 1) >> pose2(1, 2) >> pose2(1, 3);
    reader >> pose2(2, 0) >> pose2(2, 1) >> pose2(2, 2) >> pose2(2, 3);
    reader >> pose2(3, 0) >> pose2(3, 1) >> pose2(3, 2) >> pose2(3, 3);
    reader.close();
    cout << "pose 1" << endl << pose1 << endl;
    cout << "pose 2" << endl << pose2 << endl;
    reader.open("data/planes.txt");
    vector<std::vector<double>> planes;
    while (!reader.eof()) {
        double A, B, C, D;
        reader >> A >> B >> C >> D;
        vector<double> thisPlane;
        thisPlane.push_back(A);
        thisPlane.push_back(B);
        thisPlane.push_back(C);
        thisPlane.push_back(D);
        planes.push_back(thisPlane);
    }
    reader.close();
    cout << "plane A B C D " << planes[2][0] << " " << planes[2][1] << " " << planes[2][2] << " " << planes[2][3]
         << endl;
    cout << "load " << planes.size() << " planes" << endl;

    ///Start G2O

    ///declar optimizer and solver
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType *lineaSolver;
    lineaSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
    g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(lineaSolver);
    //declare optimization algorithm
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);
    ///Set KeyFrame Vertices
    g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
    Eigen::Matrix<double, 3, 3> R;
    R << pose1(0, 0), pose1(0, 1), pose1(0, 2),
            pose1(1, 0), pose1(1, 1), pose1(1, 2),
            pose1(2, 0), pose1(2, 1), pose1(2, 2);
    Eigen::Matrix<double, 3, 1> T;
    T << pose1(0, 3), pose1(1, 3), pose1(2, 3);
    g2o::SE3Quat kfpose1(R, T);
    vSE3->setEstimate(kfpose1);
    vSE3->setId(0);
    vSE3->setFixed(true);
    optimizer.addVertex(vSE3);
    g2o::VertexSE3Expmap *vSE3_2 = new g2o::VertexSE3Expmap();
    Eigen::Matrix<double, 3, 3> R2;
    R2 << pose2(0, 0), pose2(0, 1), pose2(0, 2),
            pose2(1, 0), pose2(1, 1), pose2(1, 2),
            pose2(2, 0), pose2(2, 1), pose2(2, 2);
    Eigen::Matrix<double, 3, 1> T2;
    T2 << pose1(0, 3), pose1(1, 3), pose1(2, 3);
    g2o::SE3Quat kfpose2(R2, T2);
    vSE3_2->setEstimate(kfpose2);
    vSE3_2->setId(1);
    optimizer.addVertex(vSE3_2);
    int maxKFID = 1;
    int maxMapVertex = 0;
    ///Set Mappoint vertexs
    for (int i = 0; i < point3D.size() - 1; i++) {
        g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
        Eigen::Matrix<double, 3, 1> v;
        v << point3D[i][0], point3D[i][1], point3D[i][2];
        vPoint->setEstimate(v);
        vPoint->setId(maxKFID + i + 1);
        cout<<"point i "<<i<<" set vertex id "<<maxKFID+i+1<<endl;
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);
        maxMapVertex++;
    }
    ///Set Observation Edges
    int edgeNum = 0;
    for (int i = 0; i < point2D.size() - 1; i++) {
        int pointIndex = PointFrameIndex[i].first;
        int FrameIndex = PointFrameIndex[i].second;
        int pointVertexID = pointIndex + maxKFID + 1;
        cout<<"obser i "<<i<<" point index "<<pointIndex<<" VertexID "<<pointVertexID<<" frameindex "<<FrameIndex<<endl;
        Eigen::Matrix<double, 2, 1> obs;
        obs << point2D[i][0], point2D[i][1];
        g2o::EdgeSE3ProjectXYZ *e = new g2o::EdgeSE3ProjectXYZ();
        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pointIndex)));
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(FrameIndex)));
        e->setMeasurement(obs);
        e->setInformation(Eigen::Matrix2d::Identity());
        e->fx = 481.20;
        e->fy = -480.00;
        e->cx = 319.50;
        e->cy = 239.50;
        optimizer.addEdge(e);
        edgeNum++;
    }
    int maxObserEdge = edgeNum;
    ///Set Plane Vertex
    int maxPlaneNum = 0;
    for (int i = 0; i<planes.size(); i++) {
        VertexPlane *newPlane = new VertexPlane();
        newPlane->setId(maxKFID + maxMapVertex + i + 1);
        newPlane->setEstimate(Eigen::Vector4d(planes[i][0], planes[i][1], planes[i][2], planes[i][3]));
        newPlane->setFixed(true);//TODO we do not optimize Plane. shall we?
        optimizer.addVertex(newPlane);
        maxPlaneNum++;
    }
    ///Set PlanePoint Edges
    for (int i = 0; i < point2Plane.size(); i++) {
        if (point2Plane[i] > 0) {
            int pointID = i;
            int planeID = point2Plane[i];
            EdgePlanePoint * newEdge = new EdgePlanePoint;
            newEdge->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pointID+1)));
            newEdge->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(maxKFID+maxMapVertex+planeID)));
            newEdge->setMeasurement(0);//todo measurement?
            newEdge->setInformation(Eigen::Matrix<double,1,1>::Identity());
            newEdge->setRobustKernel(new g2o::RobustKernelHuber());
            optimizer.addEdge(newEdge);
        }
    }

    ///well, i did not remove the 3d point without observation
    optimizer.initializeOptimization();
    optimizer.optimize(1000);
    g2o::VertexSE3Expmap *vSE3_after = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(0));
    cout << "pose1" << endl << vSE3_after->estimate();
    g2o::VertexSE3Expmap *vSE3_after_2 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(1));
    cout << "pose2" << endl << vSE3_after_2->estimate();
    for (int i = 0; i < point3D.size() - 1; i++) {
        //cout<<"read vertex id "<<maxKFID+i+1<<endl;
        g2o::VertexSBAPointXYZ *vPoint_after = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(maxKFID + i + 1));
        cout << vPoint_after->estimate()[0] << " " << vPoint_after->estimate()[1] << " " << vPoint_after->estimate()[2]
             << endl;
    }
    return 0;
}
