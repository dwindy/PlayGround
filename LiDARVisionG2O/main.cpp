#include <iostream>
#include <Eigen/StdVector>
#include "Converter.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include <fstream>
#include "sophus/se3.h"
#include "sophus/so3.h"

/**
 * This demo is designed to run a United Opimization of both LiDAR and Vision G2O
 */

using namespace std;
using namespace Eigen;
using namespace Sophus;

struct point{
    double x,y,z;
    int id;
    int observedFrameID;
};

struct mapPoint{
    double x,y,z;
    int id;
};

const float thHuberStereo = sqrt(7.815);
///to do modify
const float fx = 700;
const float fy = 700;
const float cx = 500;
const float cy = 500;
const float mbf = 40.0;

///LiDAR edge
class VertexLiDARPoint : public g2o::BaseVertex<3, Eigen::Vector3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual bool read(std::istream &is);

    virtual bool write(std::ostream &os) const;

    virtual void setToOriginImpl() {
        _estimate.fill(0.);
    }

    virtual void oplusImpl(const double *update) {
        Eigen::Map<const Vector3d> v(update);
        _estimate += v;
    }
};

class VertexVisionPoint : public g2o::BaseVertex<3, Eigen::Vector3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual bool read(std::istream &is);

    virtual bool write(std::ostream &os) const;

    virtual void setToOriginImpl() {
        _estimate.fill(0.);
    }

    virtual void oplusImpl(const double *update) {
        Eigen::Map<const Vector3d> v(update);
        _estimate += v;
    }
};

///struct for G2O vertex
//First 3 SO3, then translation, then focal xy then centre xy then distortion 1 2;
struct PoseAndIntrinsics{
    PoseAndIntrinsics(){}
    ///set from given data address
    explicit PoseAndIntrinsics(double *data_addr){
        rotation = SO3::exp(Vector3d(data_addr[0],data_addr[1],data_addr[2]));
        translation = Vector3d(data_addr[3],data_addr[4],data_addr[5]);
        fx = data_addr[6]; fy = data_addr[7];
        cx = data_addr[8]; cy = data_addr[9];
        k1 = data_addr[10]; k2 = data_addr[11];
    }
    ///set from estimate
    void set_to(double *data_addr){
        auto r =rotation.log();
        for(int i=0; i<3; ++i) data_addr[i] = r[i];
        for(int i=0; i<3; ++i) data_addr[i+3] = translation[i];
        data_addr[6] = fx; data_addr[7] = fy;
        data_addr[8] = cx; data_addr[9] = cy;
        data_addr[10] = k1; data_addr[11] = k2;
    }
    SO3 rotation;
    Vector3d translation = Vector3d::Zero();
    double fx = 0, fy = 0, cx = 0, cy = 0;
    double k1 = 0, k2 = 0;
};

class VertexPoseAndIntrinsics : public g2o::BaseVertex<12, PoseAndIntrinsics> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VertexPoseAndIntrinsics() {}

    virtual void setToOriginImpl() override {
        _estimate = PoseAndIntrinsics();
    };

    virtual void oplusImpl(const double *update) override{
        _estimate.rotation = SO3::exp(Vector3d(update[0],update[1],update[2]))*_estimate.rotation;
        _estimate.translation += Vector3d(update[3],update[4],update[5]);
        _estimate.fx += update[6];_estimate.fy += update[7];
        _estimate.cx += update[8];_estimate.cy += update[9];
        _estimate.k1 += update[10];_estimate.k1 += update[11];
    }

    ///vision feature project function
    Vector2d visionProject(const Vector3d &point) {
        Vector3d pc = _estimate.rotation * point + _estimate.translation;
        pc = -pc / pc[2]; ///Minus?
        double r2 = pc.squaredNorm();
        double distortion = 1.0 + r2 * (_estimate.k1 + _estimate.k2 * r2);
        return Vector2d(_estimate.fx * distortion * pc[0],
                        _estimate.fy * distortion * pc[1]);
    }

    ///lidar feature project function
    float lidarEdgeProject(const Vector3d &point){
        Vector3d pc = _estimate.rotation * point + _estimate.translation;

    }


    virtual bool read(istream &in){}
    virtual bool write(ostream &out) const {}
};

int main() {
    std::cout << "Hello, World!" << std::endl;

    ///Read data
    ifstream viPointReader, viMapPointReader, liPointReader, liMapPointReader;
    viPointReader.open("../visionPoints.txt");
    string line;
    int count = 0;
    vector<point> viPoints;
    while (getline(viPointReader, line)) {
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
        if (count % 5 == 4)
            viPoints.push_back(p);
        count++;
    }
    viPointReader.close();

    viMapPointReader.open("../visionMapPoints.txt");
    line;
    count = 0;
    vector<point> viMapPoints;
    while (getline(viMapPointReader, line)) {
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
        if (count % 5 == 4)
            viMapPoints.push_back(p);
        count++;
    }
    viMapPointReader.close();

    liPointReader.open("../lidarPoints.txt");
    line;
    count = 0;
    vector<point> liPoints;
    while (getline(liPointReader, line)) {
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
        if (count % 5 == 4)
            liPoints.push_back(p);
        count++;
    }
    liPointReader.close();

    liMapPointReader.open("../lidarMapPoints.txt");
    line;
    count = 0;
    vector<point> liMapPoints;
    while (getline(liMapPointReader, line)) {
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
        if (count % 5 == 4)
            liMapPoints.push_back(p);
        count++;
    }
    liMapPointReader.close();

    ///Set Up G2O
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType  *linearSolver;
    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
    g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);
    return 0;
}
