/*
 * read pointcloud stored by ORBSLAM2
 * iteratively fit plane and filter noise points
 */

#include <iostream>
#include <opencv2/core/core.hpp>
#include <pangolin/pangolin.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class PtRGBD {
public:
    cv::Point2d pt2d;
    cv::Point3d pt3d;
    int index2d;
    int index3d;
    int response;
};

class Plane{
public:
    double A,B,C,D;
    int PlaneId;
    Eigen::Vector3d PI;
    vector<PtRGBD> planePts;
    Plane(double Ain, double Bin, double Cin, double Din) : A(Ain), B(Bin), C(Cin), D(Din) { PlaneId = -1; }
    Plane() { PlaneId = -1;}
    void NormD2CP();
};

void Plane::NormD2CP() {
    if (this->D < 0) {
        this->A = -this->A;
        this->B = -this->B;
        this->C = -this->C;
        this->D = -this->D;
    }
    PI[0] = this->A * -this->D;
    PI[1] = this->B * -this->D;
    PI[2] = this->C * -this->D;
}

/*
 * input PointCloud.
 * output Plane and inlier index
 */
int RANSACPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, Plane &foundPlane, pcl::PointIndices &inliersOutput){
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    //create segmentation objects
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    //optional
    seg.setOptimizeCoefficients(true);
    //mandatory
    seg.setMethodType(pcl::SACMODEL_PLANE);
    seg.setModelType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.1);

    seg.setInputCloud(cloud);
    seg.segment(*inliers,*coefficients);
    if(inliers->indices.size()==0)
        return 0;
    inliersOutput = *inliers;
    foundPlane.A = coefficients->values[0];
    foundPlane.B = coefficients->values[1];
    foundPlane.C = coefficients->values[2];
    foundPlane.D = coefficients->values[3];
    for (int i = 0; i < inliers->indices.size(); i++) {
        double x = cloud->points[inliers->indices[i]].x;
        double y = cloud->points[inliers->indices[i]].y;
        double z = cloud->points[inliers->indices[i]].z;
        cv::Point3d newP = cv::Point3d(x, y, z);
        PtRGBD newPtRGBD;
        newPtRGBD.pt3d = newP;
        foundPlane.NormD2CP();
        foundPlane.planePts.push_back(newPtRGBD);
    }
    //cout<<"stored "<<foundPlane.planePts.size()<<" points in the plane, return "<<inliers->indices.size()<<endl;

    return inliers->indices.size();
}

double distance2Plane(pcl::PointXYZ point, Plane plane) {
    double upper = abs(plane.A * point.x + plane.B * point.y + plane.C * point.z + plane.D);
    double down = sqrt(plane.A * plane.A + plane.B * plane.B + plane.C * plane.C);
    return upper/down;
}

//vector<Plane> mergePlanes(vector<Plane> inputs) {
//    for (int i = 0; i < inputs.size(); i++) {
//        Plane thisOne = inputs[i];
//        for (int j = 0; j < inputs.size(); j++) {
//        }
//    }
//}

void showPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr toShowCLoud)
{
            ///show---plane points
        pangolin::CreateWindowAndBind("Main", 640, 480);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 320, 0.2, 100),
                pangolin::ModelViewLookAt(2, 0, 2, 0, 0, 0, pangolin::AxisY)
        );

        pangolin::Handler3D handler(s_cam);
        pangolin::View &d_cam = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f / 480.0f)
                .SetHandler(&handler);
        while (!pangolin::ShouldQuit()) {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            d_cam.Activate(s_cam);

            //需要绘制的东西写在这里
            //coordinate frame
            glBegin(GL_LINES);
            glLineWidth(5.0);
            glColor3f(1, 0, 0);
            glVertex3d(0.1, 0, 0); //X axis Red
            glVertex3d(0, 0, 0);
            glColor3f(0, 1, 0);
            glVertex3d(0, 0.1, 0); //Y axis Green
            glVertex3d(0, 0, 0);
            glColor3f(0, 0, 1);
            glVertex3d(0, 0, 0.1); //Z axis Blue
            glVertex3d(0, 0, 0);
            glEnd();

            //points
            glBegin(GL_POINTS);
            for (int ci = 0; ci < toShowCLoud->points.size(); ci++) {
                glColor3f(1, 1, 1);
                glPointSize(30);
                glVertex3d(toShowCLoud->points[ci].x, toShowCLoud->points[ci].y, toShowCLoud->points[ci].z);
            }
            glEnd();

//            //points
//            glBegin(GL_POINTS);
//            for (int ci = 0; ci < planes.size(); ci++) {
//                //    for (int ci = 0; ci < 1; ci++) {
//                //cout << "plane " << ci << " of total " << pointsOfEachPlane.size() << endl;
//                switch (ci) {
//                    case 0 :
//                        glColor3f(1, 0, 0);
//                        break;
//                    case 1 :
//                        glColor3f(0, 1, 0);
//                        break;
//                    case 2 :
//                        glColor3f(0, 0, 1);
//                        break;
//                    case 3 :
//                        glColor3f(1, 1, 0);
//                        break;
//                    case 4 :
//                        glColor3f(1, 0, 1);
//                        break;
//                    case 5 :
//                        glColor3f(0, 1, 1);
//                        break;
//                    case 6 :
//                        glColor3f(0.5, 0, 0);
//                        break;
//                    case 7 :
//                        glColor3f(0, 0.5, 0);
//                        break;
//                    case 8 :
//                        glColor3f(0, 0, 0.5);
//                        break;
//                    default :
//                        glColor3f(1, 1, 1);
//                        break;
//                }
////                for (int pi = 0; pi < inliersOfEachPlane[ci].indices.size(); pi++) {
////                    float x = pointsOfEachPlane[ci]->points[inliersOfEachPlane[ci].indices[pi]].x;
////                    float y = pointsOfEachPlane[ci]->points[inliersOfEachPlane[ci].indices[pi]].y;
////                    float z = pointsOfEachPlane[ci]->points[inliersOfEachPlane[ci].indices[pi]].z;
////                    //cout << " " << x << " " << y << " " << z << endl;
////                    glVertex3d(x, y, z);
////                }
//            for (int pi = 0; pi < clusters[ci].indices.size(); pi++) {
//                float x = RGBDCloudDownSample->points[clusters[ci].indices[pi]].x;
//                float y = RGBDCloudDownSample->points[clusters[ci].indices[pi]].y;
//                float z = RGBDCloudDownSample->points[clusters[ci].indices[pi]].z;
//                glVertex3d(x, y, z);
//            }
//            }
            glEnd();//点设置的结束
            pangolin::FinishFrame();
        }
}

int main() {
    std::cout << "Hello, World!" << std::endl;
    ifstream reader;
    //reader.open("initCloud_k0n_f1.txt", ios::in);
    reader.open("initCloud.txt", ios::in);
    pcl::PointCloud<pcl::PointXYZ>::Ptr RGBDCloud(new pcl::PointCloud<pcl::PointXYZ>);
    RGBDCloud->resize(500000);
    int actualNum = 0;
    while (!reader.eof()) {
        reader >> RGBDCloud->points[actualNum].x >> RGBDCloud->points[actualNum].y >> RGBDCloud->points[actualNum].z;
        actualNum++;
    }
    RGBDCloud->resize(actualNum);
    cout << "read " << RGBDCloud->points.size() << " points" << endl;

    ///Downsampling pointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr RGBDCloudDownSample(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(RGBDCloud);
    sor.setLeafSize(0.05f, 0.05f, 0.05f);
    sor.filter(*RGBDCloudDownSample);
    cout << "downsampling remains " << RGBDCloudDownSample->points.size() << endl;

    ///Estimating normals for each point
    pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
//    normal_estimator.setSearchMethod(tree);
//    normal_estimator.setInputCloud(RGBDCloudDownSample);
//    normal_estimator.setKSearch(100);
//    normal_estimator.compute(*normals);
    bool stop = false;
    int counter = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr PointEachLoop(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr InlineEachLoop(new pcl::PointCloud<pcl::PointXYZ>);
    vector<Plane> mvPlanes;
    int lastInlineNum = 0;
    int maxIteration = 1;
    clock_t startTime = clock();
    while (!stop && counter < maxIteration) {
        if (counter == 0)
            PointEachLoop = RGBDCloudDownSample;
        cout << "iteraation " << counter << " with " << PointEachLoop->points.size() << endl;
        normal_estimator.setSearchMethod(tree);
        normal_estimator.setInputCloud(PointEachLoop);
        normal_estimator.setKSearch(200);
        normal_estimator.compute(*normals);
        ///Region growing ------
        pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
        reg.setMinClusterSize(200);
        reg.setMaxClusterSize(50000);
        reg.setSearchMethod(tree);
        reg.setNumberOfNeighbours(200);//too little will cause runtime error
        reg.setInputCloud(PointEachLoop);
        reg.setInputNormals(normals);
        reg.setSmoothnessThreshold(7 / 180.0 / M_PI);
        reg.setCurvatureThreshold(7);

        ///extract each cluster
        //clock_t startTime = clock();
        std::vector<pcl::PointIndices> clusters;
        reg.extract(clusters);
        //clock_t endTime = clock();
        //double timeUsed = double(endTime - startTime) / CLOCKS_PER_SEC;
        //cout << "region growing " << timeUsed << " sec " << clusters.size() << " clusters found" << endl;
        ///Call RANSAC plane fitting for each Cluster
        vector<Plane> planes;
        vector<pcl::PointIndices> inliersOfEachPlane;
        vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pointsOfEachPlane;
        //vector<std::vector<double>> ClosePoints;
        //todo cluster sum up equal to RGBDDownsample? No
        int clusterSum = 0;
        for (int ci = 0; ci < clusters.size(); ci++) {
            clusterSum+=clusters[ci].indices.size();
            ///fill up each Cloud
            int clusterSize = clusters[ci].indices.size();
            pcl::PointCloud<pcl::PointXYZ>::Ptr thisCloud(new pcl::PointCloud<pcl::PointXYZ>);
            thisCloud->points.resize(clusterSize);
            thisCloud->height = 1;
            thisCloud->width = clusterSize;
            for (int index = 0; index < clusterSize; index++) {
                int pointIndex = clusters[ci].indices[index];
                thisCloud->points[index].x = PointEachLoop->points[pointIndex].x;
                thisCloud->points[index].y = PointEachLoop->points[pointIndex].y;
                thisCloud->points[index].z = PointEachLoop->points[pointIndex].z;
            }
            pointsOfEachPlane.push_back(thisCloud);
            Plane foundPlane;
            //startTime = clock();
            pcl::PointIndices inliersOut;
            int intPlaneNum = RANSACPlane(thisCloud, foundPlane, inliersOut);
            //endTime = clock();
            //double timeUsed = double(endTime - startTime) / CLOCKS_PER_SEC;
            //cout << "RANSAC plane fitting time used " << timeUsed << " | " << intPlaneNum << " of "<< clusters[ci].indices.size() << endl;
            planes.push_back(foundPlane);
            inliersOfEachPlane.push_back(inliersOut);
        }
        cout<<"cluster sum "<<clusterSum<<endl;

        ///Filter the points that far away from any Planes
//        double distanceThreshold = 0.1;
//        InlineEachLoop->resize(50000);
        int inlierNum = 0;
//        for (int pi = 0; pi < PointEachLoop->points.size(); pi++) {
//            //cout<<"pi "<<pi<<" of "<<DataInloop->points.size();
//            bool inlineFlag = false;
//            pcl::PointXYZ point = PointEachLoop->points[pi];
//            for (int pli = 0; pli < planes.size(); pli++) {
//                double distance = distance2Plane(point, planes[pli]);
//                if (distance < distanceThreshold) {
//                    inlineFlag = true;
//                    InlineEachLoop->points[inlierNum] = point;
//                    inlierNum++;
//                    break;
//                }
//            }
//        }
//        InlineEachLoop->resize(inlierNum);
//        InlineEachLoop->height = 1;
//        InlineEachLoop->width = inlierNum;
//        //PointEachLoop = InlineEachLoop;  this is copy pointer!!!!!
//        for (int pi = 0; pi < InlineEachLoop->points.size(); pi++) {
//            PointEachLoop->points[pi].x = InlineEachLoop->points[pi].x;
//            PointEachLoop->points[pi].y = InlineEachLoop->points[pi].y;
//            PointEachLoop->points[pi].z = InlineEachLoop->points[pi].z;
//        }
//        PointEachLoop->resize(inlierNum);
//        PointEachLoop->height = 1;
//        PointEachLoop->width = inlierNum;
        int diff = abs(lastInlineNum - inlierNum);
        lastInlineNum = inlierNum;
        cout << "inliner points " << inlierNum << " diff with last " << diff << endl;

        if (diff < 100)
            stop = true;
        counter++;

        if(stop || counter >= maxIteration){
            for (int pi = 0; pi < planes.size(); pi++) {
                cout << planes[pi].A << " " << planes[pi].B << " " << planes[pi].C << " " << planes[pi].D << endl;
                planes[pi].PlaneId=pi;
                mvPlanes.push_back(planes[pi]);
            }

            for (auto it = mvPlanes.begin(); it != mvPlanes.end(); it++) {
                //cout<<" it1 "<<it->PI.transpose()<<endl;
                for (auto it2 = mvPlanes.begin(); it2 != mvPlanes.end(); it2++) {
                    if (it != it2) {
                        //cout<<" it2 "<<it2->PI.transpose()<<endl;
                        Eigen::Vector3d PI1 = it->PI;
                        Eigen::Vector3d PI2 = it2->PI;
                        float distance = (PI1 - PI2).norm();
                        if (distance < 0.05 &&
                            abs(PI1[0] - PI2[0]) < 0.05 &&
                            abs(PI1[1] - PI2[1]) < 0.05 &&
                            abs(PI1[2] - PI2[2]) < 0.05) {
                            cout << "Merge Plane " << it->PI.transpose() << " and " << it2->PI.transpose() << endl;
                            //merge in-plane points
                            for (size_t pi = 0; pi < it2->planePts.size(); pi++) {
                                it->planePts.push_back(it2->planePts[pi]);
                            }
                            it->PI[0] = (it->PI[0] + it2->PI[0]) / 2;
                            it->PI[1] = (it->PI[1] + it2->PI[1]) / 2;
                            it->PI[2] = (it->PI[2] + it2->PI[2]) / 2;
                            mvPlanes.erase(it2);
                            it = mvPlanes.begin();
                            break;
                        }
                    }
                }
            }
        }

        if (stop || counter >= maxIteration) {

            //showPointCloud(PointEachLoop);
            ///show---plane points
            pangolin::CreateWindowAndBind("Main", 640, 480);
            glEnable(GL_DEPTH_TEST);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

            pangolin::OpenGlRenderState s_cam(
                    pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 320, 0.2, 100),
                    pangolin::ModelViewLookAt(2, 0, 2, 0, 0, 0, pangolin::AxisY)
            );

            pangolin::Handler3D handler(s_cam);
            pangolin::View &d_cam = pangolin::CreateDisplay()
                    .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f / 480.0f)
                    .SetHandler(&handler);
            while (!pangolin::ShouldQuit()) {
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                d_cam.Activate(s_cam);

                //需要绘制的东西写在这里
                //coordinate frame
                glBegin(GL_LINES);
                glLineWidth(5.0);
                glColor3f(1, 0, 0);
                glVertex3d(0.1, 0, 0); //X axis Red
                glVertex3d(0, 0, 0);
                glColor3f(0, 1, 0);
                glVertex3d(0, 0.1, 0); //Y axis Green
                glVertex3d(0, 0, 0);
                glColor3f(0, 0, 1);
                glVertex3d(0, 0, 0.1); //Z axis Blue
                glVertex3d(0, 0, 0);
                glEnd();

            //points
            glBegin(GL_POINTS);
            for (int ci = 0; ci < mvPlanes.size(); ci++) {
                //    for (int ci = 0; ci < 1; ci++) {
                //cout << "plane " << ci << " of total " << pointsOfEachPlane.size() << endl;
                switch (ci) {
                    case 0 :
                        glColor3f(1, 0, 0);
                        break;
                    case 1 :
                        glColor3f(0, 1, 0);
                        break;
                    case 2 :
                        glColor3f(0, 0, 1);
                        break;
                    case 3 :
                        glColor3f(1, 1, 0);
                        break;
                    case 4 :
                        glColor3f(1, 0, 1);
                        break;
                    case 5 :
                        glColor3f(0, 1, 1);
                        break;
                    case 6 :
                        glColor3f(0.5, 0, 0);
                        break;
                    case 7 :
                        glColor3f(0, 0.5, 0);
                        break;
                    case 8 :
                        glColor3f(0, 0, 0.5);
                        break;
                    default :
                        glColor3f(1, 1, 1);
                        break;
                }
                //ofstream writer1("planePts.txt");
                for(int pi = 0 ; pi < mvPlanes[ci].planePts.size(); pi++){
                    float x = mvPlanes[ci].planePts[pi].pt3d.x;
                    float y = mvPlanes[ci].planePts[pi].pt3d.y;
                    float z = mvPlanes[ci].planePts[pi].pt3d.z;
                    glVertex3d(x, y, z);
                    //writer1<<x << " " << y << " " << z <<endl;
//                    cout << " " << x << " " << y << " " << z <<" | ";
//                     x = pointsOfEachPlane[ci]->points[inliersOfEachPlane[ci].indices[pi]].x;
//                     y = pointsOfEachPlane[ci]->points[inliersOfEachPlane[ci].indices[pi]].y;
//                     z = pointsOfEachPlane[ci]->points[inliersOfEachPlane[ci].indices[pi]].z;
//                    cout << " " << x << " " << y << " " << z << endl;
                }
//                writer1.close();
//                ofstream writer2("indices.txt");
//                for (int pi = 0; pi < inliersOfEachPlane[ci].indices.size(); pi++) {
//                    float x = pointsOfEachPlane[ci]->points[inliersOfEachPlane[ci].indices[pi]].x;
//                    float y = pointsOfEachPlane[ci]->points[inliersOfEachPlane[ci].indices[pi]].y;
//                    float z = pointsOfEachPlane[ci]->points[inliersOfEachPlane[ci].indices[pi]].z;
//                    writer2<<x << " " << y << " " << z <<endl;
//                    //cout << " " << x << " " << y << " " << z << endl;
//                    //glVertex3d(x, y, z);
//                }
//                writer2.close();
//            for (int pi = 0; pi < clusters[ci].indices.size(); pi++) {
//                float x = RGBDCloudDownSample->points[clusters[ci].indices[pi]].x;
//                float y = RGBDCloudDownSample->points[clusters[ci].indices[pi]].y;
//                float z = RGBDCloudDownSample->points[clusters[ci].indices[pi]].z;
//                glVertex3d(x, y, z);
//            }
            }
                glEnd();//点设置的结束
                pangolin::FinishFrame();
            }
        }
    }
    clock_t endTime = clock();
    double timeUsed = double(endTime - startTime) / CLOCKS_PER_SEC;
    cout << "iteratively plane time used " << timeUsed <<endl;
    return 0;
}
