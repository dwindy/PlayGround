/*
 * This was Region growing first (0.3 sec)
 * then RANSAC to fit plane for each cluster
 */
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <limits>
#include <cstring>
#include <pangolin/pangolin.h>
#include <ctime>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/registration/icp.h>
#include <Eigen/Dense>
using namespace std;

Eigen::MatrixXf projectVision(Eigen::Matrix4f velocity, Eigen::MatrixXf &f1, Eigen::MatrixXf &f2, int &num1, int &num2)
{
    ///load vision project
    string filename1 = "../data/keypoint/2.txt";
    Eigen::Matrix<float,4,1000> InitMatrix;
    ifstream reader;
    reader.open(filename1, ios::binary);
    float x,y,z;
    int i=0;
    for (; !reader.eof(); i++) {
        reader >> x >> y >> z;
        InitMatrix(0, i) = x;
        InitMatrix(1, i) = y;
        InitMatrix(2, i) = z;
        InitMatrix(3, i) = 1;
    }
    reader.close();
    f1 = InitMatrix;
    num1 = i;
    cout<<"vision mappoint load "<<i<<endl;

    string filename2 = "../data/keypoint/3.txt";
    Eigen::Matrix<float,4,1000> InitMatrix2;
    ifstream reader2;
    reader2.open(filename2, ios::binary);
    int j=0;
    for (; !reader2.eof(); j++) {
        reader2 >> x >> y >> z;
        InitMatrix2(0, j) = x;
        InitMatrix2(1, j) = y;
        InitMatrix2(2, j) = z;
        InitMatrix2(3, j) = 1;
    }
    reader2.close();
    f2 = InitMatrix2;
    num2 = j;
    cout<<"vision mappoint 2 load "<<j<<endl;

    ///
    Eigen::MatrixXf projectedP = velocity * f1.block(0,0,4,i);
    return projectedP;
}

int main() {
    std::cout << "Hello, World!" << std::endl;
    ///load bin
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
//    cloud1->points.resize(1000000);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
//    cloud2->points.resize(1000000);
//    int32_t num = 1000000;
//    float *data = (float *) malloc(num * sizeof(float));
//    float *px = data + 0;
//    float *py = data + 1;
//    float *pz = data + 2;
//    float *pr = data + 3;
//    FILE *fstream;
//    string file1 = "../data/pointcloud/0000000002.bin";
//    fstream = fopen(file1.c_str(), "rb");
//    num = fread(data, sizeof(float), num, fstream) / 4;
//    int actualNum = 0;
//    for (int i = 0; i < num; i++) {
//        if (*px > 0 && *px < 25 && abs(*py) < 6) {
//            cloud1->points[actualNum].x = *px;
//            cloud1->points[actualNum].y = *py;
//            cloud1->points[actualNum].z = *pz;
//            actualNum++;
//        }
//        px += 4;
//        py += 4;
//        pz += 4;
//        pr += 4;
//    }
//    fclose(fstream);
//    cloud1->resize(actualNum);
//    cout << "actual loaded " << actualNum << " points" << endl;
//
//    num = 1000000;
//    float *data2 = (float *) malloc(num * sizeof(float));
//    px = data2 + 0;
//    py = data2 + 1;
//    pz = data2 + 2;
//    pr = data2 + 3;
//    string file2 = "../data/pointcloud/0000000003.bin";
//    fstream = fopen(file2.c_str(), "rb");
//    num = fread(data2, sizeof(float), num, fstream) / 4;
//    actualNum = 0;
//    for (int i = 0; i < num; i++) {
//        if (*px > 0 && *px < 25 && abs(*py) < 6) {
//            cloud2->points[actualNum].x = *px;
//            cloud2->points[actualNum].y = *py;
//            cloud2->points[actualNum].z = *pz;
//            actualNum++;
//        }
//        px += 4;
//        py += 4;
//        pz += 4;
//        pr += 4;
//    }
//    fclose(fstream);
//    cloud2->resize(actualNum);
//    cout << "actual loaded " << actualNum << " points" << endl;

    for(int f=2;f<125;f++)
    {
        //load txt data
        int numInit = 30000;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
        cloud1->points.resize(numInit);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
        cloud2->points.resize(numInit);
        string fileName = "../data/lidar/"+std::to_string(f)+".txt";
        ifstream reader;
        string lines;
        reader.open(fileName, ios::binary);
        float x,y,z;
        int i=0;
        for(; !reader.eof();i++)
        {
            reader>>x>>y>>z;
            cloud1->points[i].x = x;
            cloud1->points[i].y = y;
            cloud1->points[i].z = z;
        }
        reader.close();
        cloud1->points.resize(i);
        cout<<"point cloud 1 size "<<cloud1->points.size();

        fileName = "../data/lidar/"+std::to_string(f+1)+".txt";
        reader.open(fileName, ios::binary);
        i=0;
        for(; !reader.eof();i++)
        {
            reader>>x>>y>>z;
            cloud2->points[i].x = x;
            cloud2->points[i].y = y;
            cloud2->points[i].z = z;
            //cout<<x<<" "<<y<<" "<<z<<endl;
        }
        reader.close();
        cloud2->points.resize(i);
        cout<<"point cloud 2 size "<<cloud2->points.size()<<endl;

        ///ICP
        Eigen::Matrix4f init;
//        init << 0.99999243, 0.00071144709, 0.0038281197, 0.00091641,
//                -0.00070561917, 0.99999851, -0.0015234993, -0.00020845441,
//                -0.0038291987, 0.0015207866, 0.99999154, -0.042928811,
//                0, 0, 0, 1;
        pcl::PointCloud<pcl::PointXYZ> cloud_source_registered;
        cloud_source_registered.points.resize(numInit);
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icper;
        icper.setInputSource(cloud1);
        icper.setInputTarget(cloud2);
        //icper.setMaxCorrespondenceDistance(1);
        //icper.setTransformationEpsilon(1e-8);//?
        //icper.setEuclideanFitnessEpsilon(0.01);
        icper.setMaximumIterations(100);
        //cout <<"init input "<<endl<< init << endl << endl;
        icper.align(cloud_source_registered);//, init);
        Eigen::Matrix4f transformation = icper.getFinalTransformation();
        //cout << "cloud_source_registered size " << cloud_source_registered.size() << endl;
        cout <<"ICP result "<<endl<<transformation << endl << endl;

        ofstream writer;
        string filename = "../data/velocity_lidar/"+std::to_string(f)+".txt";
        writer.open(filename);
        writer<<transformation<<endl;
        writer.close();
    }

//    ///deal with vision points
//    Eigen::MatrixXf vision1, vision2;
//    Eigen::MatrixXf result;
//    int num1, num2;
//    result = projectVision(init, vision1, vision2, num1, num2);
//    Eigen::MatrixXf result2;
//    result2 = transformation * vision1.block(0,0,4,num1);

    ///Draw
//    pangolin::CreateWindowAndBind("Main", 640,480);
//    glEnable(GL_DEPTH_TEST);
//    glEnable(GL_BLEND);
//    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//
//    pangolin::OpenGlRenderState s_cam(pangolin::ProjectionMatrix(640,480,420,420,320,320,0.2,100), pangolin::ModelViewLookAt(2,0,2,0,0,0,pangolin::AxisY));
//    pangolin::Handler3D handler(s_cam);
//    pangolin::View &d_cam = pangolin::CreateDisplay().SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f).SetHandler(&handler);
//    while(!pangolin::ShouldQuit()){
//        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
//        d_cam.Activate(s_cam);
//        ///draw
//        //coordinate frame
//        glBegin(GL_LINES);
//        glLineWidth(5.0);
//        glColor3f(1, 0, 1);
//        glVertex3d(0.1, 0, 0);
//        glVertex3d(0, 0, 0);
//        glColor3f(0, 1, 0);
//        glVertex3d(0, 0.1, 0);
//        glVertex3d(0, 0, 0);
//        glColor3f(0, 0, 1);
//        glVertex3d(0, 0, 0.1);
//        glVertex3d(0, 0, 0);
//        glEnd();
//        ///LiDAR points
//        glBegin(GL_POINTS);
//        for(int pi = 0; pi < cloud1->points.size();pi++)
//        {
//            glColor3f(1,0,0);
//            glVertex3d(cloud1->points[pi].x,cloud1->points[pi].y,cloud1->points[pi].z);
//        }
//        glEnd();
//        glBegin(GL_POINTS);
//        for(int pi = 0; pi < cloud_source_registered.points.size();pi++)
//        {
//            glColor3f(0,1,0);
//            glVertex3d(cloud_source_registered.points[pi].x,cloud_source_registered.points[pi].y,cloud_source_registered.points[pi].z);
//        }
//        glEnd();
//        glBegin(GL_POINTS);
//        for(int pi = 0; pi < cloud2->points.size();pi++)
//        {
//            glColor3f(0,0,1);
//            glVertex3d(cloud2->points[pi].x,cloud2->points[pi].y,cloud2->points[pi].z);
//        }
//        glEnd();
//        ///vision points
////        glBegin(GL_POINTS);
////        for (int pi = 0; pi < num1; pi++) {
////            glColor3f(1, 0, 0);
////            glVertex3d(vision1(0, pi), vision1(1, pi), vision1(2, pi));
////        }
////        glEnd();
////        glBegin(GL_POINTS);
////        for (int pi = 0; pi < num1; pi++) {
////            glColor3f(0, 1, 0);
////            glVertex3d(result(0, pi), result(1, pi), result(2, pi));
////        }
////        glEnd();
////        glBegin(GL_POINTS);
////        for (int pi = 0; pi < num1; pi++) {
////            glColor3f(0, 1, 1);
////            glVertex3d(result2(0, pi), result2(1, pi), result2(2, pi));
////        }
////        glEnd();
////        glBegin(GL_POINTS);
////        for (int pi = 0; pi < num2; pi++) {
////            glColor3f(0, 0, 1);
////            glVertex3d(vision2(0, pi), vision2(1, pi), vision2(2, pi));
////        }
////        glEnd();
//        pangolin::FinishFrame();
//    }
    return 0;
}
