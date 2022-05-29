#include <iostream>
#include<opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <pangolin/pangolin.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
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
#include <pcl/filters/voxel_grid.h>

int main() {
    std::cout << "Hello, World!" << std::endl;
    ifstream reader;
    reader.open("beforeFix.txt", ios::in);
    pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud1(new pcl::PointCloud<pcl::PointXYZ>);
    PointCloud1->resize(100000);
    int actualNum1 = 0;
    while (!reader.eof()){
        reader>>PointCloud1->points[actualNum1].x>>PointCloud1->points[actualNum1].y>>PointCloud1->points[actualNum1].z;
        actualNum1++;
    }
    reader.close();
    PointCloud1->resize(actualNum1);
    cout<<"read "<<actualNum1<<" points "<<endl;

    reader.open("afterFix.txt", ios::in);
    pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud2(new pcl::PointCloud<pcl::PointXYZ>);
    PointCloud2->resize(100000);
    int actualNum2 = 0;
    while (!reader.eof()){
        reader>>PointCloud2->points[actualNum2].x>>PointCloud2->points[actualNum2].y>>PointCloud2->points[actualNum2].z;
        actualNum2++;
    }
    reader.close();
    PointCloud2->resize(actualNum2);
    cout<<"read "<<actualNum2<<" points "<<endl;

    ///show----------------------- Downsample
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
        glColor3f(1, 0, 1);
        glVertex3d(0.1, 0, 0);
        glVertex3d(0, 0, 0);
        glColor3f(0, 1, 0);
        glVertex3d(0, 0.1, 0);
        glVertex3d(0, 0, 0);
        glColor3f(0, 0, 1);
        glVertex3d(0, 0, 0.1);
        glVertex3d(0, 0, 0);
        glEnd();
        //points
//        for (int ci = 0; ci < PointCloud1->points.size(); ci++) {
//            glBegin(GL_POINTS);
//            glColor3f(1, 1, 1);
//            glVertex3d(PointCloud1->points[ci].x, PointCloud1->points[ci].y, PointCloud1->points[ci].z);
//            glEnd();
//        }

        //points
        for (int ci = 0; ci < PointCloud2->points.size(); ci++) {
            glBegin(GL_POINTS);
            glColor3f(0, 1, 1);
            glVertex3d(PointCloud2->points[ci].x, PointCloud2->points[ci].y, PointCloud2->points[ci].z);
            glEnd();
        }
        pangolin::FinishFrame();
    }
    ///-----------------------------------------

    return 0;
}
