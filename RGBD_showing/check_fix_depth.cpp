#include <iostream>
#include<opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <pangolin/pangolin.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;

// 主函数
int main(int argc, char **argv) {
    ///Read from txt file
    ifstream reader;
    reader.open("FixDepth.txt", ios::in);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_before(new pcl::PointCloud<pcl::PointXYZ>);
    pc_before->resize(500000);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_after(new pcl::PointCloud<pcl::PointXYZ>);
    pc_after->resize(500000);
    int actualNum = 0;
    while (!reader.eof()) {
        reader >> pc_before->points[actualNum].x >> pc_before->points[actualNum].y >> pc_before->points[actualNum].z
               >> pc_after->points[actualNum].x >> pc_after->points[actualNum].y >> pc_after->points[actualNum].z;
        actualNum++;
    }
    pc_before->resize(actualNum);
    pc_after->resize(actualNum);
    cout << "read " << pc_before->points.size() << " | "<<pc_after->points.size()<<" points" << endl;

    ///show-----------------------
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

        for (int ci = 0; ci < pc_before->points.size(); ci++) {
            glBegin(GL_POINTS);
            glColor3f(1, 0, 0);
            glVertex3d(pc_before->points[ci].x, pc_before->points[ci].y, pc_before->points[ci].z);
            glEnd();
        }

        for (int ci = 0; ci < pc_after->points.size(); ci++) {
            glBegin(GL_POINTS);
            glColor3f(1, 1, 1);
            glVertex3d(pc_after->points[ci].x, pc_after->points[ci].y, pc_after->points[ci].z);
            glEnd();
        }
        pangolin::FinishFrame();
    }
    ///-----------------------------------------
    return 0;

}