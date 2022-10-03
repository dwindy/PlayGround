//
// Created by xin on 22/09/22.
//
//#include "LSD_merge.h"
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/io/pcd_io.h>

#include "common_head.h"
using namespace std;

///////Camera LiDAR Calibration Parameters
//should be const https://blog.csdn.net/ZZHinclude/article/details/118557412
//double Rcl00 = 0.007533745;
//double Rcl01 = -0.999714;
//double Rcl02 = -0.000616602;
//double Rcl10 = 0.01480249;
//double Rcl11 = 0.0007280733;
//double Rcl12 = -0.9998902;
//double Rcl20 = 0.9998621;
//double Rcl21 = 0.00752379;
//double Rcl22 = 0.0148075;
//double Tcl0 = -0.004069766;
//double Tcl1 = -0.07631618;
//double Tcl2 = -0.2717806;
/////Camera Parameters
//double fx = 718.856;
//double fy = 718.856;
//double cx =  607.1928;
//double cy= 185.2157;
//double k1 = 0.0;
//double k2 = 0.0;
//double p1 = 0.0;
//double p2 = 0.0;


void readLaserPoints(string fileName, vector<vector<float>> &laserPoints);

//void ProjectLiDARtoImg(int cols, int rows, vector<point3d> LiDARPoints, vector<point2d> &LiDARonImg);
//
//void ProjectLiDARtoCam(pcl::PointCloud<pcl::PointXYZI> LiDARFeatures, vector<point3d> &LiDARPoints_Cam, vector<point3d> &GroundPoints_Cam);
//
//void ProjectLiDARtoCam(std::vector<pcl::PointCloud<pcl::PointXYZI>> laserCloud14Scans, vector<point3d> &LiDARPoints_Cam, vector<point3d> &GroundPoints_Cam);
//
//void ProjectLiDARtoCam(vector<vector<float>> LiDARPoints, vector<point3d> &LiDARPoints_Cam);