//
// Created by xin on 22/09/22.
//

#ifndef LIDARVIEWER_LIDAR_FUNCTIONS_H
#define LIDARVIEWER_LIDAR_FUNCTIONS_H

#include <pcl/filters/voxel_grid.h>
#include "tic_toc.h"
using namespace std;
const int N_SCANS = 64;
const double scanPeriod = 0.1;


void ExtractLiDARFeature(vector<vector<float>> mLaserPoints, vector<pcl::PointCloud<pcl::PointXYZI>> laserCloud16Scans);

//void mergeKeyLine1(vector<KeyLine> &keylines, vector<bool> &keylineMergeFlags);

#endif //LIDARVIEWER_LIDAR_FUNCTIONS_H
