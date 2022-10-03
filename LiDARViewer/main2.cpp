//
// Created by xin on 22/09/22.
// A tidy up version of main.cpp
// Test step by step !
//

#include "common_head.h"
#include "tool_functions.h"
#include "LiDAR_functions.h"

using namespace std;


int main(){
    std::cout << "Hello, World!" << std::endl;
    string LiDARAddress = "../000000.bin";
    string ImageAddress = "../000000.png";
    string KeyPtAddress = "../000001KeyPt.txt";//ORB features

    ///Step 1.1 load LiDAR point cloud
    vector<vector<float>> laserPoints;
    for (int i = 0; i < 1000000; i++) {
        vector<float> point = {0, 0, 0, 0};
        laserPoints.push_back(point);
    }
    readLaserPoints(LiDARAddress, laserPoints);

    ///Step 1.2 process LiDAR point cloud
    std::vector<pcl::PointCloud<pcl::PointXYZI>> laserCloud16Scans(N_SCANS/4);
    ExtractLiDARFeature(laserPoints);//todo 16laser as a input para
    for (int i = 0; i < 16; i++)
        cout << "line " << i << " size " << laserCloud16Scans[i].size() << endl;

    return 0;
}
