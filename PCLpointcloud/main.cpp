#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
typedef pcl::PointXYZI PointType;
using namespace std;
int N_SCANS = 64;
int main() {
    std::cout << "Hello, World!" << std::endl;

    std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);
    pcl::PointCloud<PointType> singlePointCloud;
    PointType newP;
    newP.x = 0;
    newP.y = 0;
    newP.z = 0;
    newP.intensity = 10;
    singlePointCloud.push_back(newP);
    cout<<"singlePointCloud size "<<singlePointCloud.size()<<endl;
    laserCloudScans[1].push_back(newP);
    cout<<"laserCloudScans "<<laserCloudScans.size()<<" "<<laserCloudScans[1].size()<<endl;


    return 0;
}
