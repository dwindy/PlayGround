/*
 * This demo designed to test for LiDAR feature extact.
 */

#include <iostream>
#include <vector>
#include <pangolin/pangolin.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>
//#include "tic_toc.h"
///Line features
#include "LSD_merge.h"
#include "tool_functions.h"
///tidy up everything

#include "LiDAR_functions.h"

//using namespace std;
using namespace cv;
using namespace cv::line_descriptor;

//typedef pcl::PointXYZI PointType;

//Plane
class Plane {
public:
    //Ax + By + Cz = D ; D > 0
    double A, B, C, D;
    /**
    * (phi, theta, d) | (nx,ny,nz,d)
    * phi = arctan(ny/nx), theta = arccos(nz) ?
    * A = cos(phi)cos(theta), B = sin(phi)cos(tehta), C = -sin(theta)
    */
    double phi, theta, dis; //should change to radian
    Eigen::Vector3d PI;
    int count;
    int PlaneId;//Id in cur frame
    int IdGlobal;//In in map/global
    Plane(double Ain, double Bin, double Cin, double Din) : A(Ain), B(Bin), C(Cin), D(Din) {
        PlaneId = -1;
        count = -1;
        IdGlobal = -1;
//        NormD2CP();
//        NormD2CP();
    }

    Plane(double phiin, double thetain, double disin) : phi(phiin), theta(thetain), dis(disin) {
        PlaneId = -1;
        count = -1;
        IdGlobal = -1;
    }

    Plane(Eigen::Vector3d PIin) : PI(PIin) {
        PlaneId = -1;
        count = -1;
        IdGlobal = -1;
    }

    Plane() {
        PlaneId = -1;
        count = -1;
        IdGlobal = -1;
    }

    //not in use now. find plane contour?
    void foundContour(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

//    //void Norm2Angle();
//    void Norm2Radian();
//
//    void NormD2CP();

    cv::Point3d centreP;//the mean of all points
//        vector<cv::Point3d> pointList;
//        vector<cv::Point2d> pointList2D;
    vector<int> keyPointList; //todo store keypoint in this plane
    vector<int> mindices;     //todo indexs of keypoint in image
    // Some funcs
    void oplus(const Eigen::Vector3d &v);
};

///LiDAR Related Parameters
//int N_SCANS = 64;
//const double scanPeriod = 0.1;
//float cloudCurvature[400000];
//int cloudSortInd[400000];
//int cloudNeighborPicked[400000];
//int cloudLabel[400000];
////Todo the parameter above should define in some other places
//bool comp (int i,int j) { return (cloudCurvature[i]<cloudCurvature[j]); }
//pcl::PointCloud<pcl::PointXYZI> mCornerPointsSharp;
//pcl::PointCloud<pcl::PointXYZI> mCornerPointsLessSharp;
//pcl::PointCloud<pcl::PointXYZI> mSurfPointsFlat;
//pcl::PointCloud<pcl::PointXYZI> mSurfPointsLessFlat;
//std::vector<pcl::PointCloud<pcl::PointXYZI>> laserCloud16Scans(N_SCANS/4);


///Camera Related Parameters
int mnScaleLevels;
float mfScaleFactor;
float mfLogScaleFactor;
vector<float> mvScaleFactors;
vector<float> mvInvScaleFactors;
vector<float> mvLevelSigma2;
vector<float> mvInvLevelSigma2;



void readLaserPoints(string const fileName, vector<vector<float>> &laserPoints) {
    ///Step 1 load laser points
    //allocate 4MB buffer (around ~130 * 4 * 4 KB)
    int32_t num = 1000000;
    float *data = (float *) malloc(num * sizeof(float));
    //pointers for reading laser point
    float *px = data + 0;
    float *py = data + 1;
    float *pz = data + 2;
    float *pr = data + 3;
    //load point cloud
    FILE *fstream;
    fstream = fopen(fileName.c_str(), "rb");
    num = fread(data, sizeof(float), num, fstream) / 4;
    for (int i = 0; i < num; i++) {
        laserPoints[i][0] = *px;
        laserPoints[i][1] = *py;
        laserPoints[i][2] = *pz;
        laserPoints[i][3] = *pr;
        px += 4;
        py += 4;
        pz += 4;
        pr += 4;
    }
    fclose(fstream);
    //reset laserpoint vector size
    laserPoints.resize(num);
}

//Remove the close-to-origin points. once removed, pcl::pointcloud height = 1.
template<typename PointT>
void removeClosedPointCloud(const pcl::PointCloud <PointT> &cloud_in,
                            pcl::PointCloud <PointT> &cloud_out, float thres) {
    if (&cloud_in != &cloud_out) {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }

    size_t j = 0;
    // 把点云距离小于给定阈值的去除掉
    for (size_t i = 0; i < cloud_in.points.size(); ++i) {
        if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y +
            cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)
            continue;
        cloud_out.points[j] = cloud_in.points[i];
        j++;
    }
    if (j != cloud_in.points.size()) {
        cloud_out.points.resize(j);
    }

    cloud_out.height = 1; //orignal height is the scan line number
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;
}

///**
// * @brief Under LiDAR coordination, this function extract edge feature and plane feature from LiDAR source
// * @param[]
// */
//void ExtractLiDARFeature(vector<vector<float>> mLaserPoints) {
//    ///Step1 : fetch 3d lidar points from mLaserPoints (lidar coordination)
//    TicToc t_whole;
//    TicToc t_prepare;
//    std::vector<int> scanStartInd(N_SCANS, 0);//vector size 64, store each starts
//    std::vector<int> scanEndInd(N_SCANS, 0);//vector size 64, store each ends
//    pcl::PointCloud <pcl::PointXYZ> laserCloudIn;//all readin laser points
//    std::vector<int> indices;
//    unsigned long lsrPtNum = mLaserPoints.size();
//    laserCloudIn.resize(lsrPtNum);
//    for (int i = 0; i < lsrPtNum; i++) {
//        laserCloudIn.points[i].x = mLaserPoints[i][0];
//        laserCloudIn.points[i].y = mLaserPoints[i][1];
//        laserCloudIn.points[i].z = mLaserPoints[i][2];
//    }
//    pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
//    removeClosedPointCloud(laserCloudIn, laserCloudIn, 0.3);
//    ///Step2 : calc the angles
//    // 计算起始点和结束点的角度，由于激光雷达是顺时针旋转，这里取反就相当于转成了逆时针
//    int cloudSize = laserCloudIn.points.size();
//    float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);
//    // atan2范围是[-Pi,PI]，这里加上2PI是为了保证起始到结束相差2PI符合实际
//    float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y,
//                          laserCloudIn.points[cloudSize - 1].x) +
//                   2 * M_PI; //the end point should be 360 degree from the start point
//
//    // 总有一些例外，比如这里大于3PI，和小于PI，就需要做一些调整到合理范围
//    if (endOri - startOri > 3 * M_PI) //start -179, ends 179 degree
//    {
//        endOri -= 2 * M_PI;
//    } else if (endOri - startOri < M_PI) //start 179, ends -179 degree
//    {
//        endOri += 2 * M_PI;
//    }
//    //printf("end Ori %f\n", endOri);
//    bool halfPassed = false;
//    int count = cloudSize;
//    pcl::PointXYZI point;
//    std::vector<pcl::PointCloud<pcl::PointXYZI>> laserCloudScans(N_SCANS);
//    // 遍历每一个点
//    for (int i = 0; i < cloudSize; i++) {
//        point.x = laserCloudIn.points[i].x;
//        point.y = laserCloudIn.points[i].y;
//        point.z = laserCloudIn.points[i].z;
//        // 计算他的俯仰角
//        //cout<<"atan "<<atan(point.z / sqrt(point.x * point.x + point.y * point.y))<<endl;
//        float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
//        int scanID = 0;
//        // 计算是第几根scan
//        if (N_SCANS == 16) {
//            scanID = int((angle + 15) / 2 + 0.5);
//            if (scanID > (N_SCANS - 1) || scanID < 0) {
//                count--;
//                continue;
//            }
//        } else if (N_SCANS == 32) {
//            scanID = int((angle + 92.0 / 3.0) * 3.0 / 4.0);
//            if (scanID > (N_SCANS - 1) || scanID < 0) {
//                count--;
//                continue;
//            }
//        } else if (N_SCANS == 64) {
//            if (angle >= -8.83)
//                scanID = int((2 - angle) * 3.0 + 0.5);
//            else
//                scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);
//
//            // use [0 50]  > 50 remove outlies
//            if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0) //Velody HDL64E elevation: [2,-24.8]
//            {
//                count--;
//                continue;
//            }
//        } else {
//            printf("wrong scan number\n");
//            //ROS_BREAK();
//        }
//        //printf("angle %f scanID %d \n", angle, scanID);
//        // 计算水平角
//        float ori = -atan2(point.y, point.x);
//        if (!halfPassed) {
//            // 确保-PI / 2 < ori - startOri < 3 / 2 * PI
//            if (ori < startOri - M_PI / 2) {
//                ori += 2 * M_PI;
//            } else if (ori > startOri + M_PI * 3 / 2) {
//                ori -= 2 * M_PI;
//            }
//            // 如果超过180度，就说明过了一半了 //half passed the start point
//            if (ori - startOri > M_PI) {
//                halfPassed = true;
//            }
//        } else {
//            // 确保-PI * 3 / 2 < ori - endOri < PI / 2
//            ori += 2 * M_PI;    // 先补偿2PI
//            if (ori < endOri - M_PI * 3 / 2) {
//                ori += 2 * M_PI;
//            } else if (ori > endOri + M_PI / 2) {
//                ori -= 2 * M_PI;
//            }
//        }
//        // 角度的计算是为了计算相对的起始时刻的时间
//        float relTime = (ori - startOri) / (endOri - startOri);
//        // 整数部分是scan的索引，小数部分是相对起始时刻的时间
//        point.intensity = scanID + scanPeriod * relTime;
//        // 根据scan的idx送入各自数组
//        laserCloudScans[scanID].push_back(point);
//        ///added
//        if(scanID%4==0){
//            laserCloud16Scans[scanID/4].push_back(point);
//        }
//    }
//    // cloudSize是有效的点云的数目
//    cloudSize = count;
//    printf("points size %d \n", cloudSize);
//
//    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
//    cout<<laserCloud->size()<<endl;
//    // 全部集合到一个点云里面去，但是使用两个数组标记starts和end，这里分别+5和-6是为了计算曲率方便
//    // For example, 0th line contains 1000 points. the scanStartInd[0] = 5; the scanEndInd[0] = 994;
//    // 1th line contains 500 points. the scanStartInd[1] = 1005; the scanEndInd[1] = 1494;
//    for (int i = 0; i < N_SCANS; i++) {
//        scanStartInd[i] = laserCloud->size() + 5; //most left 5 and right 6 didn't count curve value
//        *laserCloud += laserCloudScans[i];
//        scanEndInd[i] = laserCloud->size() - 6;
//    }
//
//    //printf("prepare time %f \n", t_prepare.toc());
//    // 开始计算曲率
//    for (int i = 5; i < cloudSize - 5; i++) {
//        float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x + laserCloud->points[i - 3].x +
//                      laserCloud->points[i - 2].x + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x +
//                      laserCloud->points[i + 1].x + laserCloud->points[i + 2].x + laserCloud->points[i + 3].x +
//                      laserCloud->points[i + 4].x + laserCloud->points[i + 5].x;
//        float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y + laserCloud->points[i - 3].y +
//                      laserCloud->points[i - 2].y + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y +
//                      laserCloud->points[i + 1].y + laserCloud->points[i + 2].y + laserCloud->points[i + 3].y +
//                      laserCloud->points[i + 4].y + laserCloud->points[i + 5].y;
//        float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z + laserCloud->points[i - 3].z +
//                      laserCloud->points[i - 2].z + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z +
//                      laserCloud->points[i + 1].z + laserCloud->points[i + 2].z + laserCloud->points[i + 3].z +
//                      laserCloud->points[i + 4].z + laserCloud->points[i + 5].z;
//        // 存储曲率，索引
//        cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
//        cloudSortInd[i] = i;
//        cloudNeighborPicked[i] = 0;
//        cloudLabel[i] = 0;
//    }
//
//    TicToc t_pts;
//
//    pcl::PointCloud <pcl::PointXYZI> cornerPointsSharp;
//    pcl::PointCloud <pcl::PointXYZI> cornerPointsLessSharp;
//    pcl::PointCloud <pcl::PointXYZI> surfPointsFlat;
//    pcl::PointCloud <pcl::PointXYZI> surfPointsLessFlat;
//
//    float t_q_sort = 0;
//    // 遍历每个scan
//    for (int i = 0; i < N_SCANS; i++) {
//        if(i%4==0){
//            cout<<"process line "<<i<<endl;
//            // 没有有效的点了，就continue
//            if (scanEndInd[i] - scanStartInd[i] < 6) //this scan soo small
//                continue;
//            // 用来存储不太平整的点
//            pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<pcl::PointXYZI>);
//            // 将每个scan等分成6等分
//            for (int j = 0; j < 6; j++) {
//                // 每个等分的起始和结束点
//                int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6;
//                int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;
//
//                TicToc t_tmp;
//                // 对点云按照曲率进行排序，小的在前，大的在后
//                std::sort(cloudSortInd + sp, cloudSortInd + ep + 1, comp);//comp compare the cloudcurvature[i]
//                t_q_sort += t_tmp.toc();
//
//                int largestPickedNum = 0;
//                // 挑选曲率比较大的部分
//                for (int k = ep; k >= sp; k--) {
//                    // 排序后顺序就乱了，这个时候索引的作用就体现出来了
//                    int ind = cloudSortInd[k];//index of the LiDAR point
//
//                    // 看看这个点是否是有效点，同时曲率是否大于阈值
//                    if (cloudNeighborPicked[ind] == 0 && //tell neightbor that this point has been selected
//                        cloudCurvature[ind] > 0.1) {
//
//                        largestPickedNum++;
//                        // 每段选2个曲率大的点
//                        if (largestPickedNum <= 2) {
//                            // label为2是曲率大的标记
//                            cloudLabel[ind] = 2;
//                            // cornerPointsSharp存放大曲率的点
//                            cornerPointsSharp.push_back(laserCloud->points[ind]);
//                            cornerPointsLessSharp.push_back(laserCloud->points[ind]);
//                        }
//                            // 以及20个曲率稍微大一些的点
//                        else if (largestPickedNum <= 20) {
//                            // label置1表示曲率稍微大
//                            cloudLabel[ind] = 1;
//                            cornerPointsLessSharp.push_back(laserCloud->points[ind]);
//                        }
//                            // 超过20个就算了
//                        else {
//                            break;
//                        }
//                        // 这个点被选中后 pick标志位置1
//                        cloudNeighborPicked[ind] = 1;
//                        // 为了保证特征点不过度集中，将选中的点周围(前后各)5个点都置1,避免后续会选到
//                        for (int l = 1; l <= 5; l++) {
//                            // 查看相邻点距离是否差异过大，如果差异过大说明点云在此不连续，是特征边缘，就会是新的特征，因此就不置位了
//                            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
//                            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
//                            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
//                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
//                                break;
//                            }
//                            cloudNeighborPicked[ind + l] = 1;
//                        }
//                        // 下面同理
//                        for (int l = -1; l >= -5; l--) {
//                            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
//                            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
//                            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
//                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
//                                break;
//                            }
//                            cloudNeighborPicked[ind + l] = 1;
//                        }
//                    }
//                }
//                // 下面开始挑选面点
//                int smallestPickedNum = 0;
//                for (int k = sp; k <= ep; k++) {
//                    int ind = cloudSortInd[k];
//                    // 确保这个点没有被pick且曲率小于阈值
//                    if (cloudNeighborPicked[ind] == 0 &&
//                        cloudCurvature[ind] < 0.1) {
//                        // -1认为是平坦的点
//                        cloudLabel[ind] = -1;
//                        surfPointsFlat.push_back(laserCloud->points[ind]);
//
//                        smallestPickedNum++;
//                        // 这里不区分平坦和比较平坦，因为剩下的点label默认是0,就是比较平坦
//                        if (smallestPickedNum >= 4) {
//                            break;
//                        }
//                        // 下面同理
//                        cloudNeighborPicked[ind] = 1;
//                        for (int l = 1; l <= 5; l++) {
//                            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
//                            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
//                            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
//                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
//                                break;
//                            }
//                            cloudNeighborPicked[ind + l] = 1;
//                        }
//                        for (int l = -1; l >= -5; l--) {
//                            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
//                            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
//                            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
//                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
//                                break;
//                            }
//                            cloudNeighborPicked[ind + l] = 1;
//                        }
//                    }
//                }
//                //Less Flat is default lable 0
//                for (int k = sp; k <= ep; k++) {
//                    // 这里可以看到，剩下来的点都是一般平坦，这个也符合实际
//                    if (cloudLabel[k] <= 0) {
//                        surfPointsLessFlatScan->push_back(laserCloud->points[k]);
//                    }
//                }
//                //In some special case, like corner points number is far more than 20, we just select 20, some of the corner point will be label 0 !!!
//            }
//
//            pcl::PointCloud<pcl::PointXYZI> surfPointsLessFlatScanDS;
//            surfPointsLessFlatScanDS.resize(100000);
//            pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
//            // 一般平坦的点比较多，所以这里做一个体素滤波
//            downSizeFilter.setInputCloud(surfPointsLessFlatScan);
//            downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
//            downSizeFilter.filter(surfPointsLessFlatScanDS);
//
//            surfPointsLessFlat += surfPointsLessFlatScanDS;
//        }
//    }
//    //printf("sort q time %f \n", t_q_sort);
//    //printf("seperate points time %f \n", t_pts.toc());
//
//    ///Step 3 : pass features to frame member
//    mCornerPointsSharp.resize(cornerPointsSharp.size());
//    mCornerPointsSharp = cornerPointsSharp;
//    mCornerPointsLessSharp.resize(cornerPointsLessSharp.size());
//    mCornerPointsLessSharp = cornerPointsLessSharp;
//    mSurfPointsFlat.resize(surfPointsFlat.size());
//    mSurfPointsFlat = surfPointsFlat;
//    mSurfPointsLessFlat.resize(surfPointsLessFlat.size());
//    mSurfPointsLessFlat = surfPointsLessFlat;
//    printf("Corner Pt: %lu, Less Corner :%luu, Flat Pt: %lu, Less Flat Pt : %lu \n", mCornerPointsSharp.size(),
//           mCornerPointsLessSharp.size(), mSurfPointsFlat.size(), mSurfPointsLessFlat.size());
//    int pause = 1;
//}

/**
 * input a pointcloud, run RANSAC to fit a plane, return inliner number
 * @param in : cloud datasource
 * @param in&out : foundPlane, to fill up
 * @param in&out : inliersOutput
 * @return inliner point number
 */
int RANSACPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, Plane &foundPlane,
                       pcl::PointIndices &inliersOutput) {
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = inputCloud.makeShared();
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    //create the segmentation objects
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    //Optional
    seg.setOptimizeCoefficients(true);
    //Mandatory
    seg.setMethodType(pcl::SACMODEL_PLANE);
    seg.setModelType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.05);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
        return 0;
    inliersOutput = *inliers;
    foundPlane.A = coefficients->values[0];
    foundPlane.B = coefficients->values[1];
    foundPlane.C = coefficients->values[2];
    foundPlane.D = coefficients->values[3];
    /** AX+BY+CZ+D=0
     *  change to form AX+BY+CZ = -D
     *  And make sure (-D)>0 to adjust the Norm direction?
     */
    if (foundPlane.D < 0) {
        foundPlane.A = -foundPlane.A;
        foundPlane.B = -foundPlane.B;
        foundPlane.C = -foundPlane.C;
        foundPlane.D = -foundPlane.D;
    }
    double sumX = 0, sumY = 0, sumZ = 0;
    for (size_t i = 0; i < inliers->indices.size(); i++) {
        double x = cloud->points[inliers->indices[i]].x;
        double y = cloud->points[inliers->indices[i]].y;
        double z = cloud->points[inliers->indices[i]].z;
        sumX += x;
        sumY += y;
        sumZ += z;
        //cv::Point3d newP = cv::Point3d(x, y, z);
    }
//    foundPlane.Norm2Radian();
//    foundPlane.NormD2CP();
    foundPlane.centreP = cv::Point3d(sumX / inliers->indices.size(), sumY / inliers->indices.size(),
                                     sumZ / inliers->indices.size());
    return inliers->indices.size();
}

/**
* @brief Project LiDAR Points to Image. src: https://github.com/williamhyin/lidar_to_camera/blob/master/src/project_lidar_to_camera.cpp
* @param[in] mK : camera distortion and project parameters
* @param[in] cols : image cols boundary
* @param[in] rows : image rows boundary
*/

///**
// * @brief Project LiDAR Points and Features (PCL pointset) to Camera coordination system
// */
//void ProjectLiDARtoCam(vector<vector<float>> LiDARPoints, vector<point3d> &LiDARPoints_Cam) {
//    cv::Mat mTcamlid = cv::Mat::eye(4, 4, CV_64F);
//    mTcamlid.at<double>(0, 0) = Rcl00;
//    mTcamlid.at<double>(0, 1) = Rcl01;
//    mTcamlid.at<double>(0, 2) = Rcl02;
//    mTcamlid.at<double>(0, 3) = Tcl0;
//    mTcamlid.at<double>(1, 0) = Rcl10;
//    mTcamlid.at<double>(1, 1) = Rcl11;
//    mTcamlid.at<double>(1, 2) = Rcl12;
//    mTcamlid.at<double>(1, 3) = Tcl1;
//    mTcamlid.at<double>(2, 0) = Rcl20;
//    mTcamlid.at<double>(2, 1) = Rcl21;
//    mTcamlid.at<double>(2, 2) = Rcl22;
//    mTcamlid.at<double>(2, 3) = Tcl2;
//    ///Step1 Project common LiDAR points
//    int lsrPtNum = LiDARPoints.size();
//    if (lsrPtNum > 0) {
//        cv::Mat P_lidar(4, 1, CV_64F);//3D LiDAR point
//        cv::Mat P_cam(4, 1, CV_64F);//3D LiDAR point under Cam coordination
//        int counter = 0;
//        for (int li = 0; li < lsrPtNum; li++) {
//            //Velodyne Vertical FOV 26.9 mounted on 1.73. At 6 meter-distance, tan(26.9/2). it can only detect ~ 1.52+1.73 height
//            //X front, Y left, Z up
//            //Todo Test a valid threshold?
//            double maxX = 25.0, maxY = 25.0, minZ = 1.8;
//            maxX = 50.0, maxY = 50.0, minZ = 50;
//            if (LiDARPoints[li][0] > maxX || LiDARPoints[li][0] < 0.0
//                || LiDARPoints[li][1] > maxY || LiDARPoints[li][1] < -maxY
//                || LiDARPoints[li][2] > 20 || LiDARPoints[li][2] < -minZ) {
//                continue;
//            }
//            P_lidar.at<double>(0, 0) = LiDARPoints[li][0];
//            P_lidar.at<double>(1, 0) = LiDARPoints[li][1];
//            P_lidar.at<double>(2, 0) = LiDARPoints[li][2];
//            P_lidar.at<double>(3, 0) = 1;
//            P_cam = mTcamlid * P_lidar;
////            std::cout<<P_lidar.t()<<endl;//            std::cout<<mTcamlid<<endl;//            std::cout<<P_cam.t()<<endl;//            std::cout<<"-----"<<endl;
//            point3d newP;
//            newP.x = P_cam.at<double>(0, 0);
//            newP.y = P_cam.at<double>(1, 0);
//            newP.z = P_cam.at<double>(2, 0);
//            LiDARPoints_Cam.push_back(newP);
//        }
//    }
//}

///**
// * @brief Project LiDAR Points and Features (PCL pointset) to Camera coordination system
// */
//void ProjectLiDARtoCam(std::vector<pcl::PointCloud<pcl::PointXYZI>> laserCloud14Scans, vector<point3d> &LiDARPoints_Cam, vector<point3d> &GroundPoints_Cam) {
//    cv::Mat mTcamlid = cv::Mat::eye(4, 4, CV_64F);
//    mTcamlid.at<double>(0, 0) = Rcl00;
//    mTcamlid.at<double>(0, 1) = Rcl01;
//    mTcamlid.at<double>(0, 2) = Rcl02;
//    mTcamlid.at<double>(0, 3) = Tcl0;
//    mTcamlid.at<double>(1, 0) = Rcl10;
//    mTcamlid.at<double>(1, 1) = Rcl11;
//    mTcamlid.at<double>(1, 2) = Rcl12;
//    mTcamlid.at<double>(1, 3) = Tcl1;
//    mTcamlid.at<double>(2, 0) = Rcl20;
//    mTcamlid.at<double>(2, 1) = Rcl21;
//    mTcamlid.at<double>(2, 2) = Rcl22;
//    mTcamlid.at<double>(2, 3) = Tcl2;
//    for(int i = 0; i < laserCloud14Scans.size();i++){
//        int lsrPtNum = laserCloud14Scans[i].size();
//        if (lsrPtNum > 0) {
//            cv::Mat P_lidar(4, 1, CV_64F);//3D LiDAR point
//            cv::Mat P_cam(4, 1, CV_64F);//3D LiDAR point under Cam coordination
//            int counter = 0;
//            for (int li = 0; li < lsrPtNum; li++) {
//                //Velodyne Vertical FOV 26.9 mounted on 1.73. At 6 meter-distance, tan(26.9/2). it can only detect ~ 1.52+1.73 height
//                //X front, Y left, Z up
//                //Todo Test a valid threshold?
//                double maxX = 25.0, maxY = 25.0, minZ = 1.8;
//                maxX = 50.0, maxY = 50.0, minZ = 50;
//                if (laserCloud14Scans[i].points[li].x > maxX || laserCloud14Scans[i].points[li].x < 0.0
//                    || laserCloud14Scans[i].points[li].y > maxY || laserCloud14Scans[i].points[li].y < -maxY
//                    || laserCloud14Scans[i].points[li].z > 20 || laserCloud14Scans[i].points[li].z < -minZ) {
//                    continue;
//                }
//                P_lidar.at<double>(0, 0) = laserCloud14Scans[i].points[li].x;
//                P_lidar.at<double>(1, 0) = laserCloud14Scans[i].points[li].y;
//                P_lidar.at<double>(2, 0) = laserCloud14Scans[i].points[li].z;
//                P_lidar.at<double>(3, 0) = 1;
//                P_cam = mTcamlid * P_lidar;
////            std::cout<<P_lidar.t()<<endl;//            std::cout<<mTcamlid<<endl;//            std::cout<<P_cam.t()<<endl;//            std::cout<<"-----"<<endl;
//                point3d newP;
//                newP.x = P_cam.at<double>(0, 0);
//                newP.y = P_cam.at<double>(1, 0);
//                newP.z = P_cam.at<double>(2, 0);
//                newP.indexScani = i;
//                newP.indexScanj = li;
//                LiDARPoints_Cam.push_back(newP);
//                if(laserCloud14Scans[i].points[li].z<-1.7)
//                    GroundPoints_Cam.push_back(newP);
//            }
//        }
//    }
//}

///**
// * @brief Project LiDAR Points and Features (PCL pointset) to Camera coordination system
// */
//void ProjectLiDARtoCam(pcl::PointCloud<pcl::PointXYZI> LiDARFeatures, vector<point3d> &LiDARPoints_Cam, vector<point3d> &GroundPoints_Cam) {
//    cv::Mat mTcamlid = cv::Mat::eye(4, 4, CV_64F);
//    mTcamlid.at<double>(0, 0) = Rcl00;
//    mTcamlid.at<double>(0, 1) = Rcl01;
//    mTcamlid.at<double>(0, 2) = Rcl02;
//    mTcamlid.at<double>(0, 3) = Tcl0;
//    mTcamlid.at<double>(1, 0) = Rcl10;
//    mTcamlid.at<double>(1, 1) = Rcl11;
//    mTcamlid.at<double>(1, 2) = Rcl12;
//    mTcamlid.at<double>(1, 3) = Tcl1;
//    mTcamlid.at<double>(2, 0) = Rcl20;
//    mTcamlid.at<double>(2, 1) = Rcl21;
//    mTcamlid.at<double>(2, 2) = Rcl22;
//    mTcamlid.at<double>(2, 3) = Tcl2;
//        int lsrPtNum = LiDARFeatures.size();
//        if (lsrPtNum > 0) {
//            cv::Mat P_lidar(4, 1, CV_64F);//3D LiDAR point
//            cv::Mat P_cam(4, 1, CV_64F);//3D LiDAR point under Cam coordination
//            int counter = 0;
//            for (int li = 0; li < lsrPtNum; li++) {
//                //Velodyne Vertical FOV 26.9 mounted on 1.73. At 6 meter-distance, tan(26.9/2). it can only detect ~ 1.52+1.73 height
//                //X front, Y left, Z up
//                //Todo Test a valid threshold?
//                double maxX = 25.0, maxY = 25.0, minZ = 1.8;
//                maxX = 50.0, maxY = 50.0, minZ = 50;
//                if (LiDARFeatures.points[li].x > maxX || LiDARFeatures.points[li].x < 0.0
//                    || LiDARFeatures.points[li].y > maxY || LiDARFeatures.points[li].y < -maxY
//                    || LiDARFeatures.points[li].z > 20 || LiDARFeatures.points[li].z < -minZ) {
//                    continue;
//                }
//                P_lidar.at<double>(0, 0) = LiDARFeatures.points[li].x;
//                P_lidar.at<double>(1, 0) = LiDARFeatures.points[li].y;
//                P_lidar.at<double>(2, 0) = LiDARFeatures.points[li].z;
//                P_lidar.at<double>(3, 0) = 1;
//                P_cam = mTcamlid * P_lidar;
////            std::cout<<P_lidar.t()<<endl;//            std::cout<<mTcamlid<<endl;//            std::cout<<P_cam.t()<<endl;//            std::cout<<"-----"<<endl;
//                point3d newP;
//                newP.x = P_cam.at<double>(0, 0);
//                newP.y = P_cam.at<double>(1, 0);
//                newP.z = P_cam.at<double>(2, 0);
//                //newP.intensity?
//                LiDARPoints_Cam.push_back(newP);
//                if(newP.z<-1.7)
//                    GroundPoints_Cam.push_back(newP);
//            }
//        }
//}

///**
// * LiDAR under Cam coordination
// * Project to Image
// */
//void ProjectLiDARtoImg(int cols, int rows, vector<point3d> LiDARPoints, vector<point2d> &LiDARonImg) {
//    //it seems like OpenCV upgraded, then the func changed?
//    //cv::Mat P_rect_00 = cv::Mat::zeros(CvSize(4, 3), CV_64F);
//    cv::Mat P_rect_00 = cv::Mat::zeros(3,4,CV_64F);
//    P_rect_00.at<double>(0, 0) = fx;
//    P_rect_00.at<double>(0, 2) = cx;
//    P_rect_00.at<double>(1, 1) = fy;
//    P_rect_00.at<double>(1, 2) = cy;
//    P_rect_00.at<double>(2, 2) = 1;
//    //cv::Mat R_rect_00 = cv::Mat::eye(CvSize(4, 4), CV_64F);
//    cv::Mat R_rect_00 = cv::Mat::eye(4, 4, CV_64F);
//    int ptNum = LiDARPoints.size();
//    cv::Mat X(4, 1, CV_64F);//3D LiDAR point
//    cv::Mat Y(3, 1, CV_64F);//2D LiDAR projection
//    int counter = 0;
//    for (int pi = 0; pi < ptNum; pi++) {
//        //cv::Point pt;
//        point2d pt;
//        X.at<double>(0, 0) = LiDARPoints[pi].x;
//        X.at<double>(1, 0) = LiDARPoints[pi].y;
//        X.at<double>(2, 0) = LiDARPoints[pi].z;
//        X.at<double>(3, 0) = 1;
//        Y = P_rect_00 * R_rect_00 * X;
//        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0);
//        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0);
//        pt.index2line = -1;
//        pt.id2d = pi;
//        pt.indexScani = LiDARPoints[pi].indexScani;
//        pt.indexScanj = LiDARPoints[pi].indexScanj;
//        //why comment the checking will not lead system fail? dray point outside the image is okay for opencv?
////        if (pt.x < 0 || pt.x >= cols || pt.y < 0 || pt.y >= rows) {
////            //cout<<X.at<double>(0, 0)<<" "<<X.at<double>(1, 0)<<" "<<X.at<double>(2, 0)<<" -> "<<pt.x<<" "<<pt.y<<endl;
////            //mLaserPt_cam[pi].index2d = -1;
////            continue;
////        }
//        //cout<<X.at<double>(0, 0)<<" "<<X.at<double>(1, 0)<<" "<<X.at<double>(2, 0)<<" -> "<<pt.x<<" "<<pt.y<<endl;
////        mLaserPt_cam[pi].pt2d = pt;
////        mLaserPt_cam[pi].index2d = counter;
//        LiDARonImg.push_back(pt);
//        counter++;
//    }
//    //Test
////    X.at<double>(0, 0) = 3;
////    X.at<double>(1, 0) = 1;
////    X.at<double>(2, 0) = 5;
////    Y = P_rect_00 * R_rect_00 * X;
////    cv::Point pt;
////    pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0);
////    pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0);
////    cout<<"(3,1,5) -> "<<pt.x<<" "<<pt.y<<endl;
//    //TODO Check why the Projected LiDAR points are under some certain height
//    cout << "Lidar points total : " << LiDARPoints.size() << " in image frame : " << counter << endl;
//}


/**
 * @brief fitting a plane for lidar point on road
 */
bool PlaneFitting(vector<point3d> point3ds, vector<point2d> point2ds, cv::Mat im) {
    cv::Mat testIMG = im.clone();
    ///Step 1 store all 14 lines into a container
    pcl::PointCloud<pcl::PointXYZ>::Ptr allPoints(new pcl::PointCloud<pcl::PointXYZ>);
    allPoints->resize(point3ds.size());
    int actualNum = 0;
    for (int i = 0; i < point3ds.size(); i++) {
        allPoints->points[actualNum].x = point3ds[i].x;
        allPoints->points[actualNum].y = point3ds[i].y;
        allPoints->points[actualNum].z = point3ds[i].z;
        //allPoints->points[actualNum].intensity = laserCloud14Scans[i].points[j].intensity;
        actualNum++;

        cv::circle(testIMG,cv::Point(point2ds[i].x,point2ds[i].y),10,cv::Scalar(255,0,0),1);
        imshow("check ground points",testIMG);
        waitKey(1);
    }
    imshow("check ground points",testIMG);

    allPoints->resize(actualNum);
    cout<<" ground floor points "<<actualNum<<endl;
    //Step 2 DownSampling and calc norm
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud14DwSpl(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(allPoints);
    sor.setLeafSize(0.1f,0.1f,0.1f);
    sor.filter(*cloud14DwSpl);
    cout<<" downsample ground floor points "<<cloud14DwSpl->points.size()<<endl;
    vector<point2d> downsample2d;
    vector<point3d> downsample3d;
    for(int i = 0 ; i < cloud14DwSpl->size();i++){
        point3d newpoint;
        newpoint.x = cloud14DwSpl->points[i].x;
        newpoint.y = cloud14DwSpl->points[i].y;
        newpoint.z = cloud14DwSpl->points[i].z;
        downsample3d.push_back(newpoint);
    }
    ProjectLiDARtoImg(im.cols, im.rows, downsample3d, downsample2d);
    for (int i = 0; i < downsample2d.size(); i++) {
        cv::circle(testIMG,cv::Point(downsample2d[i].x,downsample2d[i].y),7,cv::Scalar(0,0,255),1);
        imshow("check ground points",testIMG);
        waitKey(1);
    }
    imshow("check ground points",testIMG);
    waitKey(1);

    //Calc norms
    pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(cloud14DwSpl);
    normal_estimator.setKSearch(100);
    normal_estimator.compute(*normals);
    //Step3 Region Growing
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize(200);
    reg.setMaxClusterSize(10000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(200);
    reg.setInputCloud(cloud14DwSpl);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(5.0/180.0/M_PI);
    reg.setCurvatureThreshold(5.0);
    //extract each cluster
    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);
    ///Step 4 RANSAC for each cluster
    vector<point3d> inliers3d;
    vector<point2d> inliers2d;
    for(size_t i = 0; i < clusters.size();i++){
        pcl::PointCloud<pcl::PointXYZ>::Ptr thisCloud(new pcl::PointCloud<pcl::PointXYZ>);
        thisCloud->points.resize(clusters[i].indices.size());
        thisCloud->height = 1;
        thisCloud->width = clusters[i].indices.size();
        for (size_t j = 0; j < clusters[i].indices.size(); j++) {
            thisCloud->points[j].x = cloud14DwSpl->points[clusters[i].indices[j]].x;
            thisCloud->points[j].y = cloud14DwSpl->points[clusters[i].indices[j]].y;
            thisCloud->points[j].z = cloud14DwSpl->points[clusters[i].indices[j]].z;
            //cout<<" cluster point "<<thisCloud->points[index].x<<" "<<thisCloud->points[index].y<<" "<<thisCloud->points[index].z<<endl;
            point3d newpoint;
            newpoint.x = thisCloud->points[j].x;
            newpoint.y = thisCloud->points[j].y;
            newpoint.z = thisCloud->points[j].z;
            inliers3d.push_back(newpoint);
        }
        cout<<"region growing points "<<inliers3d.size()<<endl;
        ProjectLiDARtoImg(im.cols, im.rows, inliers3d, inliers2d);
        for (int j = 0; j < inliers2d.size(); j++) {
            cv::circle(testIMG,cv::Point(inliers2d[j].x,inliers2d[j].y),5,cv::Scalar(0,255,0),1);
            imshow("check ground points",testIMG);
            waitKey(1);
        }
        imshow("check ground points",testIMG);
        //waitKey(0);

        Plane foundPlane;
        pcl::PointIndices inlierPoints;
        int intPlaneNum = RANSACPlane(thisCloud, foundPlane, inlierPoints);
        cout<<intPlaneNum<<" out of "<<thisCloud->points.size()<<" in RANSAC ground plane "<<endl;
        cout<<"plane attribution "<<foundPlane.A<<" "<<foundPlane.B<<" "<<foundPlane.C<<" "<<foundPlane.D<<endl;
        vector<point3d> ransac3d;
        vector<point2d> ransac2d;
        for(int j=0;j<inlierPoints.indices.size();j++){
            point3d newpoint;
            newpoint.x = thisCloud->points[inlierPoints.indices[j]].x;
            newpoint.y = thisCloud->points[inlierPoints.indices[j]].y;
            newpoint.z = thisCloud->points[inlierPoints.indices[j]].z;
            ransac3d.push_back(newpoint);
        }
        ProjectLiDARtoImg(im.cols, im.rows, ransac3d, ransac2d);
        for (int j = 0; j < ransac2d.size(); j++) {
            cv::circle(testIMG,cv::Point(ransac2d[j].x,ransac2d[j].y),1,cv::Scalar(255,255,0));
            imshow("check ground points",testIMG);
            waitKey(1);
        }
        imshow("check ground points",testIMG);
        //waitKey(0);

        return true;
    }
    return false;
}


void drawImage(cv::Mat &im, vector<point2d> LiDAR2d, vector<point2d> ImageFeatures){
    for (int i = 0; i < LiDAR2d.size(); i++) {
        if (LiDAR2d[i].index2line >= 0)
            cv::circle(im, cv::Point2d(LiDAR2d[i].x, LiDAR2d[i].y), 1, cv::Scalar(0, 0, 200), -1);
        else
            cv::circle(im, cv::Point2d(LiDAR2d[i].x, LiDAR2d[i].y), 1, cv::Scalar(127, 127, 0), -1);
    }
    for (int i = 0; i < ImageFeatures.size(); i++) {
        if (ImageFeatures[i].index2line >= 0)
            cv::circle(im, cv::Point2d(ImageFeatures[i].x, ImageFeatures[i].y), 1, cv::Scalar(0, 255, 0), -1);
        else
            cv::circle(im, cv::Point2d(ImageFeatures[i].x, ImageFeatures[i].y), 1, cv::Scalar(255, 0, 0), -1);
    }
    cv::circle(im,cv::Point2d(50,367),3,cv::Scalar(255, 255, 0),-1);
    cout<<"LiDAR2d size "<<LiDAR2d.size()<<endl;
}

///*
// * pair up the LSD lines, keypoints and 2d lidar points
// */
//void connectPointsLines(cv::Mat &im, vector<KeyLine> selectedKeyLSDLines, vector<point2d> &imgKeyPoint, vector<point2d> &LiDAR2d) {
//    //todo this should be in a line class ---(y2-y1)x+(x1-x2)y+(x2y1-x1y2)=0
//    vector<vector<float>> keyLineABCs;//store keylines in terms of Ax+By+C = 0;
//    for (int i = 0; i < selectedKeyLSDLines.size(); i++) {
//        float x1 = selectedKeyLSDLines[i].startPointX, y1 = selectedKeyLSDLines[i].startPointY;
//        float x2 = selectedKeyLSDLines[i].endPointX, y2 = selectedKeyLSDLines[i].endPointY;
//        float A = y2 - y1, B = x1 - x2, C = x2 * y1 - x1 * y2;
//        vector<float> thisLine;
//        thisLine.push_back(A);
//        thisLine.push_back(B);
//        thisLine.push_back(C);
//        keyLineABCs.push_back(thisLine);
//    }
//    ///Step 1 pair up LSD lines and ORB keypoints
//    vector<int> keyPt2LSD(imgKeyPoint.size(), -1);
//    for (int i = 0; i < imgKeyPoint.size(); i++) {
//        float x0 = imgKeyPoint[i].x, y0 = imgKeyPoint[i].y;
//        float minDistance1 = 5, minDistance2 = 5, minDistance3 = 5;
//        imgKeyPoint[i].index2line = -1;
//        int index1 = -1, index2 = -1, index3 = -1;
//        for (int j = 0; j < selectedKeyLSDLines.size(); j++) {
//            ///Step 1.1 search for 3 lines
//            ///point to line --- d = abs(Ax0+By0+C) / abs(sqrt(A^2+B^2))
//            float A = keyLineABCs[j][0], B = keyLineABCs[j][1], C = keyLineABCs[j][2];
//            float dis = abs(A * x0 + B * y0 + C) /
//                        sqrt(A * A + B * B);
//            if (dis < minDistance1) {
//                minDistance3 = minDistance2;
//                index3 = index2;
//                minDistance2 = minDistance1;
//                index2 = index1;
//                minDistance1 = dis;
//                index1 = j;
//            } else {
//                if (dis >= minDistance1 && dis < minDistance2) {
//                    minDistance3 = minDistance2;
//                    index3 = index2;
//                    minDistance2 = dis;
//                    index2 = j;
//                } else {
//                    if (dis >= minDistance2 && dis < minDistance3) {
//                        minDistance3 = dis;
//                        index3 = j;
//                    }
//                }
//            }
//        }
//        ///Step 1.2 select nearest line from above 3
//        float disToLineThres = 3; //? not in use current because above 3 line is qualified already
//        float disTo2EndsThres = 1;
//        if (index1 > -1) {
//            //cout<<"point "<<x0<<","<<y0<<" close to "<<keyLineABCs[index][0]<<","<<keyLineABCs[index][1]<<","<<keyLineABCs[index][2]<<endl;
//            ///Candidate 1 --- distance to each endpoints and sum up
//            float x1 = selectedKeyLSDLines[index1].startPointX, y1 = selectedKeyLSDLines[index1].startPointY;
//            float x2 = selectedKeyLSDLines[index1].endPointX, y2 = selectedKeyLSDLines[index1].endPointY;
//            float disToStart = sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
//            float disToEnd = sqrt((x0 - x2) * (x0 - x2) + (y0 - y2) * (y0 - y2));
//            float disTo2EndPoints = disToStart + disToEnd;
//            if (disToStart <= disToLineThres || disToEnd <= disToLineThres || disTo2EndPoints <= disTo2EndsThres) {
//                imgKeyPoint[i].index2line = index1;
//            } else {
//                if (index2 > -1) {
//                    ///Candidate 2 --- distance to each endpoints and sum up
//                    x1 = selectedKeyLSDLines[index2].startPointX, y1 = selectedKeyLSDLines[index2].startPointY;
//                    x2 = selectedKeyLSDLines[index2].endPointX, y2 = selectedKeyLSDLines[index2].endPointY;
//                    disToStart = sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
//                    disToEnd = sqrt((x0 - x2) * (x0 - x2) + (y0 - y2) * (y0 - y2));
//                    disTo2EndPoints = disToStart + disToEnd;
//                    if (disToStart <= disToLineThres || disToEnd <= disToLineThres ||
//                        disTo2EndPoints <= disTo2EndsThres) {
//                        imgKeyPoint[i].index2line = index2;
//                    }
//                } else {
//                    if (index3 > -1) {
//                        ///Candidate 2 --- distance to each endpoints and sum up
//                        x1 = selectedKeyLSDLines[index3].startPointX, y1 = selectedKeyLSDLines[index3].startPointY;
//                        x2 = selectedKeyLSDLines[index3].endPointX, y2 = selectedKeyLSDLines[index3].endPointY;
//                        disToStart = sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
//                        disToEnd = sqrt((x0 - x2) * (x0 - x2) + (y0 - y2) * (y0 - y2));
//                        disTo2EndPoints = disToStart + disToEnd;
//                        if (disToStart <= disToLineThres || disToEnd <= disToLineThres ||
//                            disTo2EndPoints <= disTo2EndsThres) {
//                            imgKeyPoint[i].index2line = index3;
//                        }
//                    }
//                }
//            }
//        }
//    }
//    ///Step 2 pair up LSD lines and lidar points
//    //vector<int> lidarPt2LSD(LiDAR2d.size(), -1);
//    for (int i = 0; i < LiDAR2d.size(); i++) {
//        float x0 = LiDAR2d[i].x, y0 = LiDAR2d[i].y;
//        ///Step 2.1 search for 3 nearest lines first
//        float minDistance1 = 5, minDistance2 = 5, minDistance3 = 5;
//        LiDAR2d[i].index2line = -1;
//        int index1 = -1, index2 = -1, index3 = -1;
//        for (int j = 0; j < selectedKeyLSDLines.size(); j++) {
//            ///Step 1.1 point to line --- d = abs(Ax0+By0+C) / abs(sqrt(A^2+B^2))
//            float A = keyLineABCs[j][0], B = keyLineABCs[j][1], C = keyLineABCs[j][2];
//            float dis = abs(A * x0 + B * y0 + C) /
//                        sqrt(A * A + B * B);
//            if (dis < minDistance1) {
//                minDistance3 = minDistance2;
//                index3 = index2;
//                minDistance2 = minDistance1;
//                index2 = index1;
//                minDistance1 = dis;
//                index1 = j;
//            } else {
//                if (dis >= minDistance1 && dis < minDistance2) {
//                    minDistance3 = minDistance2;
//                    index3 = index2;
//                    minDistance2 = dis;
//                    index2 = j;
//                } else {
//                    if (dis >= minDistance2 && dis < minDistance3) {
//                        minDistance3 = dis;
//                        index3 = j;
//                    }
//                }
//            }
//        }
//        ///Step 2.2 LiDAR point distance to each LSD endpoints
//        float disToLineThres = 3; //? not in use current because above 3 line is qualified already
//        float disTo2EndsThres = 1;
//        ///Candidate 1 --- distance to each endpoints and sum up
//        float x1 = selectedKeyLSDLines[index1].startPointX, y1 = selectedKeyLSDLines[index1].startPointY;
//        float x2 = selectedKeyLSDLines[index1].endPointX, y2 = selectedKeyLSDLines[index1].endPointY;
//        float disToStart = sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
//        float disToEnd = sqrt((x0 - x2) * (x0 - x2) + (y0 - y2) * (y0 - y2));
//        float disTo2EndPoints = disToStart + disToEnd;
//        if (disToStart <= disToLineThres || disToEnd <= disToLineThres || disTo2EndPoints <= disTo2EndsThres) {
//            LiDAR2d[i].index2line = index1;
//        } else {
//            if (index2 > -1) {
//                ///Candidate 2 --- distance to each endpoints and sum up
//                x1 = selectedKeyLSDLines[index2].startPointX, y1 = selectedKeyLSDLines[index2].startPointY;
//                x2 = selectedKeyLSDLines[index2].endPointX, y2 = selectedKeyLSDLines[index2].endPointY;
//                disToStart = sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
//                disToEnd = sqrt((x0 - x2) * (x0 - x2) + (y0 - y2) * (y0 - y2));
//                disTo2EndPoints = disToStart + disToEnd;
//                if (disToStart <= disToLineThres || disToEnd <= disToLineThres ||
//                    disTo2EndPoints <= disTo2EndsThres) {
//                    LiDAR2d[i].index2line = index2;
//                }
//            } else {
//                if (index3 > -1) {
//                    ///Candidate 2 --- distance to each endpoints and sum up
//                    x1 = selectedKeyLSDLines[index3].startPointX, y1 = selectedKeyLSDLines[index3].startPointY;
//                    x2 = selectedKeyLSDLines[index3].endPointX, y2 = selectedKeyLSDLines[index3].endPointY;
//                    disToStart = sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
//                    disToEnd = sqrt((x0 - x2) * (x0 - x2) + (y0 - y2) * (y0 - y2));
//                    disTo2EndPoints = disToStart + disToEnd;
//                    if (disToStart <= disToLineThres || disToEnd <= disToLineThres ||
//                        disTo2EndPoints <= disTo2EndsThres) {
//                        LiDAR2d[i].index2line = index3;
//                    }
//                }
//            }
//        }
//    }
//    for(int i = 0; i < keyLineABCs.size();i++){
//        keyLineABCs[i].clear();
//    }
//    keyLineABCs.clear();
//}

int main() {
    std::cout << "Hello, World!" << std::endl;
    string LiDARAddress = "../000000.bin";
    string ImageAddress = "../000000.png";
    string KeyPtAddress = "../000001KeyPt.txt";//ORB features
    ///Step 1 load point cloud
    vector<vector<float>> laserPoints;
    for (int i = 0; i < 1000000; i++) {
        vector<float> point = {0, 0, 0, 0};
        laserPoints.push_back(point);
    }
    readLaserPoints(LiDARAddress, laserPoints);
    cout << "LaserPoints point size " << laserPoints.size() << endl;
    ///Step 2 process point cloud
    ExtractLiDARFeature(laserPoints);//todo 16laser as a input para
    for (int i = 0; i < 16; i++)
        cout << "line " << i << " size " << laserCloud16Scans[i].size() << endl;

    ///Project to image
    cv::Mat im = cv::imread(ImageAddress, CV_LOAD_IMAGE_UNCHANGED);
    cout<<"image cols "<<im.cols<<" rows "<<im.rows<<endl;
    vector<point3d> LiDARPoints_Cam;//Lidar point under cam coordination system
    vector<point3d> LiDARGroundPoints_Cam;//on ground floor point
    //ProjectLiDARtoCam(laserPoints, LiDARPoints_Cam);
    ProjectLiDARtoCam(laserCloud16Scans, LiDARPoints_Cam, LiDARGroundPoints_Cam);//project 16 lines
    //cout<<"LiDARPoints on Cam Size :"<<LiDARPoints_Cam.size()<<endl;
    vector<point2d> LiDAR2d;//Lidar point in 2d image
    vector<point2d> LiDARfloor;
    ProjectLiDARtoImg(im.cols, im.rows, LiDARPoints_Cam, LiDAR2d);
    ProjectLiDARtoImg(im.cols, im.rows, LiDARGroundPoints_Cam, LiDARfloor);
    cout<<"gournd point number "<<LiDARGroundPoints_Cam.size()<<endl;

    ///Read Image Key pt
    vector<point2d> ImgKeyPoint;//Lidar point in image ???? why i wrote this?
    ifstream keyReader;
    keyReader.open("../000001KeyPt.txt");
    double x,y;
    int id = 0;
    while(keyReader>>x>>y){
        //cout<<"id "<<id<<": "<<x<<" "<<y<<endl;
        point2d newPt;
        newPt.x = x; newPt.y = y; newPt.id2d=id; newPt.index2line = -1;
        id ++;
        ImgKeyPoint.push_back(newPt);
    }
    keyReader.close();

    ///Step 2.5 LSD and Lines
    //cvtColor(im,im,CV_GRAY2BGR);
    //binary mask
    cv::Mat mask = Mat::ones(im.size(), CV_8UC1);
    Ptr<BinaryDescriptor> bd = BinaryDescriptor::createBinaryDescriptor();
    vector<KeyLine> priKeylines;
    cv::Mat output_bd = im.clone();
    cv::Mat output_bd_merge = im.clone();
    cvtColor(output_bd_merge,output_bd_merge,CV_GRAY2BGR);
    cvtColor(im,im,CV_GRAY2BGR);
    bd->detect(im, priKeylines, mask);
    /* compute descriptors */
    cv::Mat descriptors;
    bd->compute(im, priKeylines, descriptors);
    /* merge primary keylines*/
    std::vector<bool> keylineMergeFlags(priKeylines.size(), false);
    mergeKeyLine1(priKeylines,keylineMergeFlags);
    /* select the lines with enough length */
    vector<KeyLine> selectedKeyLines;
    for (int i = 0; i < priKeylines.size(); i++) {
        if (true==keylineMergeFlags[i])
            continue;
        if(priKeylines[i].lineLength>lineLength){
            selectedKeyLines.push_back(priKeylines[i]);
        }
    }
    /* show */
    for (size_t i = 0; i < selectedKeyLines.size(); i++) {
        KeyLine kl = selectedKeyLines[i];
//        cout << "selected keyline No." << selectedKeyLines[i].class_id << " with angle " << selectedKeyLines[i].angle
//             << " response " << selectedKeyLines[i].response << " length :" << selectedKeyLines[i].lineLength
//             << " start point " << selectedKeyLines[i].startPointX << " " << selectedKeyLines[i].startPointY
//             << " end point " << selectedKeyLines[i].endPointX << " " << selectedKeyLines[i].endPointY
//             << endl;
        if (kl.octave == 0) {
            /* get a random color */
            int R = (rand() % (int) (255 + 1));
            int G = (rand() % (int) (255 + 1));
            int B = (rand() % (int) (255 + 1));
            /* get extremes of line */
            Point pt1 = Point2f(kl.startPointX, kl.startPointY);
            Point pt2 = Point2f(kl.endPointX, kl.endPointY);
            /* draw line */
            line(output_bd_merge, pt1, pt2, Scalar(B, G, R), 2);
            line(im, pt1, pt2, Scalar(B, G, R), 1);
        }
    }
    imshow("BD lines merged", output_bd_merge);
    waitKey();

    ///Step 3 connect feature point and feature line
    connectPointsLines(output_bd_merge, selectedKeyLines, ImgKeyPoint, LiDAR2d);

    ///Step 4 calc road plane
    PlaneFitting(LiDARGroundPoints_Cam, LiDARfloor, im);

    ///Step 5 show point cloud
//    //todo thread show point clouds
//    //cv::circle(im, cv::Point2f(470, 359), 3, cv::Scalar(0, 200, 200), -1);
    drawImage(im,LiDAR2d,ImgKeyPoint);
//    cv::imshow("ORB-SLAM2: Current Frame",im);
//    cv::waitKey(0);
//    ///show------------LiDAR-----------
//    pangolin::CreateWindowAndBind("Main", 640, 480);
//    glEnable(GL_DEPTH_TEST);
//    glEnable(GL_BLEND);
//    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//
//    pangolin::OpenGlRenderState s_cam(
//            pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 320, 0.2, 100),
//            pangolin::ModelViewLookAt(2, 0, 2, 0, 0, 0, pangolin::AxisY)
//    );
//
//    pangolin::Handler3D handler(s_cam);
//    pangolin::View &d_cam = pangolin::CreateDisplay()
//            .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f / 480.0f)
//            .SetHandler(&handler);
//    //while (!pangolin::ShouldQuit()) {
//    while (cvWaitKey(1)) {
//        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//        d_cam.Activate(s_cam);
//
//        //需要绘制的东西写在这里
//        //coordinate frame
//        glBegin(GL_LINES);
//        glLineWidth(5.0);
//        glColor3f(1, 0, 0);
//        glVertex3d(0.1, 0, 0);
//        glVertex3d(0, 0, 0);
//        glColor3f(0, 1, 0);
//        glVertex3d(0, 0.1, 0);
//        glVertex3d(0, 0, 0);
//        glColor3f(0, 0, 1);
//        glVertex3d(0, 0, 0.1);
//        glVertex3d(0, 0, 0);
//        glEnd();
//
//        for (int li = 0; li < laserCloud16Scans.size(); li++) {
//            glBegin(GL_POINTS);
//            for (int ci = 0; ci < laserCloud16Scans[li].size(); ci++) {
//                if (laserCloud16Scans[li].points[ci].x > 0) {
//                    glColor3f(1, 1, 1); //60 20 220
//                    if (laserCloud16Scans[li].points[ci].z < -1.7) {
//                        glColor3f(0, 1, 1); //60 20 220
//                    }
//                    glVertex3d(laserCloud16Scans[li].points[ci].x,
//                               laserCloud16Scans[li].points[ci].y,
//                               laserCloud16Scans[li].points[ci].z);
//                }
//            }
//            glEnd();
//        }
//        for(int i =0; i < LiDAR2d.size();i++){
//            glBegin(GL_POINTS);
//            glColor3f(1, 0, 0); //60 20 220
//            if(LiDAR2d[i].index2line>-1){
//                int indexLine = LiDAR2d[i].indexScani;
//                int indexPoint = LiDAR2d[i].indexScanj;
//                glVertex3d(laserCloud16Scans[indexLine].points[indexPoint].x, laserCloud16Scans[indexLine].points[indexPoint].y,
//                           laserCloud16Scans[indexLine].points[indexPoint].z);
//            }
//            glEnd();
//        }
//
////        for (int ci = 0; ci < laserPoints.size(); ci++) {
////            glBegin(GL_POINTS);
////            glColor3f(1, 1, 1); //60 20 220
////            glVertex3d(laserPoints[ci][0], laserPoints[ci][1], laserPoints[ci][2]);
////            glEnd();
////        }
//
////        for (int ci = 0; ci < mCornerPointsSharp.points.size(); ci++) {
////            if (mCornerPointsSharp.points[ci].x > 0) {
////                glBegin(GL_POINTS);
////                glColor3f(0.8627, 0.078, 0.2352); //60 20 220
////                glVertex3d(mCornerPointsSharp.points[ci].x, mCornerPointsSharp.points[ci].y,
////                           mCornerPointsSharp.points[ci].z);
////                glEnd();
////            }
////
////        }
////
////        for (int ci = 0; ci < mCornerPointsLessSharp.points.size(); ci++) {
////            if (mCornerPointsLessSharp.points[ci].x > 0) {
////                glBegin(GL_POINTS);
////                glColor3f(1, 0.4117, 0.7058); //180 105 255
////                glVertex3d(mCornerPointsLessSharp.points[ci].x, mCornerPointsLessSharp.points[ci].y,
////                           mCornerPointsLessSharp.points[ci].z);
////                glEnd();
////            }
////        }
//
////        for (int ci = 0; ci < mSurfPointsFlat.points.size(); ci++) {
////            glBegin(GL_POINTS);
////            glColor3f(0, 0, 1); //255 0 0
////            glVertex3d(mSurfPointsFlat.points[ci].x, mSurfPointsFlat.points[ci].y, mSurfPointsFlat.points[ci].z);
////            glEnd();
////        }
//
////        for (int ci = 0; ci < mSurfPointsLessFlat.points.size(); ci++) {
////            glBegin(GL_POINTS);
////            glColor3f(0, 1, 1); //255 255 0
////            glVertex3d(mSurfPointsLessFlat.points[ci].x, mSurfPointsLessFlat.points[ci].y,
////                       mSurfPointsLessFlat.points[ci].z);
////            glEnd();
////        }
//
//        pangolin::FinishFrame();
//    }
    ///-----------------------------------------
    return 0;
}
