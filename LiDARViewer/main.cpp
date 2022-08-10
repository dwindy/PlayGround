/*
 * This demo designed to test for LiDAR feature extact.
 */
#include <iostream>
#include <vector>
#include <pangolin/pangolin.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include "tic_toc.h"

using namespace std;
typedef pcl::PointXYZI PointType;

///LiDAR Related Parameters
int N_SCANS = 64;
const double scanPeriod = 0.1;
float cloudCurvature[400000];
int cloudSortInd[400000];
int cloudNeighborPicked[400000];
int cloudLabel[400000];
//Todo the parameter above should define in some other places
bool comp (int i,int j) { return (cloudCurvature[i]<cloudCurvature[j]); }
pcl::PointCloud<PointType> mCornerPointsSharp;
pcl::PointCloud<PointType> mCornerPointsLessSharp;
pcl::PointCloud<PointType> mSurfPointsFlat;
pcl::PointCloud<PointType> mSurfPointsLessFlat;
std::vector<pcl::PointCloud<PointType>> laserCloud14Scans(N_SCANS/4);

///Camera LiDAR Calibration Parameters
double Rcl00 = 0.007533745;
double Rcl01 = -0.999714;
double Rcl02 = -0.000616602;
double Rcl10 = 0.01480249;
double Rcl11 = 0.0007280733;
double Rcl12 = -0.9998902;
double Rcl20 = 0.9998621;
double Rcl21 = 0.00752379;
double Rcl22 = 0.0148075;
double Tcl0 = -0.004069766;
double Tcl1 = -0.07631618;
double Tcl2 = -0.2717806;
///Camera Parameters
double fx = 718.856;
double fy = 718.856;
double cx =  607.1928;
double cy= 185.2157;
double k1 = 0.0;
double k2 = 0.0;
double p1 = 0.0;
double p2 = 0.0;
///Camera Related Parameters
int mnScaleLevels;
float mfScaleFactor;
float mfLogScaleFactor;
vector<float> mvScaleFactors;
vector<float> mvInvScaleFactors;
vector<float> mvLevelSigma2;
vector<float> mvInvLevelSigma2;

struct point2d{
    float x;
    float y;
    int id2d;
};

struct point3d{
    float x;
    float y;
    float z;
    int id3d;
};

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

/**
 * @brief Under LiDAR coordination, this function extract edge feature and plane feature from LiDAR source
 * @param[]
 */
void ExtractLiDARFeature(vector<vector<float>> mLaserPoints) {
    ///Step1 : fetch 3d lidar points from mLaserPoints (lidar coordination)
    TicToc t_whole;
    TicToc t_prepare;
    std::vector<int> scanStartInd(N_SCANS, 0);//vector size 64, store each starts
    std::vector<int> scanEndInd(N_SCANS, 0);//vector size 64, store each ends
    pcl::PointCloud <pcl::PointXYZ> laserCloudIn;//all readin laser points
    std::vector<int> indices;
    unsigned long lsrPtNum = mLaserPoints.size();
    laserCloudIn.resize(lsrPtNum);
    for (int i = 0; i < lsrPtNum; i++) {
        laserCloudIn.points[i].x = mLaserPoints[i][0];
        laserCloudIn.points[i].y = mLaserPoints[i][1];
        laserCloudIn.points[i].z = mLaserPoints[i][2];
    }
    pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
    removeClosedPointCloud(laserCloudIn, laserCloudIn, 0.3);
    ///Step2 : calc the angles
    // 计算起始点和结束点的角度，由于激光雷达是顺时针旋转，这里取反就相当于转成了逆时针
    int cloudSize = laserCloudIn.points.size();
    float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);
    // atan2范围是[-Pi,PI]，这里加上2PI是为了保证起始到结束相差2PI符合实际
    float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y,
                          laserCloudIn.points[cloudSize - 1].x) +
                   2 * M_PI; //the end point should be 360 degree from the start point

    // 总有一些例外，比如这里大于3PI，和小于PI，就需要做一些调整到合理范围
    if (endOri - startOri > 3 * M_PI) //start -179, ends 179 degree
    {
        endOri -= 2 * M_PI;
    } else if (endOri - startOri < M_PI) //start 179, ends -179 degree
    {
        endOri += 2 * M_PI;
    }
    //printf("end Ori %f\n", endOri);
    bool halfPassed = false;
    int count = cloudSize;
    PointType point;
    std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);
    // 遍历每一个点
    for (int i = 0; i < cloudSize; i++) {
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;
        // 计算他的俯仰角
        //cout<<"atan "<<atan(point.z / sqrt(point.x * point.x + point.y * point.y))<<endl;
        float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
        int scanID = 0;
        // 计算是第几根scan
        if (N_SCANS == 16) {
            scanID = int((angle + 15) / 2 + 0.5);
            if (scanID > (N_SCANS - 1) || scanID < 0) {
                count--;
                continue;
            }
        } else if (N_SCANS == 32) {
            scanID = int((angle + 92.0 / 3.0) * 3.0 / 4.0);
            if (scanID > (N_SCANS - 1) || scanID < 0) {
                count--;
                continue;
            }
        } else if (N_SCANS == 64) {
            if (angle >= -8.83)
                scanID = int((2 - angle) * 3.0 + 0.5);
            else
                scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);

            // use [0 50]  > 50 remove outlies
            if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0) //Velody HDL64E elevation: [2,-24.8]
            {
                count--;
                continue;
            }
        } else {
            printf("wrong scan number\n");
            //ROS_BREAK();
        }
        //printf("angle %f scanID %d \n", angle, scanID);
        // 计算水平角
        float ori = -atan2(point.y, point.x);
        if (!halfPassed) {
            // 确保-PI / 2 < ori - startOri < 3 / 2 * PI
            if (ori < startOri - M_PI / 2) {
                ori += 2 * M_PI;
            } else if (ori > startOri + M_PI * 3 / 2) {
                ori -= 2 * M_PI;
            }
            // 如果超过180度，就说明过了一半了 //half passed the start point
            if (ori - startOri > M_PI) {
                halfPassed = true;
            }
        } else {
            // 确保-PI * 3 / 2 < ori - endOri < PI / 2
            ori += 2 * M_PI;    // 先补偿2PI
            if (ori < endOri - M_PI * 3 / 2) {
                ori += 2 * M_PI;
            } else if (ori > endOri + M_PI / 2) {
                ori -= 2 * M_PI;
            }
        }
        // 角度的计算是为了计算相对的起始时刻的时间
        float relTime = (ori - startOri) / (endOri - startOri);
        // 整数部分是scan的索引，小数部分是相对起始时刻的时间
        point.intensity = scanID + scanPeriod * relTime;
        // 根据scan的idx送入各自数组
        laserCloudScans[scanID].push_back(point);
        ///added
        if(scanID%4==0){
            laserCloud14Scans[scanID/4].push_back(point);
        }
    }
    // cloudSize是有效的点云的数目
    cloudSize = count;
    printf("points size %d \n", cloudSize);

    pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
    cout<<laserCloud->size()<<endl;
    // 全部集合到一个点云里面去，但是使用两个数组标记starts和end，这里分别+5和-6是为了计算曲率方便
    // For example, 0th line contains 1000 points. the scanStartInd[0] = 5; the scanEndInd[0] = 994;
    // 1th line contains 500 points. the scanStartInd[1] = 1005; the scanEndInd[1] = 1494;
    for (int i = 0; i < N_SCANS; i++) {
        scanStartInd[i] = laserCloud->size() + 5; //most left 5 and right 6 didn't count curve value
        *laserCloud += laserCloudScans[i];
        scanEndInd[i] = laserCloud->size() - 6;
    }

    //printf("prepare time %f \n", t_prepare.toc());
    // 开始计算曲率
    for (int i = 5; i < cloudSize - 5; i++) {
        float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x + laserCloud->points[i - 3].x +
                      laserCloud->points[i - 2].x + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x +
                      laserCloud->points[i + 1].x + laserCloud->points[i + 2].x + laserCloud->points[i + 3].x +
                      laserCloud->points[i + 4].x + laserCloud->points[i + 5].x;
        float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y + laserCloud->points[i - 3].y +
                      laserCloud->points[i - 2].y + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y +
                      laserCloud->points[i + 1].y + laserCloud->points[i + 2].y + laserCloud->points[i + 3].y +
                      laserCloud->points[i + 4].y + laserCloud->points[i + 5].y;
        float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z + laserCloud->points[i - 3].z +
                      laserCloud->points[i - 2].z + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z +
                      laserCloud->points[i + 1].z + laserCloud->points[i + 2].z + laserCloud->points[i + 3].z +
                      laserCloud->points[i + 4].z + laserCloud->points[i + 5].z;
        // 存储曲率，索引
        cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
        cloudSortInd[i] = i;
        cloudNeighborPicked[i] = 0;
        cloudLabel[i] = 0;
    }

    TicToc t_pts;

    pcl::PointCloud <PointType> cornerPointsSharp;
    pcl::PointCloud <PointType> cornerPointsLessSharp;
    pcl::PointCloud <PointType> surfPointsFlat;
    pcl::PointCloud <PointType> surfPointsLessFlat;

    float t_q_sort = 0;
    // 遍历每个scan
    for (int i = 0; i < N_SCANS; i++) {
        if(i%4==0){
            cout<<"process line "<<i<<endl;
            // 没有有效的点了，就continue
            if (scanEndInd[i] - scanStartInd[i] < 6) //this scan soo small
                continue;
            // 用来存储不太平整的点
            pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);
            // 将每个scan等分成6等分
            for (int j = 0; j < 6; j++) {
                // 每个等分的起始和结束点
                int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6;
                int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;

                TicToc t_tmp;
                // 对点云按照曲率进行排序，小的在前，大的在后
                std::sort(cloudSortInd + sp, cloudSortInd + ep + 1, comp);//comp compare the cloudcurvature[i]
                t_q_sort += t_tmp.toc();

                int largestPickedNum = 0;
                // 挑选曲率比较大的部分
                for (int k = ep; k >= sp; k--) {
                    // 排序后顺序就乱了，这个时候索引的作用就体现出来了
                    int ind = cloudSortInd[k];//index of the LiDAR point

                    // 看看这个点是否是有效点，同时曲率是否大于阈值
                    if (cloudNeighborPicked[ind] == 0 && //tell neightbor that this point has been selected
                        cloudCurvature[ind] > 0.1) {

                        largestPickedNum++;
                        // 每段选2个曲率大的点
                        if (largestPickedNum <= 2) {
                            // label为2是曲率大的标记
                            cloudLabel[ind] = 2;
                            // cornerPointsSharp存放大曲率的点
                            cornerPointsSharp.push_back(laserCloud->points[ind]);
                            cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                        }
                            // 以及20个曲率稍微大一些的点
                        else if (largestPickedNum <= 20) {
                            // label置1表示曲率稍微大
                            cloudLabel[ind] = 1;
                            cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                        }
                            // 超过20个就算了
                        else {
                            break;
                        }
                        // 这个点被选中后 pick标志位置1
                        cloudNeighborPicked[ind] = 1;
                        // 为了保证特征点不过度集中，将选中的点周围(前后各)5个点都置1,避免后续会选到
                        for (int l = 1; l <= 5; l++) {
                            // 查看相邻点距离是否差异过大，如果差异过大说明点云在此不连续，是特征边缘，就会是新的特征，因此就不置位了
                            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
                                break;
                            }
                            cloudNeighborPicked[ind + l] = 1;
                        }
                        // 下面同理
                        for (int l = -1; l >= -5; l--) {
                            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
                                break;
                            }
                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }
                // 下面开始挑选面点
                int smallestPickedNum = 0;
                for (int k = sp; k <= ep; k++) {
                    int ind = cloudSortInd[k];
                    // 确保这个点没有被pick且曲率小于阈值
                    if (cloudNeighborPicked[ind] == 0 &&
                        cloudCurvature[ind] < 0.1) {
                        // -1认为是平坦的点
                        cloudLabel[ind] = -1;
                        surfPointsFlat.push_back(laserCloud->points[ind]);

                        smallestPickedNum++;
                        // 这里不区分平坦和比较平坦，因为剩下的点label默认是0,就是比较平坦
                        if (smallestPickedNum >= 4) {
                            break;
                        }
                        // 下面同理
                        cloudNeighborPicked[ind] = 1;
                        for (int l = 1; l <= 5; l++) {
                            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
                                break;
                            }
                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--) {
                            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
                                break;
                            }
                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }
                //Less Flat is default lable 0
                for (int k = sp; k <= ep; k++) {
                    // 这里可以看到，剩下来的点都是一般平坦，这个也符合实际
                    if (cloudLabel[k] <= 0) {
                        surfPointsLessFlatScan->push_back(laserCloud->points[k]);
                    }
                }
                //In some special case, like corner points number is far more than 20, we just select 20, some of the corner point will be label 0 !!!
            }

            pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
            surfPointsLessFlatScanDS.resize(100000);
            pcl::VoxelGrid<PointType> downSizeFilter;
            // 一般平坦的点比较多，所以这里做一个体素滤波
            downSizeFilter.setInputCloud(surfPointsLessFlatScan);
            downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
            downSizeFilter.filter(surfPointsLessFlatScanDS);

            surfPointsLessFlat += surfPointsLessFlatScanDS;
        }
    }
    //printf("sort q time %f \n", t_q_sort);
    //printf("seperate points time %f \n", t_pts.toc());

    ///Step 3 : pass features to frame member
    mCornerPointsSharp.resize(cornerPointsSharp.size());
    mCornerPointsSharp = cornerPointsSharp;
    mCornerPointsLessSharp.resize(cornerPointsLessSharp.size());
    mCornerPointsLessSharp = cornerPointsLessSharp;
    mSurfPointsFlat.resize(surfPointsFlat.size());
    mSurfPointsFlat = surfPointsFlat;
    mSurfPointsLessFlat.resize(surfPointsLessFlat.size());
    mSurfPointsLessFlat = surfPointsLessFlat;
    printf("Corner Pt: %u, Less Corner : %u, Flat Pt: %u, Less Flat Pt : %u \n", mCornerPointsSharp.size(),
           mCornerPointsLessSharp.size(), mSurfPointsFlat.size(), mSurfPointsLessFlat.size());
    int pause = 1;
}

/**
* @brief Project LiDAR Points to Image. src: https://github.com/williamhyin/lidar_to_camera/blob/master/src/project_lidar_to_camera.cpp
* @param[in] mK : camera distortion and project parameters
* @param[in] cols : image cols boundary
* @param[in] rows : image rows boundary
*/

/**
 * @brief Project LiDAR Points and Features (PCL pointset) to Camera coordination system
 */
void ProjectLiDARtoCam(vector<vector<float>> LiDARPoints, vector<point3d> &LiDARPoints_Cam) {
    cv::Mat mTcamlid = cv::Mat::eye(4, 4, CV_64F);
    mTcamlid.at<double>(0, 0) = Rcl00;
    mTcamlid.at<double>(0, 1) = Rcl01;
    mTcamlid.at<double>(0, 2) = Rcl02;
    mTcamlid.at<double>(0, 3) = Tcl0;
    mTcamlid.at<double>(1, 0) = Rcl10;
    mTcamlid.at<double>(1, 1) = Rcl11;
    mTcamlid.at<double>(1, 2) = Rcl12;
    mTcamlid.at<double>(1, 3) = Tcl1;
    mTcamlid.at<double>(2, 0) = Rcl20;
    mTcamlid.at<double>(2, 1) = Rcl21;
    mTcamlid.at<double>(2, 2) = Rcl22;
    mTcamlid.at<double>(2, 3) = Tcl2;
    ///Step1 Project common LiDAR points
    int lsrPtNum = LiDARPoints.size();
    if (lsrPtNum > 0) {
        cv::Mat P_lidar(4, 1, CV_64F);//3D LiDAR point
        cv::Mat P_cam(4, 1, CV_64F);//3D LiDAR point under Cam coordination
        int counter = 0;
        for (int li = 0; li < lsrPtNum; li++) {
            //Velodyne Vertical FOV 26.9 mounted on 1.73. At 6 meter-distance, tan(26.9/2). it can only detect ~ 1.52+1.73 height
            //X front, Y left, Z up
            //Todo Test a valid threshold?
            double maxX = 25.0, maxY = 25.0, minZ = 1.8;
            maxX = 50.0, maxY = 50.0, minZ = 50;
            if (LiDARPoints[li][0] > maxX || LiDARPoints[li][0] < 0.0
                || LiDARPoints[li][1] > maxY || LiDARPoints[li][1] < -maxY
                || LiDARPoints[li][2] > 20 || LiDARPoints[li][2] < -minZ) {
                continue;
            }
            P_lidar.at<double>(0, 0) = LiDARPoints[li][0];
            P_lidar.at<double>(1, 0) = LiDARPoints[li][1];
            P_lidar.at<double>(2, 0) = LiDARPoints[li][2];
            P_lidar.at<double>(3, 0) = 1;
            P_cam = mTcamlid * P_lidar;
//            std::cout<<P_lidar.t()<<endl;//            std::cout<<mTcamlid<<endl;//            std::cout<<P_cam.t()<<endl;//            std::cout<<"-----"<<endl;
            point3d newP;
            newP.x = P_cam.at<double>(0, 0);
            newP.y = P_cam.at<double>(1, 0);
            newP.z = P_cam.at<double>(2, 0);
            LiDARPoints_Cam.push_back(newP);
        }
    }
}

/**
 * @brief Project LiDAR Points and Features (PCL pointset) to Camera coordination system
 */
void ProjectLiDARtoCam(std::vector<pcl::PointCloud<PointType>> laserCloud14Scans, vector<point3d> &LiDARPoints_Cam) {
    cv::Mat mTcamlid = cv::Mat::eye(4, 4, CV_64F);
    mTcamlid.at<double>(0, 0) = Rcl00;
    mTcamlid.at<double>(0, 1) = Rcl01;
    mTcamlid.at<double>(0, 2) = Rcl02;
    mTcamlid.at<double>(0, 3) = Tcl0;
    mTcamlid.at<double>(1, 0) = Rcl10;
    mTcamlid.at<double>(1, 1) = Rcl11;
    mTcamlid.at<double>(1, 2) = Rcl12;
    mTcamlid.at<double>(1, 3) = Tcl1;
    mTcamlid.at<double>(2, 0) = Rcl20;
    mTcamlid.at<double>(2, 1) = Rcl21;
    mTcamlid.at<double>(2, 2) = Rcl22;
    mTcamlid.at<double>(2, 3) = Tcl2;
    for(int i = 0; i < laserCloud14Scans.size();i++){
        int lsrPtNum = laserCloud14Scans[i].size();
        if (lsrPtNum > 0) {
            cv::Mat P_lidar(4, 1, CV_64F);//3D LiDAR point
            cv::Mat P_cam(4, 1, CV_64F);//3D LiDAR point under Cam coordination
            int counter = 0;
            for (int li = 0; li < lsrPtNum; li++) {
                //Velodyne Vertical FOV 26.9 mounted on 1.73. At 6 meter-distance, tan(26.9/2). it can only detect ~ 1.52+1.73 height
                //X front, Y left, Z up
                //Todo Test a valid threshold?
                double maxX = 25.0, maxY = 25.0, minZ = 1.8;
                maxX = 50.0, maxY = 50.0, minZ = 50;
                if (laserCloud14Scans[i].points[li].x > maxX || laserCloud14Scans[i].points[li].x < 0.0
                    || laserCloud14Scans[i].points[li].y > maxY || laserCloud14Scans[i].points[li].y < -maxY
                    || laserCloud14Scans[i].points[li].z > 20 || laserCloud14Scans[i].points[li].z < -minZ) {
                    continue;
                }
                P_lidar.at<double>(0, 0) = laserCloud14Scans[i].points[li].x;
                P_lidar.at<double>(1, 0) = laserCloud14Scans[i].points[li].y;
                P_lidar.at<double>(2, 0) = laserCloud14Scans[i].points[li].z;
                P_lidar.at<double>(3, 0) = 1;
                P_cam = mTcamlid * P_lidar;
//            std::cout<<P_lidar.t()<<endl;//            std::cout<<mTcamlid<<endl;//            std::cout<<P_cam.t()<<endl;//            std::cout<<"-----"<<endl;
                point3d newP;
                newP.x = P_cam.at<double>(0, 0);
                newP.y = P_cam.at<double>(1, 0);
                newP.z = P_cam.at<double>(2, 0);
                LiDARPoints_Cam.push_back(newP);
            }
        }
    }
}

/**
 * @brief Project LiDAR Points and Features (PCL pointset) to Camera coordination system
 */
void ProjectLiDARtoCam(pcl::PointCloud<PointType> LiDARFeatures, vector<point3d> &LiDARPoints_Cam) {
    cv::Mat mTcamlid = cv::Mat::eye(4, 4, CV_64F);
    mTcamlid.at<double>(0, 0) = Rcl00;
    mTcamlid.at<double>(0, 1) = Rcl01;
    mTcamlid.at<double>(0, 2) = Rcl02;
    mTcamlid.at<double>(0, 3) = Tcl0;
    mTcamlid.at<double>(1, 0) = Rcl10;
    mTcamlid.at<double>(1, 1) = Rcl11;
    mTcamlid.at<double>(1, 2) = Rcl12;
    mTcamlid.at<double>(1, 3) = Tcl1;
    mTcamlid.at<double>(2, 0) = Rcl20;
    mTcamlid.at<double>(2, 1) = Rcl21;
    mTcamlid.at<double>(2, 2) = Rcl22;
    mTcamlid.at<double>(2, 3) = Tcl2;
        int lsrPtNum = LiDARFeatures.size();
        if (lsrPtNum > 0) {
            cv::Mat P_lidar(4, 1, CV_64F);//3D LiDAR point
            cv::Mat P_cam(4, 1, CV_64F);//3D LiDAR point under Cam coordination
            int counter = 0;
            for (int li = 0; li < lsrPtNum; li++) {
                //Velodyne Vertical FOV 26.9 mounted on 1.73. At 6 meter-distance, tan(26.9/2). it can only detect ~ 1.52+1.73 height
                //X front, Y left, Z up
                //Todo Test a valid threshold?
                double maxX = 25.0, maxY = 25.0, minZ = 1.8;
                maxX = 50.0, maxY = 50.0, minZ = 50;
                if (LiDARFeatures.points[li].x > maxX || LiDARFeatures.points[li].x < 0.0
                    || LiDARFeatures.points[li].y > maxY || LiDARFeatures.points[li].y < -maxY
                    || LiDARFeatures.points[li].z > 20 || LiDARFeatures.points[li].z < -minZ) {
                    continue;
                }
                P_lidar.at<double>(0, 0) = LiDARFeatures.points[li].x;
                P_lidar.at<double>(1, 0) = LiDARFeatures.points[li].y;
                P_lidar.at<double>(2, 0) = LiDARFeatures.points[li].z;
                P_lidar.at<double>(3, 0) = 1;
                P_cam = mTcamlid * P_lidar;
//            std::cout<<P_lidar.t()<<endl;//            std::cout<<mTcamlid<<endl;//            std::cout<<P_cam.t()<<endl;//            std::cout<<"-----"<<endl;
                point3d newP;
                newP.x = P_cam.at<double>(0, 0);
                newP.y = P_cam.at<double>(1, 0);
                newP.z = P_cam.at<double>(2, 0);
                LiDARPoints_Cam.push_back(newP);
            }
        }
}
/*
 * LiDAR under Cam coordination
 * Project to Image
 */
void ProjectLiDARtoImg(int cols, int rows, vector<point3d> LiDARPoints, vector<point2d> &LiDARonImg) {
    //it seems like OpenCV upgraded, then the func changed?
    //cv::Mat P_rect_00 = cv::Mat::zeros(CvSize(4, 3), CV_64F);
    cv::Mat P_rect_00 = cv::Mat::zeros(3,4,CV_64F);
    P_rect_00.at<double>(0, 0) = fx;
    P_rect_00.at<double>(0, 2) = cx;
    P_rect_00.at<double>(1, 1) = fy;
    P_rect_00.at<double>(1, 2) = cy;
    P_rect_00.at<double>(2, 2) = 1;
    //cv::Mat R_rect_00 = cv::Mat::eye(CvSize(4, 4), CV_64F);
    cv::Mat R_rect_00 = cv::Mat::eye(4, 4, CV_64F);
    int ptNum = LiDARPoints.size();
    cv::Mat X(4, 1, CV_64F);//3D LiDAR point
    cv::Mat Y(3, 1, CV_64F);//2D LiDAR projection
    int counter = 0;
    for (int pi = 0; pi < ptNum; pi++) {
        //cv::Point pt;
        point2d pt;
        X.at<double>(0, 0) = LiDARPoints[pi].x;
        X.at<double>(1, 0) = LiDARPoints[pi].y;
        X.at<double>(2, 0) = LiDARPoints[pi].z;
        X.at<double>(3, 0) = 1;
        Y = P_rect_00 * R_rect_00 * X;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0);
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0);
        //why comment the checking will not lead system fail? dray point outside the image is okay for opencv?
//        if (pt.x < 0 || pt.x >= cols || pt.y < 0 || pt.y >= rows) {
//            //cout<<X.at<double>(0, 0)<<" "<<X.at<double>(1, 0)<<" "<<X.at<double>(2, 0)<<" -> "<<pt.x<<" "<<pt.y<<endl;
//            //mLaserPt_cam[pi].index2d = -1;
//            continue;
//        }
        //cout<<X.at<double>(0, 0)<<" "<<X.at<double>(1, 0)<<" "<<X.at<double>(2, 0)<<" -> "<<pt.x<<" "<<pt.y<<endl;
//        mLaserPt_cam[pi].pt2d = pt;
//        mLaserPt_cam[pi].index2d = counter;
        LiDARonImg.push_back(pt);
        counter++;
    }
    //Test
//    X.at<double>(0, 0) = 3;
//    X.at<double>(1, 0) = 1;
//    X.at<double>(2, 0) = 5;
//    Y = P_rect_00 * R_rect_00 * X;
//    cv::Point pt;
//    pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0);
//    pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0);
//    cout<<"(3,1,5) -> "<<pt.x<<" "<<pt.y<<endl;
    //TODO Check why the Projected LiDAR points are under some certain height
    cout << "Lidar points total : " << LiDARPoints.size() << " in image frame : " << counter << endl;
}

void drawImage(cv::Mat &im, vector<point2d> LiDAR2d, vector<point2d> ImageFeatures){
    for(int i = 0; i < LiDAR2d.size(); i ++)
    {
        cv::circle(im,cv::Point2d(LiDAR2d[i].x, LiDAR2d[i].y),2,cv::Scalar(0,255,0),-1);
    }
    for(int i = 0; i < ImageFeatures.size(); i ++)
    {
        cv::circle(im,cv::Point2d(ImageFeatures[i].x, ImageFeatures[i].y),2,cv::Scalar(255,0,0),-1);
    }
    cout<<"LiDAR2d size "<<LiDAR2d.size()<<endl;
}

int main() {
    std::cout << "Hello, World!" << std::endl;
    string LiDARAddress = "../000001.bin";
    string ImageAddress = "../000001.png";
    string KeyPtAddress = "../000001KeyPt.txt";
    ///Step 1 load point cloud
    vector<vector<float>> laserPoints;
    for (int i = 0; i < 1000000; i++) {
        vector<float> point = {0, 0, 0, 0};
        laserPoints.push_back(point);
    }
    readLaserPoints(LiDARAddress, laserPoints);
    cout << "LaserPoints point size " << laserPoints.size() << endl;
    ///Step 2 process point cloud
    //todo decrease to 16 lines
    ExtractLiDARFeature(laserPoints);
    ///added
    for (int i = 0; i < 16; i++) {
        cout << "line " << i << " size " << laserCloud14Scans[i].size() << endl;
    }
    ///Project 2 image
    cv::Mat im = cv::imread(ImageAddress, CV_LOAD_IMAGE_UNCHANGED);
    cout<<"image cols "<<im.cols<<" "<<im.rows<<endl;
    vector<point3d> LiDARPoints_Cam;//Lidar point under cam coor
    //ProjectLiDARtoCam(laserPoints, LiDARPoints_Cam);
    ProjectLiDARtoCam(laserCloud14Scans, LiDARPoints_Cam);
    //cout<<"LiDARPoints on Cam Size :"<<LiDARPoints_Cam.size()<<endl;

    vector<point2d> LiDAR2d;//Lidar point in image
    ProjectLiDARtoImg(im.cols, im.rows, LiDARPoints_Cam, LiDAR2d);
    ///Read Image Key pt
    vector<point2d> ImgKeyPoint;//Lidar point in image
    ifstream keyReader;
    keyReader.open("../000001KeyPt.txt");
    double x,y;
    int id = 0;
    while(keyReader>>x>>y){
        //cout<<"id "<<id<<": "<<x<<" "<<y<<endl;
        point2d newPt;
        newPt.x = x; newPt.y = y; newPt.id2d=id;
        id ++;
        ImgKeyPoint.push_back(newPt);
    }
    keyReader.close();
    ///Step 2.5 Canny and Lines
    cvtColor(im,im,CV_GRAY2BGR);

    cv::Mat dst,cdst,cdstP,dstP;
    // Edge detection
    Canny(im, dst, 50, 150, 3);
    // Copy edges to the images that will display the results in BGR
    cvtColor(dst, cdst, cv::COLOR_GRAY2BGR);
    cdstP = cdst.clone();
    // Standard Hough Line Transform
    vector<cv::Vec2f> lines; // will hold the results of the detection
    HoughLines(dst, lines, 1, CV_PI/180, 150, 0, 0 ); // runs the actual detection
    // Draw the lines
    for( size_t i = 0; i < lines.size(); i++ )
    {
        float rho = lines[i][0], theta = lines[i][1];
        cv::Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        line( cdst, pt1, pt2, cv::Scalar(0,0,255), 1, cv::LINE_AA);
    }
    // Probabilistic Line Transform
    vector<cv::Vec4i> linesP; // will hold the results of the detection
    HoughLinesP(dst, linesP, 1, CV_PI/180, 150, 50, 10 ); // runs the actual detection
    // Draw the lines
    int counter = 0;
    for( size_t i = 0; i < linesP.size(); i++ )
    {
        cv::Vec4i l = linesP[i];
        line( cdstP, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 1, cv::LINE_AA);
        double dist = (l[0]-l[2])*(l[0]-l[2]) + (l[1]-l[3])*(l[1]-l[3]);
        if(dist>10000){
            counter ++;
            line( im, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 1, cv::LINE_AA);
        }
    }
    cout<<"drawn "<<counter<<" lines in image"<<endl;
    imshow("Source", im);
    cv::waitKey(0);
    imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst);
    cv::waitKey(0);
    imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP);
    cv::waitKey(1);
    ///Step 3 show point cloud
    //todo thread show point clouds
    //cvtColor(im,im,CV_GRAY2BGR);
    drawImage(im,LiDAR2d,ImgKeyPoint);
    cv::imshow("ORB-SLAM2: Current Frame",im);
    cv::waitKey(0);
    ///show------------LiDAR-----------
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
    //while (!pangolin::ShouldQuit()) {
    while (cvWaitKey(1)) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);

        //需要绘制的东西写在这里
        //coordinate frame
        glBegin(GL_LINES);
        glLineWidth(5.0);
        glColor3f(1, 0, 0);
        glVertex3d(0.1, 0, 0);
        glVertex3d(0, 0, 0);
        glColor3f(0, 1, 0);
        glVertex3d(0, 0.1, 0);
        glVertex3d(0, 0, 0);
        glColor3f(0, 0, 1);
        glVertex3d(0, 0, 0.1);
        glVertex3d(0, 0, 0);
        glEnd();

        for (int li = 0; li < laserCloud14Scans.size(); li++) {
            glBegin(GL_POINTS);
            glColor3f(1, 1, 1); //60 20 220
            for (int ci = 0; ci < laserCloud14Scans[li].size(); ci++) {
                glVertex3d(laserCloud14Scans[li].points[ci].x, laserCloud14Scans[li].points[ci].y,
                           laserCloud14Scans[li].points[ci].z);
            }
            glEnd();
        }

//        for (int ci = 0; ci < laserPoints.size(); ci++) {
//            glBegin(GL_POINTS);
//            glColor3f(1, 1, 1); //60 20 220
//            glVertex3d(laserPoints[ci][0], laserPoints[ci][1], laserPoints[ci][2]);
//            glEnd();
//        }

//        for (int ci = 0; ci < mCornerPointsSharp.points.size(); ci++) {
//            if (mCornerPointsSharp.points[ci].x > 0) {
//                glBegin(GL_POINTS);
//                glColor3f(0.8627, 0.078, 0.2352); //60 20 220
//                glVertex3d(mCornerPointsSharp.points[ci].x, mCornerPointsSharp.points[ci].y,
//                           mCornerPointsSharp.points[ci].z);
//                glEnd();
//            }
//
//        }
//
//        for (int ci = 0; ci < mCornerPointsLessSharp.points.size(); ci++) {
//            if (mCornerPointsLessSharp.points[ci].x > 0) {
//                glBegin(GL_POINTS);
//                glColor3f(1, 0.4117, 0.7058); //180 105 255
//                glVertex3d(mCornerPointsLessSharp.points[ci].x, mCornerPointsLessSharp.points[ci].y,
//                           mCornerPointsLessSharp.points[ci].z);
//                glEnd();
//            }
//        }

//        for (int ci = 0; ci < mSurfPointsFlat.points.size(); ci++) {
//            glBegin(GL_POINTS);
//            glColor3f(0, 0, 1); //255 0 0
//            glVertex3d(mSurfPointsFlat.points[ci].x, mSurfPointsFlat.points[ci].y, mSurfPointsFlat.points[ci].z);
//            glEnd();
//        }

//        for (int ci = 0; ci < mSurfPointsLessFlat.points.size(); ci++) {
//            glBegin(GL_POINTS);
//            glColor3f(0, 1, 1); //255 255 0
//            glVertex3d(mSurfPointsLessFlat.points[ci].x, mSurfPointsLessFlat.points[ci].y,
//                       mSurfPointsLessFlat.points[ci].z);
//            glEnd();
//        }

        pangolin::FinishFrame();
    }
    ///-----------------------------------------
    return 0;
}
