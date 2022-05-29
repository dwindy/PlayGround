/*
 * Design for test KDtree search
 */

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "tic_toc.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <pangolin/pangolin.h>
#include <pcl/io/pcd_io.h>


typedef pcl::PointXYZI PointType;
using namespace std;

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

/**
 * @brief Load Laser Scans
 * @param [in] strPathToSequence : the data folder address
 * @param [in,out] vstrLaserscanFilenames : the string vector contains each Scan File name
 * @param [in,out] vTimestamps : the double vector contains timestamps of each Scan file
 * @param [in,out] vTimestarts : the double vector contains start time of each Scan
 * @param [in,out] vTimeends : the double vector contains end time of each Scan
 */
void LoadLaserscans(const string &strPathToSequence, vector<string> &vstrLaserscanFilenames, vector<double> &vTimestamps, vector<double> &vTimestarts, vector<double> &vTimeends)
{
    //load scan times
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/timestamps_processed.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }
    fTimes.close();

    //load scan start times
    string strPathStartTimeFile = strPathToSequence + "/timestamps_start_processed.txt";
    fTimes.open(strPathStartTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestarts.push_back(t);
        }
    }
    fTimes.close();

    //load scan end times
    string strPathEndTimeFile = strPathToSequence + "/timestamps_end_processed.txt";
    fTimes.open(strPathEndTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimeends.push_back(t);
        }
    }
    fTimes.close();

    //load Laser Scan file names
    //string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixLeft = strPathToSequence + "/data/";

    const int nTimes = vTimestamps.size();
    vstrLaserscanFilenames.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        //ss << setfill('0') << setw(6) << i;
        ss << setfill('0') << setw(10) << i;
        vstrLaserscanFilenames[i] = strPrefixLeft + ss.str() + ".bin";
    }
}

template <typename PointT>
void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                            pcl::PointCloud<PointT> &cloud_out, float thres)
{
    if (&cloud_in != &cloud_out)
    {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }

    size_t j = 0;
    // 把点云距离小于给定阈值的去除掉
    for (size_t i = 0; i < cloud_in.points.size(); ++i)
    {
        if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)
            continue;
        cloud_out.points[j] = cloud_in.points[i];
        j++;
    }
    if (j != cloud_in.points.size())
    {
        cloud_out.points.resize(j);
    }

    cloud_out.height = 1; //orignal height is the scan line number
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;
}

/**
 * @brief Load laserpoint by given filename.
 * @param [in] vstrScanFilename : filename of laser scans
 * @param [in,out] laserPoints : laser points
 */
void readLaserPoints(string vstrScanFilename, vector<vector<float>> &laserPoints)
{
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
    fstream = fopen(vstrScanFilename.c_str(), "rb");
    num = fread(data, sizeof(float), num, fstream)/4;
    for(int i=0; i<num;i++)
    {
        laserPoints[i][0] = *px;
        laserPoints[i][1] = *py;
        laserPoints[i][2] = *pz;
        laserPoints[i][3] = *pr;
//        cout<<"load 1 "<<*px<<" "<<*py<<" "<<*pz<<" "<<*pr<<" "<<endl;
//        cout<<"load 2 "<<laserPoints[i][0]<<" "<<laserPoints[i][1]<<" "<<laserPoints[i][2]<<" "<<laserPoints[i][3]<<" "<<endl;
        px+=4;py+=4;pz+=4;pr+=4;
    }
    fclose(fstream);
    //reset laserpoint vector size
    laserPoints.resize(num);
    cout<<"laserPoints load "<<num<<" LiDAR points "<<endl;
    ///Step2 why i wrote Step2, is there supposed to have a step2?
}

/**
 * @brief Under LiDAR coordination, this function extract edge feature and plane feature from LiDAR source
 * @param[]
 */
void ExtractLiDARFeature(vector<vector<float>> mLaserPoints){
    ///Step1 : fetch 3d lidar points from mLaserPoints (lidar coordination)
    TicToc t_whole;
    TicToc t_prepare;
    std::vector<int> scanStartInd(N_SCANS, 0);
    std::vector<int> scanEndInd(N_SCANS, 0);
    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
    std::vector<int> indices;
    int lsrPtNum = mLaserPoints.size();
    laserCloudIn.resize(lsrPtNum);
    for(int i =0;i<lsrPtNum;i++){
        laserCloudIn.points[i].x = mLaserPoints[i][0];
        laserCloudIn.points[i].y = mLaserPoints[i][1];
        laserCloudIn.points[i].z = mLaserPoints[i][2];
//        if(abs(mLaserPoints[i][1])<=0.3){
//            cout<<laserCloudIn.points[i].x<<" "<<laserCloudIn.points[i].y<<" "<<laserCloudIn.points[i].z<<endl;
//        }
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
    }
    else if (endOri - startOri < M_PI) //start 179, ends -179 degree
    {
        endOri += 2 * M_PI;
    }
    //printf("end Ori %f\n", endOri);
    bool halfPassed = false;
    int count = cloudSize;
    PointType point;
    std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);
    // 遍历每一个点
    for (int i = 0; i < cloudSize; i++)
    {
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;
        // 计算他的俯仰角
        //cout<<"atan "<<atan(point.z / sqrt(point.x * point.x + point.y * point.y))<<endl;
        float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
        int scanID = 0;
        // 计算是第几根scan
        if (N_SCANS == 16)
        {
            scanID = int((angle + 15) / 2 + 0.5);
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else if (N_SCANS == 32)
        {
            scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else if (N_SCANS == 64)
        {
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
        }
        else
        {
            printf("wrong scan number\n");
            //ROS_BREAK();
        }
        //printf("angle %f scanID %d \n", angle, scanID);
        // 计算水平角
        float ori = -atan2(point.y, point.x);
        if (!halfPassed)
        {
            // 确保-PI / 2 < ori - startOri < 3 / 2 * PI
            if (ori < startOri - M_PI / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > startOri + M_PI * 3 / 2)
            {
                ori -= 2 * M_PI;
            }
            // 如果超过180度，就说明过了一半了 //half passed the start point
            if (ori - startOri > M_PI)
            {
                halfPassed = true;
            }
        }
        else
        {
            // 确保-PI * 3 / 2 < ori - endOri < PI / 2
            ori += 2 * M_PI;    // 先补偿2PI
            if (ori < endOri - M_PI * 3 / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > endOri + M_PI / 2)
            {
                ori -= 2 * M_PI;
            }
        }
        // 角度的计算是为了计算相对的起始时刻的时间
        float relTime = (ori - startOri) / (endOri - startOri);
        // 整数部分是scan的索引，小数部分是相对起始时刻的时间
        point.intensity = scanID + scanPeriod * relTime;
        // 根据scan的idx送入各自数组
        laserCloudScans[scanID].push_back(point);
    }
    // cloudSize是有效的点云的数目
    cloudSize = count;
    printf("points size %d \n", cloudSize);

    pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
    // 全部集合到一个点云里面去，但是使用两个数组标记其实和结果，这里分别+5和-6是为了计算曲率方便
    for (int i = 0; i < N_SCANS; i++)
    {
        scanStartInd[i] = laserCloud->size() + 5; //most left 5 and right 6 didn't count curve value
        *laserCloud += laserCloudScans[i];
        scanEndInd[i] = laserCloud->size() - 6;
    }

    //printf("prepare time %f \n", t_prepare.toc());
    // 开始计算曲率
    for (int i = 5; i < cloudSize - 5; i++)
    {
        float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x + laserCloud->points[i + 5].x;
        float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y + laserCloud->points[i + 5].y;
        float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z + laserCloud->points[i + 5].z;
        // 存储曲率，索引
        cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
        cloudSortInd[i] = i;
        cloudNeighborPicked[i] = 0;
        cloudLabel[i] = 0;
    }

    TicToc t_pts;

    pcl::PointCloud<PointType> cornerPointsSharp;
    pcl::PointCloud<PointType> cornerPointsLessSharp;
    pcl::PointCloud<PointType> surfPointsFlat;
    pcl::PointCloud<PointType> surfPointsLessFlat;

    float t_q_sort = 0;
    // 遍历每个scan
    for (int i = 0; i < N_SCANS; i++)
    {
        // 没有有效的点了，就continue
        if( scanEndInd[i] - scanStartInd[i] < 6) //this scan soo small
            continue;
        // 用来存储不太平整的点
        pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);
        // 将每个scan等分成6等分
        for (int j = 0; j < 6; j++)
        {
            // 每个等分的起始和结束点
            int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6;
            int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;

            TicToc t_tmp;
            // 对点云按照曲率进行排序，小的在前，大的在后
            std::sort (cloudSortInd + sp, cloudSortInd + ep + 1, comp);
            t_q_sort += t_tmp.toc();

            int largestPickedNum = 0;
            // 挑选曲率比较大的部分
            for (int k = ep; k >= sp; k--)
            {
                // 排序后顺序就乱了，这个时候索引的作用就体现出来了
                int ind = cloudSortInd[k];

                // 看看这个点是否是有效点，同时曲率是否大于阈值
                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] > 0.1)
                {

                    largestPickedNum++;
                    // 每段选2个曲率大的点
                    if (largestPickedNum <= 2)
                    {
                        // label为2是曲率大的标记
                        cloudLabel[ind] = 2;
                        // cornerPointsSharp存放大曲率的点
                        cornerPointsSharp.push_back(laserCloud->points[ind]);
//                        if(abs(laserCloud->points[ind].y)<=0.5 && laserCloud->points[ind].x <10 && laserCloud->points[ind].x > 0)
//                            cout<<"cloudCurvature "<<laserCloud->points[ind].x<<" "<<laserCloud->points[ind].y<<" "<<laserCloud->points[ind].z<<" "<<cloudCurvature[ind]<<endl;
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                    }
                        // 以及20个曲率稍微大一些的点
                    else if (largestPickedNum <= 20)
                    {
                        // label置1表示曲率稍微大
                        cloudLabel[ind] = 1;
//                        if(abs(laserCloud->points[ind].y)<=0.5 && laserCloud->points[ind].x <10 && laserCloud->points[ind].x > 0)
//                            cout<<"cloudCurvature "<<laserCloud->points[ind].x<<" "<<laserCloud->points[ind].y<<" "<<laserCloud->points[ind].z<<" "<<cloudCurvature[ind]<<endl;
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                    }
                        // 超过20个就算了
                    else
                    {
                        break;
                    }
                    // 这个点被选中后 pick标志位置1
                    cloudNeighborPicked[ind] = 1;
                    // 为了保证特征点不过度集中，将选中的点周围5个点都置1,避免后续会选到
                    for (int l = 1; l <= 5; l++)
                    {
                        // 查看相邻点距离是否差异过大，如果差异过大说明点云在此不连续，是特征边缘，就会是新的特征，因此就不置位了
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }
                        cloudNeighborPicked[ind + l] = 1;
                    }
                    // 下面同理
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }
                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }
            // 下面开始挑选面点
            int smallestPickedNum = 0;
            for (int k = sp; k <= ep; k++)
            {
                int ind = cloudSortInd[k];
                // 确保这个点没有被pick且曲率小于阈值
                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] < 0.1)
                {
                    // -1认为是平坦的点
                    cloudLabel[ind] = -1;
                    surfPointsFlat.push_back(laserCloud->points[ind]);

                    smallestPickedNum++;
                    // 这里不区分平坦和比较平坦，因为剩下的点label默认是0,就是比较平坦
                    if (smallestPickedNum >= 4)
                    {
                        break;
                    }
                    // 下面同理
                    cloudNeighborPicked[ind] = 1;
                    for (int l = 1; l <= 5; l++)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }
                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }
                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }
            //Less Flat is default lable 0
            for (int k = sp; k <= ep; k++)
            {
                // 这里可以看到，剩下来的点都是一般平坦，这个也符合实际
                if (cloudLabel[k] <= 0)
                {
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
    //printf("sort q time %f \n", t_q_sort);
    //printf("seperate points time %f \n", t_pts.toc());

    ///Step 3 : pass features to frame member
    mCornerPointsSharp.resize(cornerPointsSharp.size());
    mCornerPointsSharp=cornerPointsSharp;
    mCornerPointsLessSharp.resize(cornerPointsLessSharp.size());
    mCornerPointsLessSharp=cornerPointsLessSharp;
    mSurfPointsFlat.resize(surfPointsFlat.size());
    mSurfPointsFlat=surfPointsFlat;
    mSurfPointsLessFlat.resize(surfPointsLessFlat.size());
    mSurfPointsLessFlat=surfPointsLessFlat;
    printf("Corner Pt: %u, Less Corner : %u, Flat Pt: %u, Less Flat Pt : %u \n", mCornerPointsSharp.size(),mCornerPointsLessSharp.size(),mSurfPointsFlat.size(),mSurfPointsLessFlat.size());
    int pause = 1;
}

int main(int argc, char **argv) {
    std::cout << "Hello, World!" << std::endl;
    if(argc != 2)
    {
        cerr << endl << "Usage: ./mono_kitti path_to_scan_sequence" << endl;
        return 1;
    }
    ///Added All Laser file names and timestamps
    //Retrieve paths to Laser Scans
    vector<string> vstrScanFilenames;
    vector<double> vLaserTimestamps;
    vector<double> vLaserStartTimes;
    vector<double> vLaserEndTimes;
    LoadLaserscans(string(argv[1]), vstrScanFilenames, vLaserTimestamps, vLaserStartTimes, vLaserEndTimes);

    ///Frame by Frame read Laser
    int frameNum = vstrScanFilenames.size();
    vector<vector<float>> preFrameLaserPoints;
    for(int i = 0 ; i < frameNum ; i++){
        cout<<"Frame Number "<<i<<endl;
        //Load scans of this frame
        vector<vector<float>> laserPoints;
        //1000000 is the KITTI readme file suggested number
        for(int j = 0; j<1000000; j++)
        {
            vector<float> point = {0,0,0,0};
            laserPoints.push_back(point);
            if(j%300000==0)
                printf("read %d points ",j);
        }
        readLaserPoints(vstrScanFilenames[i], laserPoints);
        ExtractLiDARFeature(laserPoints);

        //test kdtree
        if (i > 0) {
            pcl::PointCloud<PointType>::Ptr cornerPointsLastFrame(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr cornerPointsCurrentFrame(new pcl::PointCloud<PointType>());
            cornerPointsLastFrame->clear();
            cornerPointsCurrentFrame->clear();
            int preSize = preFrameLaserPoints.size();
            int curSize = laserPoints.size();
            cornerPointsLastFrame->resize(preSize);
            cornerPointsCurrentFrame->resize(curSize);
            for (int j = 0; j < preFrameLaserPoints.size(); j++) {
                cornerPointsLastFrame->points[j].x = preFrameLaserPoints[j][0];
                cornerPointsLastFrame->points[j].y = preFrameLaserPoints[j][1];
                cornerPointsLastFrame->points[j].z = preFrameLaserPoints[j][2];
                cornerPointsLastFrame->points[j].intensity = preFrameLaserPoints[j][3];
            }
            //fake dataset
//            preSize = 12000;
//            cornerPointsLastFrame->resize(preSize);
//            for (int j = 0; j < preSize; j++) {
//                cornerPointsLastFrame->points[j].x = j;
//                cornerPointsLastFrame->points[j].y = j;
//                cornerPointsLastFrame->points[j].z = j;
//                cornerPointsLastFrame->points[j].intensity = j+10;
//            }
            cout<<"cornerPointsLastFrame size"<<cornerPointsLastFrame->size()<<endl;
            for (int j = 0; j < laserPoints.size(); j++) {
                cornerPointsCurrentFrame->points[j].x = laserPoints[j][0];
                cornerPointsCurrentFrame->points[j].y = laserPoints[j][1];
                cornerPointsCurrentFrame->points[j].z = laserPoints[j][2];
                cornerPointsCurrentFrame->points[j].intensity = laserPoints[j][3];
            }
            cout<<"cornerPointsCurrentFrame size"<<cornerPointsCurrentFrame->size()<<endl;
            pcl::KdTreeFLANN<pcl::PointXYZI> kdtreeCornerLast; //(new pcl::KdTreeFLANN<pcl::PointXYZI>());
            kdtreeCornerLast.setInputCloud(cornerPointsLastFrame);
            pcl::PointXYZI pointSel;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;
            //pointSel.x = 5;pointSel.y = 1;pointSel.z = -1.5;//fake point. to search for cloest points

            kdtreeCornerLast.nearestKSearch(cornerPointsLastFrame->points[15], 10, pointSearchInd, pointSearchSqDis);
            cout<<"search for "<<cornerPointsLastFrame->points[15].x<<" "<<cornerPointsLastFrame->points[15].y<<" "<<cornerPointsLastFrame->points[15].z<<" "<<endl;
            cout<<"found "<<pointSearchInd.size()<<endl;
            for (int j = 0; j < pointSearchInd.size(); j++) {
                cout << "SearchID "<< pointSearchInd[j] << " : " << cornerPointsLastFrame->points[pointSearchInd[j]].x<<" "
                     << cornerPointsLastFrame->points[pointSearchInd[j]].y << " "
                     << cornerPointsLastFrame->points[pointSearchInd[j]].z << " "
                     << " dis "<<pointSearchSqDis[pointSearchInd[j]] << endl;
            }
        }
        //store the previous
        for (int j = 0; j < laserPoints.size(); j++) {
            preFrameLaserPoints.push_back(laserPoints[j]);
        }
//        ///show-----------------------
//        pangolin::CreateWindowAndBind("Main", 640, 480);
//        glEnable(GL_DEPTH_TEST);
//        glEnable(GL_BLEND);
//        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//
//        pangolin::OpenGlRenderState s_cam(
//                pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 320, 0.2, 100),
//                pangolin::ModelViewLookAt(2, 0, 2, 0, 0, 0, pangolin::AxisY)
//        );
//
//        pangolin::Handler3D handler(s_cam);
//        pangolin::View &d_cam = pangolin::CreateDisplay()
//                .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f / 480.0f)
//                .SetHandler(&handler);
//        //while (!pangolin::ShouldQuit()) {
//        while (cvWaitKey(0)) {
//            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//            d_cam.Activate(s_cam);
//
//            //需要绘制的东西写在这里
//            //coordinate frame
//            glBegin(GL_LINES);
//            glLineWidth(5.0);
//            glColor3f(1, 0, 1);
//            glVertex3d(0.1, 0, 0);
//            glVertex3d(0, 0, 0);
//            glColor3f(0, 1, 0);
//            glVertex3d(0, 0.1, 0);
//            glVertex3d(0, 0, 0);
//            glColor3f(0, 0, 1);
//            glVertex3d(0, 0, 0.1);
//            glVertex3d(0, 0, 0);
//            glEnd();
//
//            for (int ci = 0; ci < mCornerPointsSharp.points.size(); ci++) {
//                glBegin(GL_POINTS);
//                glColor3f(0.8627, 0.078, 0.2352); //60 20 220
//                glVertex3d(mCornerPointsSharp.points[ci].x, mCornerPointsSharp.points[ci].y, mCornerPointsSharp.points[ci].z);
//                glEnd();
//            }
//
//            for (int ci = 0; ci < mCornerPointsLessSharp.points.size(); ci++) {
//                glBegin(GL_POINTS);
//                glColor3f(1, 0.4117, 0.7058); //180 105 255
//                glVertex3d(mCornerPointsLessSharp.points[ci].x, mCornerPointsLessSharp.points[ci].y, mCornerPointsLessSharp.points[ci].z);
//                glEnd();
//            }
//
//            for (int ci = 0; ci < mSurfPointsFlat.points.size(); ci++) {
//                glBegin(GL_POINTS);
//                glColor3f(0, 0, 1); //255 0 0
//                glVertex3d(mSurfPointsFlat.points[ci].x, mSurfPointsFlat.points[ci].y, mSurfPointsFlat.points[ci].z);
//                glEnd();
//            }
//
//            for (int ci = 0; ci < mSurfPointsLessFlat.points.size(); ci++) {
//                glBegin(GL_POINTS);
//                glColor3f(0, 1, 1); //255 255 0
//                glVertex3d(mSurfPointsLessFlat.points[ci].x, mSurfPointsLessFlat.points[ci].y, mSurfPointsLessFlat.points[ci].z);
//                glEnd();
//            }
//
//            pangolin::FinishFrame();
//        }
//        ///-----------------------------------------
    }

    return 0;
}


        //Play with simple kdtree code
//        pcl::PointCloud<PointType>::Ptr group1(new pcl::PointCloud<PointType>());
//        pcl::PointCloud<PointType>::Ptr group2(new pcl::PointCloud<PointType>());
//        group1->clear();
//        group2->clear();
//        group1->resize(3);
//        group2->resize(3);
//        cout<<endl;
//        for (size_t i = 0; i < 3; i++) {
//            group1->points[i].x = i;
//            group1->points[i].y = i;
//            group1->points[i].z = i;
//            group1->points[i].intensity = i + 10;
//            cout<<group1->points[i].x<<" "<<group1->points[i].y<<" "<<group1->points[i].z<<" "<<group1->points[i].intensity<<endl;
//            group2->points[i].x = i + 1;
//            group2->points[i].y = i + 1;
//            group2->points[i].z = i + 1;
//            group2->points[i].intensity = i + 100;
//        }
//        pcl::KdTreeFLANN<pcl::PointXYZI> kdtreeplay;
//        kdtreeplay.setInputCloud(group1);
//
//        pcl::PointXYZI point2match;
//        std::vector<int> pointSearchID;
//        std::vector<float> pointSearchSQDis;
//        point2match.x = 5;point2match.y = 1;point2match.z = -1.5;
//        kdtreeplay.nearestKSearch(point2match, 3, pointSearchID, pointSearchSQDis);
//        cout<<"found "<<pointSearchID.size()<<endl;
//        for (int j = 0; j < pointSearchID.size(); j++) {
//            cout << "SearchID "<< pointSearchID[j] << " : " << group1->points[pointSearchID[j]].x<<" "
//                 << group1->points[pointSearchID[j]].y << " "
//                 << group1->points[pointSearchID[j]].z << " "
//                 << " dis "<<pointSearchSQDis[j] << endl;
//        }