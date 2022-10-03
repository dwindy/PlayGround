//
// Created by xin on 22/09/22.
//

#include "LiDAR_functions.h"


float cloudCurvature[400000];
int cloudSortInd[400000];
int cloudNeighborPicked[400000];
int cloudLabel[400000];
bool comp (int i,int j) { return (cloudCurvature[i]<cloudCurvature[j]); }
pcl::PointCloud<pcl::PointXYZI> mCornerPointsSharp;
pcl::PointCloud<pcl::PointXYZI> mCornerPointsLessSharp;
pcl::PointCloud<pcl::PointXYZI> mSurfPointsFlat;
pcl::PointCloud<pcl::PointXYZI> mSurfPointsLessFlat;
//std::vector<pcl::PointCloud<pcl::PointXYZI>> laserCloud16Scans(N_SCANS/4);
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
void ExtractLiDARFeature(vector<vector<float>> mLaserPoints, vector<pcl::PointCloud<pcl::PointXYZI>> laserCloud16Scans) {
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
    pcl::PointXYZI point;
    std::vector<pcl::PointCloud<pcl::PointXYZI>> laserCloudScans(N_SCANS);
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
            laserCloud16Scans[scanID/4].push_back(point);
        }
    }
    // cloudSize是有效的点云的数目
    cloudSize = count;
    printf("points size %d \n", cloudSize);

    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
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

    pcl::PointCloud <pcl::PointXYZI> cornerPointsSharp;
    pcl::PointCloud <pcl::PointXYZI> cornerPointsLessSharp;
    pcl::PointCloud <pcl::PointXYZI> surfPointsFlat;
    pcl::PointCloud <pcl::PointXYZI> surfPointsLessFlat;

    float t_q_sort = 0;
    // 遍历每个scan
    for (int i = 0; i < N_SCANS; i++) {
        if(i%4==0){
            cout<<"process line "<<i<<endl;
            // 没有有效的点了，就continue
            if (scanEndInd[i] - scanStartInd[i] < 6) //this scan soo small
                continue;
            // 用来存储不太平整的点
            pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<pcl::PointXYZI>);
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

            pcl::PointCloud<pcl::PointXYZI> surfPointsLessFlatScanDS;
            surfPointsLessFlatScanDS.resize(100000);
            pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
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
    printf("Corner Pt: %lu, Less Corner :%luu, Flat Pt: %lu, Less Flat Pt : %lu \n", mCornerPointsSharp.size(),
           mCornerPointsLessSharp.size(), mSurfPointsFlat.size(), mSurfPointsLessFlat.size());
    int pause = 1;
}