//
// Created by xin on 22/09/22.
//
#include "tool_functions.h"

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

/**
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
        pt.index2line = -1;
        pt.id2d = pi;
        pt.indexScani = LiDARPoints[pi].indexScani;
        pt.indexScanj = LiDARPoints[pi].indexScanj;
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

/**
 * @brief Project LiDAR Points and Features (PCL pointset) to Camera coordination system
 */
void ProjectLiDARtoCam(pcl::PointCloud<pcl::PointXYZI> LiDARFeatures, vector<point3d> &LiDARPoints_Cam, vector<point3d> &GroundPoints_Cam) {
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
            //newP.intensity?
            LiDARPoints_Cam.push_back(newP);
            if(newP.z<-1.7)
                GroundPoints_Cam.push_back(newP);
        }
    }
}

/**
 * @brief Project LiDAR Points and Features (PCL pointset) to Camera coordination system
 */
void ProjectLiDARtoCam(std::vector<pcl::PointCloud<pcl::PointXYZI>> laserCloud14Scans, vector<point3d> &LiDARPoints_Cam, vector<point3d> &GroundPoints_Cam) {
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
                newP.indexScani = i;
                newP.indexScanj = li;
                LiDARPoints_Cam.push_back(newP);
                if(laserCloud14Scans[i].points[li].z<-1.7)
                    GroundPoints_Cam.push_back(newP);
            }
        }
    }
}

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

