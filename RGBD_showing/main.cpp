///read pointcloud stored by ORBSLAM2
/*
 * record parameters
 * for no noise data
 * sor.setLeafSize(0.03f, 0.03f, 0.03f);
 * seg.setDistanceThreshold(0.001);
 *     normal_estimator.setKSearch(100);
    reg.setMinClusterSize(200);
    reg.setMaxClusterSize(50000);
    reg.setNumberOfNeighbours(100);//too little will cause runtime error
    reg.setSmoothnessThreshold(2.0 / 180.0 / M_PI);
    reg.setCurvatureThreshold(2.0);
 */
#include <iostream>
#include<opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <pangolin/pangolin.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>
#include <opencv2/highgui.hpp>


using namespace std;

// 定义点云类型
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// 相机内参
const double camera_factor = 5000;
const double camera_cx = 319.5;
const double camera_cy = 239.5;
const double camera_fx = 481.2;
const double camera_fy = -480.0;

class PtLsr {
public:
    cv::Point2d pt2d;
    cv::Point3d pt3d;
    int index2d;
    int index3d;
    int response;
};

class Plane {
public:
    double A, B, C, D;
    double phi, theta, dis;
    int count;
    int PlaneId;//Id in frame
    int planeIdGlobal;//In in map/global
    Plane(double Ain, double Bin, double Cin, double Din) : A(Ain), B(Bin), C(Cin), D(Din) { PlaneId = -1; }

    Plane(double phiin, double thetain, double disin) : phi(phiin), theta(thetain), dis(disin) { PlaneId = -1; }

    Plane() { PlaneId = -1; }

    cv::Point3d centreP;
//        vector<cv::Point3d> pointList;
//        vector<cv::Point2d> pointList2D;
    vector<PtLsr> points3D;
    vector<int> keyPointList; //todo store keypoint in this plane
    vector<int> mindices;     //todo indexs of keypoint in image
};

int RANSACPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, Plane &foundPlane, pcl::PointIndices &inliersOutput) {
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
    seg.setDistanceThreshold(0.01);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
        return 0;
    inliersOutput = *inliers;
    foundPlane.A = coefficients->values[0];
    foundPlane.B = coefficients->values[1];
    foundPlane.C = coefficients->values[2];
    foundPlane.D = coefficients->values[3];
    //cout<<" coefficients : "<<coefficients->values[0]<<" "<<coefficients->values[1]<<" "<<coefficients->values[2]<<" "<<coefficients->values[3]<<endl;
    double sumX = 0, sumY = 0, sumZ = 0;
    for (int i = 0; i < inliers->indices.size(); i++) {
        double x = cloud->points[inliers->indices[i]].x;
        double y = cloud->points[inliers->indices[i]].y;
        double z = cloud->points[inliers->indices[i]].z;
        sumX += x;
        sumY += y;
        sumZ += z;
        cv::Point3d newP = cv::Point3d(x, y, z);
        PtLsr newPtlsr;
        newPtlsr.pt3d = newP;
        foundPlane.points3D.push_back(newPtlsr);
        //foundPlane.points3D.push_back(cv::Point3d(x,y,z));
        //cout<<"inliner push back "<<x<<" "<<y<<" "<<z<<endl;
    }
    foundPlane.centreP = cv::Point3d(sumX / inliers->indices.size(), sumY / inliers->indices.size(),
                                     sumZ / inliers->indices.size());
    return inliers->indices.size();
}

void loadDepthImage(string RGBaddress, string Depthaddress, int maxLoadNum, cv::Mat& rgb, cv::Mat& depth, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2, pcl::PointCloud<pcl::PointXYZ>::Ptr RGBDCloud2){
    //read from png file
    rgb = cv::imread(RGBaddress);
    depth = cv::imread(Depthaddress, -1);
    //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGBA>);
    int actualNum2 = 0;
    //pcl::PointCloud<pcl::PointXYZ>::Ptr RGBDCloud2(new pcl::PointCloud<pcl::PointXYZ>);
    RGBDCloud2->resize(maxLoadNum);
    for(int m = 0; m < depth.rows;m++){
        for(int n = 0; n < depth.cols;n++){
            ushort d = depth.ptr<ushort>(m)[n];
            if(d <= 0)
                continue;
            pcl::PointXYZRGBA p;
            float d_de = double(d) /camera_factor;
            p.z = d_de;
            p.x = (n - camera_cx) * p.z / camera_fx;
            p.y = (m - camera_cy) * p.z / camera_fy;
            p.b = rgb.ptr<uchar>(m)[n * 3];
            p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
            p.r = rgb.ptr<uchar>(m)[n * 3 + 2];
            cloud2->points.push_back(p);
            RGBDCloud2->points[actualNum2].x = p.x;
            RGBDCloud2->points[actualNum2].y = p.y;
            RGBDCloud2->points[actualNum2].z = p.z;
            actualNum2++;
        }
    }
    RGBDCloud2->resize(actualNum2);
    cloud2->height = 1;
    cloud2->width = cloud2->points.size();
    cout << "point cloud size=" << cloud2->points.size() << endl;
//    cloud2->is_dense = false;
//    pcl::io::savePCDFile("./pointcloud.pcd", *cloud2);
//    //清除数据并保存
//    cloud2->points.clear();
//    cout << "Point cloud saved." << endl;
}

// 主函数
int main(int argc, char **argv) {
    ///Read from txt file
//    ifstream reader;
//    reader.open("initCloud.txt", ios::in);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr RGBDCloud(new pcl::PointCloud<pcl::PointXYZ>);
//    RGBDCloud->resize(500000);
//    int actualNum = 0;
//    while (!reader.eof()) {
//        reader >> RGBDCloud->points[actualNum].x >> RGBDCloud->points[actualNum].y >> RGBDCloud->points[actualNum].z;
//        actualNum++;
//    }
//    RGBDCloud->resize(actualNum);
//    cout << "read " << RGBDCloud->points.size() << " points" << endl;

    ///Read from png image
//    string rgbAddress = "/home/xin/Downloads/DATASET/ICL-NUIM/living_room_traj0n_frei_png/rgb/900.png";
//    string depthAddress = "/home/xin/Downloads/DATASET/ICL-NUIM/living_room_traj0n_frei_png/depth_planarreconstruct/800.png";
//    string rgbAddress = "/home/xin/Downloads/DATASET/TUM/structure/rgbd_dataset_freiburg3_structure_texture_far/rgb/1341839113.657637.png";
//    string depthAddress = "/home/xin/Downloads/DATASET/TUM/structure/rgbd_dataset_freiburg3_structure_texture_far/depth/1341839113.657663.png";
//    string rgbAddress = "/home/xin/Downloads/DATASET/TUM/structure/rgbd_dataset_freiburg3_structure_texture_far/rgb/1341839145.329591.png";
//    string depthAddress = "/home/xin/Downloads/DATASET/TUM/structure/rgbd_dataset_freiburg3_structure_texture_far/depth/1341839145.329622.png";
    string rgbAddress = "/home/xin/Downloads/DATASET/TUM/3d_object_reconstruct/rgbd_dataset_freiburg3_cabinet/rgb/1341841281.622633.png";
    string depthAddress = "/home/xin/Downloads/DATASET/TUM/3d_object_reconstruct/rgbd_dataset_freiburg3_cabinet/depth/1341841281.622658.png";
    cv::Mat rgb, depth;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr RGBDCloud(new pcl::PointCloud<pcl::PointXYZ>);
    loadDepthImage(rgbAddress, depthAddress, 500000, rgb, depth, cloud, RGBDCloud);

    cv::Mat imD;
    imD = cv::imread(depthAddress,  CV_LOAD_IMAGE_UNCHANGED);
    cout<<imD.at<float>(100,100)<<" "<<imD.at<float>(100,101)<<" "<<imD.at<float>(100,102)<<endl;
    imD.convertTo(imD,CV_32F,5000);
    cout<<imD.at<float>(100,100)<<" "<<imD.at<float>(100,101)<<" "<<imD.at<float>(100,102)<<endl;
    cv::imshow("depth1",imD);
    cv::waitKey(0);

//    string rgbAddress2 = "/home/xin/Downloads/DATASET/ICL-NUIM/living_room_traj0n_frei_png/rgb/901.png";
//    string depthAddress2 = "/home/xin/Downloads/DATASET/ICL-NUIM/living_room_traj0n_frei_png/depth_planarreconstruct/801.png";
    string rgbAddress2 = "/home/xin/Downloads/DATASET/TUM/structure/rgbd_dataset_freiburg3_structure_texture_far/rgb/1341839113.693575.png";
    string depthAddress2 = "/home/xin/Downloads/DATASET/TUM/structure/rgbd_dataset_freiburg3_structure_texture_far/depth/1341839113.693593.png";
    cv::Mat rgb2, depth2;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr RGBDCloud2(new pcl::PointCloud<pcl::PointXYZ>);
    loadDepthImage(rgbAddress2,depthAddress2,500000,rgb2,depth2,cloud2,RGBDCloud2);

    ///Downsampling pointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr RGBDCloudDownSample(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(RGBDCloud);
    sor.setLeafSize(0.03f, 0.03f, 0.03f);
    sor.filter(*RGBDCloudDownSample);
    cout << " downsampling remains " << RGBDCloudDownSample->points.size() << endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr RGBDCloudDownSample2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor2;
    sor.setInputCloud(RGBDCloud2);
    sor.setLeafSize(0.03f, 0.03f, 0.03f);
    sor.filter(*RGBDCloudDownSample2);
    cout << " downsampling remains " << RGBDCloudDownSample2->points.size() << endl;

    ///show----------------------- Downsample
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
//    while (!pangolin::ShouldQuit()) {
//        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//        d_cam.Activate(s_cam);
//
//        //需要绘制的东西写在这里
//        //coordinate frame
//        glBegin(GL_LINES);
//        glLineWidth(5.0);
//        glColor3f(1, 0, 1);
//        glVertex3d(0.1, 0, 0);
//        glVertex3d(0, 0, 0);
//        glColor3f(0, 1, 0);
//        glVertex3d(0, 0.1, 0);
//        glVertex3d(0, 0, 0);
//        glColor3f(0, 0, 1);
//        glVertex3d(0, 0, 0.1);
//        glVertex3d(0, 0, 0);
//        glEnd();
//        ////downsample points
//        for (int ci = 0; ci < RGBDCloudDownSample->points.size(); ci++) {
//            glBegin(GL_POINTS);
//            glColor3f(1, 0, 0);
//            glVertex3d(RGBDCloudDownSample->points[ci].x, RGBDCloudDownSample->points[ci].y, RGBDCloudDownSample->points[ci].z);
//            glEnd();
//        }
////        for (int ci = 0; ci < RGBDCloudDownSample->points.size(); ci++) {
////            glBegin(GL_POINTS);
////            glColor3f(0, 1, 0);
////            glVertex3d(RGBDCloudDownSample2->points[ci].x, RGBDCloudDownSample2->points[ci].y, RGBDCloudDownSample2->points[ci].z);
////            glEnd();
////        }
//        ////all points
////        for (int ci = 0; ci < RGBDCloud->points.size(); ci++) {
////            glBegin(GL_POINTS);
////            glColor3f(0, 1, 0);
////            glVertex3d(RGBDCloud->points[ci].x, RGBDCloud->points[ci].y, RGBDCloud->points[ci].z);
////            glEnd();
////        }
////        for (int ci = 0; ci < RGBDCloud2->points.size(); ci++) {
////            glBegin(GL_POINTS);
////            glColor3f(1, 0, 0);
////            glVertex3d(RGBDCloud2->points[ci].x, RGBDCloud2->points[ci].y, RGBDCloud2->points[ci].z);
////            glEnd();
////        }
//        pangolin::FinishFrame();
//    }
    ///-----------------------------------------

    ///estimating normals for each point
    pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(RGBDCloudDownSample);//RGBDCloud //RGBDCloudDownSample
    normal_estimator.setKSearch(200);
    normal_estimator.compute(*normals);
    ///region growing
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize(100);
    reg.setMaxClusterSize(10000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(200);//too little will cause runtime error
    reg.setInputCloud(RGBDCloudDownSample);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(5 / 180.0 / M_PI);
    reg.setCurvatureThreshold(5);
    ///extract each cluster
    clock_t startTime = clock();
    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);
    clock_t endTime = clock();
    double timeUsed = double(endTime - startTime) / CLOCKS_PER_SEC;
    cout << "region growing " << timeUsed << " sec " << clusters.size() << " clusters " << endl;
    ///Call RANSAC plane fitting for each Cluster
    vector<Plane> planes;
    vector<pcl::PointIndices> inliersOfEachPlane; //store the index of inliners
    vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pointsOfEachPlane; //used to run RANSAC
    vector<std::vector<double>> ClosePoints; //CP representation of Plane
    for (int ci = 0; ci < clusters.size(); ci++) {
        //generate a temporary pointcloud and fill it up
        pcl::PointCloud<pcl::PointXYZ>::Ptr thisCloud(new pcl::PointCloud<pcl::PointXYZ>);
        thisCloud->points.resize(clusters[ci].indices.size());
        thisCloud->height = 1;
        thisCloud->width = clusters[ci].indices.size();
        //cout << " | cluster contains " << clusters[ci].indices.size();
        for (int index = 0; index < clusters[ci].indices.size(); index++) {
            int pointIndex = clusters[ci].indices[index];
            thisCloud->points[index].x = RGBDCloudDownSample->points[pointIndex].x;
            thisCloud->points[index].y = RGBDCloudDownSample->points[pointIndex].y;
            thisCloud->points[index].z = RGBDCloudDownSample->points[pointIndex].z;
            //cout<<" cluster point "<<thisCloud->points[index].x<<" "<<thisCloud->points[index].y<<" "<<thisCloud->points[index].z<<endl;
            //cout<<pointIndex<<endl;
        }
        pointsOfEachPlane.push_back(thisCloud);
        Plane foundPlane;
        startTime = clock();
        pcl::PointIndices inliersOUT;
        int intPlaneNum = RANSACPlane(thisCloud, foundPlane, inliersOUT);
        endTime = clock();
        cout<<"inliers-----"<<endl;
//        for (int i = 0; i < inliersOUT.indices.size(); i++) {
//            cout<<inliersOUT.indices[i]<<endl;
//        }
        //cout<<inliersOUT.indices.size()<<endl;
        double timeUsed = double(endTime - startTime) / CLOCKS_PER_SEC;
        cout << " RANSAC time " << timeUsed << " sec. Cluster Num: "<<clusters[ci].indices.size()<<" Inliners: " << intPlaneNum << " | ";

        double theta = acos(foundPlane.C / sqrt(foundPlane.A * foundPlane.A + foundPlane.B * foundPlane.B +
                                                foundPlane.C * foundPlane.C)) * 180 / 3.14;
        double phi = atan2(foundPlane.B, foundPlane.A) * 180 / 3.14;
//        cout << " Theta " << theta << " " << " phi " << phi << endl;
        foundPlane.theta = theta;
        foundPlane.phi = phi;
        planes.push_back(foundPlane);
        inliersOfEachPlane.push_back(inliersOUT);
        //Close Point
        double X, Y, Z;
//        if (foundPlane.D > 0) {
//            X = foundPlane.A * -foundPlane.D;
//            Y = foundPlane.B * -foundPlane.D;
//            Z = foundPlane.C * -foundPlane.D;
//        } else {
        cout << "A B C D " << foundPlane.A << " " << foundPlane.B << " " << foundPlane.C << " " << foundPlane.D<<"| ";
        X = foundPlane.A * -foundPlane.D;
        Y = foundPlane.B * -foundPlane.D;
        Z = foundPlane.C * -foundPlane.D;
//        }
        cout << "close point " << X << " " << Y << " " << Z << endl;
        vector<double> thisPoint;
        thisPoint.push_back(X);
        thisPoint.push_back(Y);
        thisPoint.push_back(Z);
        ClosePoints.push_back(thisPoint);
    }
    cout<<"total inlier point number "<<ClosePoints.size()<<endl;

    ///show---planes
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
        glColor3f(1, 0, 0);
        glVertex3d(0.1, 0, 0); //X axis Red
        glVertex3d(0, 0, 0);
        glColor3f(0, 1, 0);
        glVertex3d(0, 0.1, 0); //Y axis Green
        glVertex3d(0, 0, 0);
        glColor3f(0, 0, 1);
        glVertex3d(0, 0, 0.1); //Z axis Blue
        glVertex3d(0, 0, 0);
        glEnd();

        //points
        glBegin(GL_POINTS);
        for (int ci = 0; ci < ClosePoints.size(); ci++) {
            glColor3f(1, 1, 1);
            glPointSize(30);
            glVertex3d(ClosePoints[ci][0], ClosePoints[ci][1], ClosePoints[ci][2]);
        }
        glEnd();

        //points
        glBegin(GL_POINTS);
        for (int ci = 0; ci < planes.size(); ci++) {//
            //Well I check with meld that cluster do contains more point than inliers but why can;t show it?
            //cluster points --- including outliers
//            for (int pi = 0; pi < clusters[ci].indices.size(); pi++) {
//                float x = RGBDCloudDownSample->points[clusters[ci].indices[pi]].x;
//                float y = RGBDCloudDownSample->points[clusters[ci].indices[pi]].y;
//                float z = RGBDCloudDownSample->points[clusters[ci].indices[pi]].z;
//                //cout<<x<<" "<<y<<" "<<z<<endl;
//                glColor3f(1, 1, 1);
//                glVertex3d(x, y, z);
//            }
//            //cout<<"-----"<<endl;

            //chose color for plane points
            switch (ci) {
                case 0 :
                    glColor3f(1, 0, 0);
                    break;
                case 1 :
                    glColor3f(0, 1, 0);
                    break;
                case 2 :
                    glColor3f(0, 0, 1);
                    break;
                case 3 :
                    glColor3f(1, 1, 0);
                    break;
                case 4 :
                    glColor3f(1, 0, 1);
                    break;
                case 5 :
                    glColor3f(0, 1, 1);
                    break;
                case 6 :
                    glColor3f(0.5, 0, 0);
                    break;
                case 7 :
                    glColor3f(0, 0.5, 0);
                    break;
                case 8 :
                    glColor3f(0, 0, 0.5);
                    break;
                default :
                    glColor3f(1, 1, 1);
                    break;
            }

            //plane inlier points
            for (int pi = 0; pi < inliersOfEachPlane[ci].indices.size(); pi++) {
                float x = pointsOfEachPlane[ci]->points[inliersOfEachPlane[ci].indices[pi]].x;
                float y = pointsOfEachPlane[ci]->points[inliersOfEachPlane[ci].indices[pi]].y;
                float z = pointsOfEachPlane[ci]->points[inliersOfEachPlane[ci].indices[pi]].z;
                //cout <<x << " " << y << " " << z << endl;
                glVertex3d(x, y, z);
            }

        }
        glEnd();//点设置的结束

        pangolin::FinishFrame();
    }

    return 0;
}


///Read from png file
//    //读取rgb图像和depth图像，并转化为点云
//    //图像矩阵
//    cv::Mat rgb, depth;
//    //使用cv::imread()来读取图像
//    //rgb图像是8UC3的彩色图像
//    rgb = cv::imread("/home/xin/Downloads/DATASET/ICL-NUIM/living_room_traj0n_frei_png/rgb/901.png");
//    //depth是16UC1的单通道图像，注意flags设置为-1，表示读取原始数据不做修改
//    ///home/xin/Downloads/PlanarReconstruction/predict/_depth.png
//    ///home/xin/Downloads/DATASET/ICL-NUIM/living_room_traj0n_frei_png/depth_planarreconstruct/0.png
//    depth = cv::imread("/home/xin/Downloads/DATASET/ICL-NUIM/living_room_traj0n_frei_png/depth_planarreconstruct/901.png", -1);
//    //print(depth);
//    //点云变量
//    //使用智能指针，创建一个空点云。这种指针用完会自动释放
//    PointCloud::Ptr cloud(new PointCloud);
//    //for later process
//    int actualNum = 0;
//    pcl::PointCloud<pcl::PointXYZ>::Ptr RGBDCloud(new pcl::PointCloud<pcl::PointXYZ>);
//    RGBDCloud->resize(500000);
//
//    //遍历深度图
//    for (int m = 0; m < depth.rows; m++) {
//        for (int n = 0; n < depth.cols; n++) {
//            //获取深度图中(m,n)处的值
//            ushort d = depth.ptr<ushort>(m)[n];
//            //d可能没有值，若如此，跳过此点
//            if (d == 0)
//                continue;
//            //d存在值，则向点云增加一个点
//            PointT p;
//            //计算这个点的空间坐标
//            //p.z = double(d) / camera_factor;
//            float d_de = double(d) / camera_factor;
////            float u_u0_by_fx = (n-camera_cx)/camera_fx;
////            float v_v0_by_fy = (m-camera_cy)/camera_fy;
////            float z =  d_de / sqrt(u_u0_by_fx*u_u0_by_fx + v_v0_by_fy*v_v0_by_fy + 1 ) ;
////            p.z = z;
//            p.z = d_de;
//            p.x = (n - camera_cx) * p.z / camera_fx;
//            p.y = (m - camera_cy) * p.z / camera_fy;
//
//            //从rgb图像中获取它的颜色
//            //rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
//            p.b = rgb.ptr<uchar>(m)[n * 3];
//            p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
//            p.r = rgb.ptr<uchar>(m)[n * 3 + 2];
//            //把p加入到点云中
//            cloud->points.push_back(p);
//
//            RGBDCloud->points[actualNum].x = p.x;
//            RGBDCloud->points[actualNum].y = p.y;
//            RGBDCloud->points[actualNum].z = p.z;
////            cout<<p.x<<" "<<p.y<<" "<<p.z<<endl;
//            actualNum++;
//        }
//    }
//    RGBDCloud->resize(actualNum);
//    cout << "read " << RGBDCloud->points.size() << " points" << endl;
//    //设置并保存点云
//    cloud->height = 1;
//    cloud->width = cloud->points.size();
//    cout << "point cloud size=" << cloud->points.size() << endl;
//    cloud->is_dense = false;
//    pcl::io::savePCDFile("./pointcloud.pcd", *cloud);
////    //清除数据并保存
////    cloud->points.clear();
//    cout << "Point cloud saved." << endl;