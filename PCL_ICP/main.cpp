/*
 * This project designed to test PCL ICP algorithm
 * To check if this can be used for RGBD point matching
 */
#include <iostream>
#include <pcl/io/pcd_io.h>
#include "pcl/point_types.h"
#include "pcl/registration/icp.h"
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
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

using namespace std;
const double camera_factor = 5000;
const double camera_cx = 319.5;
const double camera_cy = 239.5;
const double camera_fx = 481.2;
const double camera_fy = -480.0;

int main(){
    int maxLoadNum = 500000;
    ///Load Frame 1
    //read from png file
    cv::Mat rgb, depth;
    rgb = cv::imread("/home/xin/Downloads/DATASET/ICL-NUIM/living_room_traj0n_frei_png/rgb/900.png");
    depth = cv::imread("/home/xin/Downloads/DATASET/ICL-NUIM/living_room_traj0n_frei_png/depth_planarreconstruct/900.png", -1);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    int actualNum = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr RGBDCloud(new pcl::PointCloud<pcl::PointXYZ>);
    RGBDCloud->resize(maxLoadNum);
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
            cloud->points.push_back(p);
            RGBDCloud->points[actualNum].x = p.x;
            RGBDCloud->points[actualNum].y = p.y;
            RGBDCloud->points[actualNum].z = p.z;
            actualNum++;
        }
    }
    RGBDCloud->resize(actualNum);
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cout << "point cloud size=" << cloud->points.size() << endl;
    ///Load Frame 2
    //read from png file
    cv::Mat rgb2, depth2;
    rgb2 = cv::imread("/home/xin/Downloads/DATASET/ICL-NUIM/living_room_traj0n_frei_png/rgb/901.png");
    depth2 = cv::imread("/home/xin/Downloads/DATASET/ICL-NUIM/living_room_traj0n_frei_png/depth_planarreconstruct/901.png", -1);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGBA>);
    int actualNum2 = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr RGBDCloud2(new pcl::PointCloud<pcl::PointXYZ>);
    RGBDCloud2->resize(maxLoadNum);
    for(int m = 0; m < depth2.rows;m++){
        for(int n = 0; n < depth2.cols;n++){
            ushort d = depth2.ptr<ushort>(m)[n];
            if(d <= 0)
                continue;
            pcl::PointXYZRGBA p;
            float d_de = double(d) /camera_factor;
            p.z = d_de;
            p.x = (n - camera_cx) * p.z / camera_fx;
            p.y = (m - camera_cy) * p.z / camera_fy;
            p.b = rgb2.ptr<uchar>(m)[n * 3];
            p.g = rgb2.ptr<uchar>(m)[n * 3 + 1];
            p.r = rgb2.ptr<uchar>(m)[n * 3 + 2];
            cloud2->points.push_back(p);
            RGBDCloud2->points[actualNum2].x = p.x;
            RGBDCloud2->points[actualNum2].y = p.y;
            RGBDCloud2->points[actualNum2].z = p.z;
            //cout<<p.x<<" "<<p.y<<" "<<p.z<<endl;
            actualNum2++;
        }
    }
    RGBDCloud2->resize(actualNum2);
    cloud2->height = 1;
    cloud2->width = cloud2->points.size();
    cout << "point cloud size=" << cloud2->points.size() << endl;
    ///DownSample
    pcl::PointCloud<pcl::PointXYZ>::Ptr RGBDCloudDownSample(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor1;
    sor1.setInputCloud(RGBDCloud);
    //sor.setLeafSize(0.03f, 0.03f, 0.03f);
    sor1.setLeafSize(0.01f, 0.01f, 0.01f);
    sor1.filter(*RGBDCloudDownSample);
    cout <<RGBDCloud->points.size()<< " downsampling remains " << RGBDCloudDownSample->points.size() << endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr RGBDCloudDownSample2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor2;
    sor2.setInputCloud(RGBDCloud);
    //sor.setLeafSize(0.03f, 0.03f, 0.03f);
    sor2.setLeafSize(0.01f, 0.01f, 0.01f);
    sor2.filter(*RGBDCloudDownSample2);
    cout <<RGBDCloud2->points.size()<< " downsampling remains " << RGBDCloudDownSample2->points.size() << endl;

    ///ICP
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(RGBDCloudDownSample);
    icp.setInputTarget(RGBDCloudDownSample2);
//    icp.setInputSource(RGBDCloud);
//    icp.setInputTarget(RGBDCloud2);

    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);

    std::cout<<"has converged ? : "<<icp.hasConverged()<<" score: "<<icp.getFitnessScore()<<std::endl;
    std::cout<<icp.getFinalTransformation()<<std::endl;

    return 0;
}

//int main() {
//    std::cout << "Hello, World!" << std::endl;
//
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>(5,1));
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
//
//    for (auto &point : *cloud_in) {
//        point.x = 1024 * rand() / (RAND_MAX + 1.0f);
//        point.y = 1024 * rand() / (RAND_MAX + 1.0f);
//        point.z = 1024 * rand() / (RAND_MAX + 1.0f);
//    }
//    std::cout<<"Saved "<<cloud_in->size()<<" data points to inputs: "<<std::endl;
//    for (auto& point : *cloud_in)
//        std::cout << point << std::endl;
//
//    *cloud_out = *cloud_in;
//    std::cout << "size:" << cloud_out->size() << std::endl;
//    for (auto& point : *cloud_out)
//        point.x += 0.7f;
//
//    std::cout << "Transformed " << cloud_in->size () << " data points:" << std::endl;
//
//    for(auto &point : *cloud_out)
//        std::cout<<point<<std::endl;
//
//    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//    icp.setInputSource(cloud_in);
//    icp.setInputTarget(cloud_out);
//
//    pcl::PointCloud<pcl::PointXYZ> Final;
//    icp.align(Final);
//
//    std::cout<<"has converged ? : "<<icp.hasConverged()<<" score: "<<icp.getFitnessScore()<<std::endl;
//    std::cout<<icp.getFinalTransformation()<<std::endl;
//
//    return 0;
//}
