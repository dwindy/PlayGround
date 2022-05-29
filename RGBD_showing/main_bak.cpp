///Read png file and restore 3D structure
#include <iostream>
#include<opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <pangolin/pangolin.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;

// 定义点云类型
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud <PointT> PointCloud;

// 相机内参
const double camera_factor = 5000;
const double camera_cx = 319.5;
const double camera_cy = 239.5;
const double camera_fx = 481.2;
const double camera_fy = -480.0;

// 主函数
int main(int argc, char **argv) {
    //读取rgb图像和depth图像，并转化为点云
    //图像矩阵
    cv::Mat rgb, depth;
    //使用cv::imread()来读取图像
    //rgb图像是8UC3的彩色图像
    rgb = cv::imread("/home/xin/Downloads/DATASET/ICL-NUIM/traj1_frei_png/rgb/0.png");
    //depth是16UC1的单通道图像，注意flags设置为-1，表示读取原始数据不做修改
    depth = cv::imread("/home/xin/Downloads/DATASET/ICL-NUIM/traj1_frei_png/depth/0.png", -1);

    //点云变量
    //使用智能指针，创建一个空点云。这种指针用完会自动释放
    PointCloud::Ptr cloud(new PointCloud);

    //遍历深度图
    for (int m = 0; m < depth.rows; m++) {
        for (int n = 0; n < depth.cols; n++) {
            //获取深度图中(m,n)处的值
            ushort d = depth.ptr<ushort>(m)[n];
            //d可能没有值，若如此，跳过此点
            if (d == 0)
                continue;
            //d存在值，则向点云增加一个点
            PointT p;
            //计算这个点的空间坐标
            //p.z = double(d) / camera_factor;
            float d_de = double(d) / camera_factor;
//            float u_u0_by_fx = (n-camera_cx)/camera_fx;
//            float v_v0_by_fy = (m-camera_cy)/camera_fy;
//            float z =  d_de / sqrt(u_u0_by_fx*u_u0_by_fx + v_v0_by_fy*v_v0_by_fy + 1 ) ;
//            p.z = z;
            p.z = d_de;
            p.x = (n - camera_cx) * p.z / camera_fx;
            p.y = (m - camera_cy) * p.z / camera_fy;

            //从rgb图像中获取它的颜色
            //rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
            p.b = rgb.ptr<uchar>(m)[n * 3];
            p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
            p.r = rgb.ptr<uchar>(m)[n * 3 + 2];
            //把p加入到点云中
            cloud->points.push_back(p);
        }
    }
    //设置并保存点云
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cout << "point cloud size=" << cloud->points.size() << endl;
    cloud->is_dense = false;
    pcl::io::savePCDFile("./pointcloud.pcd", *cloud);
//    //清除数据并保存
//    cloud->points.clear();
    cout << "Point cloud saved." << endl;

    ///show
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
        glColor3f(1, 0, 1);
        glVertex3d(0.1, 0, 0);
        glVertex3d(0, 0, 0);
        glColor3f(0, 1, 0);
        glVertex3d(0, 0.1, 0);
        glVertex3d(0, 0, 0);
        glColor3f(0, 0, 1);
        glVertex3d(0, 0, 0.1);
        glVertex3d(0, 0, 0);
        glEnd();
        //points
        glBegin(GL_POINTS);
//        for (int ci = 0; ci < cloud->points.size(); ci++) {
//            glColor3f(1, 1, 1);
//            glVertex3d(cloud->points[ci].x, cloud->points[ci].y, cloud->points[ci].z);
//            //cout<<cloud->points[ci].x<<" "<<cloud->points[ci].y<<" "<<cloud->points[ci].z<<endl;
//        }
        for (int m = 0; m < depth.rows; m++) {
            for (int n = 0; n < depth.cols; n++) {
                //rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
                float b = rgb.ptr<uchar>(m)[n * 3]/255.0;
                float g = rgb.ptr<uchar>(m)[n * 3 + 1]/255.0;
                float r = rgb.ptr<uchar>(m)[n * 3 + 2]/255.0;
                glColor3f(r, g, b);
                int index = m*depth.cols + n;
                glVertex3d(cloud->points[index].x, cloud->points[index].y, cloud->points[index].z);
            }
        }
        glEnd();//点设置的结束
        pangolin::FinishFrame();
    }

    return 0;
}

//int main() {
//    std::cout << "Hello, World!" << std::endl;
//    string depthFilename = "/home/xin/Downloads/DATASET/ICL-NUIM/traj1_frei_png/depth/0.png";
//    string RGBFilename = "/home/xin/Downloads/DATASET/ICL-NUIM/traj1_frei_png/rgb/0.png";
//    cv::Mat im = cv::imread(depthFilename);
//    cv::Mat imD = cv::imread(RGBFilename);
//    float K[3][3] = {481.20, 0, 319.50,
//                     0, -480.00, 239.50,
//                     0, 0, 1.00};
//    float cx = 319.5;
//    float cy = 239.5;
//    float fx = 481.2;
//    float fy = -480.0;
//    vector<cv::Point3d> P3Ds;
//    for (int v = 0; v < im.rows; v++) {
//        for (int u = 0; u < im.cols; u++) {
//            cout<<" r,c "<<v<<" "<<u<<" ";
//            float u_u0_by_fx = (u - cx) / fx;
//            float v_v0_by_fy = (v - cy) / fy;
//            //cout<<" by focal "<<u_u0_by_fx<<" "<<v_v0_by_fy<<" ";
//            float z_defactor = imD.at<ushort>(v, u) / 5000;
//            float z = z_defactor / sqrt(u_u0_by_fx * u_u0_by_fx + v_v0_by_fy * v_v0_by_fy + 1);
//            cout<<" origin Z "<<imD.at<ushort>(v, u)<<" defact Z "<<z_defactor<<" compute Z "<<z<<endl;
//            //z = z_defactor;
//            cv::Point3d P3d;
//            P3d.x = (u_u0_by_fx) * (z);
//            P3d.y = (v_v0_by_fy) * (z);
//            P3d.z = z;
//            cout<<P3d.x<<" "<<P3d.y<<" "<<P3d.z<<endl;
//            P3Ds.push_back(P3d);
//        }
//    }
//
//    ///show
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
//        //points
//        glBegin(GL_POINTS);
//        for (int ci = 0; ci < P3Ds.size(); ci++) {
//            glColor3f(1, 1, 1);
//            glVertex3d(P3Ds[ci].x, P3Ds[ci].y, P3Ds[ci].z);
//        }
//        glEnd();//点设置的结束
//        pangolin::FinishFrame();
//    }
//}
