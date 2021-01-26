#include <fstream>
#include <iostream>
#include <string>

// #include "calibrate.hpp"
#include <ceres/ceres.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

typedef pcl::PointXYZRGB PointType;
using namespace std;
using namespace Eigen;
using namespace cv;

pcl::PointCloud<PointType> read_pointcloud(std::string path)
{
    pcl::PointCloud<PointType> pc;
    pc.points.resize(1e8);
    std::fstream file;
    file.open(path);
    size_t cnt = 0;
    float x, y, z;
    while (!file.eof())
    {
        file >> x >> y >> z;
        pc.points[cnt].x = x;
        pc.points[cnt].y = y;
        pc.points[cnt].z = z;
        cnt++;
    }
    file.close();
    pc.points.resize(cnt);
    return pc;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "calib_ouster");
    ros::NodeHandle nh("~");

    ros::Publisher pub_out =
        nh.advertise<sensor_msgs::PointCloud2>("/color_pt", 10000);

    string pc_path, camera_param, image_path;
    nh.getParam("pointcloud_path", pc_path);
    nh.getParam("camera_param", camera_param);
    nh.getParam("image_path", image_path);

    cv::FileStorage param_reader(camera_param, cv::FileStorage::READ);
    cv::Mat camera_matrix, dist_coeff;
    param_reader["camera_matrix"] >> camera_matrix;
    param_reader["distortion_coefficients"] >> dist_coeff;
    cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);

    vector<cv::Point3f> world_pts;
    vector<cv::Point2f> image_pts;

    Matrix3d R, Rx, Ry, Rz;
    double pi = 3.1415926;
    double a = pi / 180 * (0);
    double b = pi / 180 * (-89);
    double g = pi / 180 * (90);
    Rx << 1, 0, 0, 0, cos(a), -sin(a), 0, sin(a), cos(a);
    Ry << cos(b), 0, sin(b), 0, 1, 0, -sin(b), 0, cos(b);
    Rz << cos(g), -sin(g), 0, sin(g), cos(g), 0, 0, 0, 1;
    R = Rz * Ry;
    g = pi / 180 * (-0.6);
    Rz << cos(g), -sin(g), 0, sin(g), cos(g), 0, 0, 0, 1;
    R = R * Rz;
    a = pi / 180 * (-1);
    Rx << 1, 0, 0, 0, cos(a), -sin(a), 0, sin(a), cos(a);
    R = R * Rx;
    cout<<"R"<<endl;
    cout<<R<<endl;

    Vector3d t(-0.07886, -0.0387, -0.01511);
    cv::Vec3d rvec, tvec;
    cv::Mat R_mat = cv::Mat_<double>(3, 3);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            R_mat.at<double>(i, j) = R(i, j);

    cv::Rodrigues(R_mat, rvec);
    for (int i = 0; i < 3; i++)
        tvec(i) = t(i);

    pcl::PointCloud<PointType>::Ptr pc_src(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr pc_out(new pcl::PointCloud<PointType>);
    *pc_src = read_pointcloud(pc_path);
    size_t pc_size = pc_src->points.size();

    for (size_t i = 0; i < pc_size; i++)
    {
        Point3f p(pc_src->points[i].x, pc_src->points[i].y, pc_src->points[i].z);
        world_pts.push_back(p);
        projectPoints(Mat(world_pts), Mat(rvec), Mat(tvec), camera_matrix,
                      dist_coeff, image_pts);
        world_pts.clear();
        int c = image_pts[0].x;
        int r = image_pts[0].y;

        if (r >= image.size().height || c >= image.size().width || r < 0 || c < 0)
            continue;

        Vec3b pixel = image.at<Vec3b>(r, c);
        PointType point;
        point.x = float(pc_src->points[i].x);
        point.y = float(pc_src->points[i].y);
        point.z = float(pc_src->points[i].z);
        point.r = uint8_t(pixel[2]);
        point.g = uint8_t(pixel[1]);
        point.b = uint8_t(pixel[0]);
        pc_out->push_back(point);
    }

    cout<<"complete"<<endl;

    sensor_msgs::PointCloud2 laserCloudMsg;
    pcl::toROSMsg(*pc_out, laserCloudMsg);
    laserCloudMsg.header.stamp = ros::Time::now();
    laserCloudMsg.header.frame_id = "camera_init";
    pub_out.publish(laserCloudMsg);

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        pub_out.publish(laserCloudMsg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}