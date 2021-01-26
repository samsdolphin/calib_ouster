#include <iostream>
#include <fstream>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std;
using namespace cv;
using namespace Eigen;
using namespace message_filters;

typedef pcl::PointXYZI PointType;
Mat K, D;
string pc_write_path, img_write_path, filename;
bool pc_init = false;
bool img_init = false;

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    if (pc_init && !img_init)
    {
        cout<<"img captured!"<<endl;
        Mat InImage = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8)->image;
        filename = img_write_path + to_string(8) + ".png";
        imwrite(filename, InImage);
        img_init = true;
    }
}

void pc_callback(const sensor_msgs::PointCloud2ConstPtr& pc_msg)
{
    pcl::PointCloud<PointType>::Ptr pc(new pcl::PointCloud<PointType>);
    pcl::fromROSMsg(*pc_msg, *pc);
    if (!pc_init)
    {
        cout<<"pc captured!"<<endl;
        ofstream file_w;
        file_w.open(pc_write_path, std::ofstream::trunc);
        for (size_t i = 0; i < pc->points.size(); i++)
            if (pc->points[i].x > 0)
                file_w << pc->points[i].x << "\t"
                       << pc->points[i].y << "\t" << pc->points[i].z << "\n";
        file_w.close();
        pc_init = true;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ouster_extractor");
    ros::NodeHandle nh("~");

    nh.getParam("pc_write_path", pc_write_path);
    nh.getParam("img_write_path", img_write_path);

    ros::Subscriber sub_img =
        nh.subscribe<sensor_msgs::Image>("/camera/image_raw", 30, img_callback);
    ros::Subscriber sub_pc = 
        nh.subscribe<sensor_msgs::PointCloud2>("/os_cloud_node/points", 100, pc_callback);

    string cam_cal;
    nh.getParam("cam_cal_file", cam_cal);
    cv::FileStorage param_reader(cam_cal, cv::FileStorage::READ);

    param_reader["camera_matrix"] >> K;
    param_reader["distortion_coefficients"] >> D;

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}