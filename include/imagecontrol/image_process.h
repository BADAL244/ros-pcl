#ifndef IMAGE_PROCESS_H
#define IMAGE_PROCESS_H

#include "ros/ros.h"
#include <std_msgs/String.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <unordered_set>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "processPointClouds.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include "imagecontrol/BoundingBox3d.h"
#include "imagecontrol/BoundingBoxes3d.h"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include "imagecontrol/GetNormals.h"
#include <pcl/features/normal_3d.h>

using namespace std;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

class Controller
{
    public:

        explicit Controller(ros::NodeHandle nh);

        void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg );
        void display_marker(imagecontrol::BoundingBoxes3d& boxes);  
        bool getNormalsReq(imagecontrol::GetNormals::Request &req, imagecontrol::GetNormals::Response &rsp);
        
    private:
        ros::NodeHandle m_nh;
        ros::Publisher m_state;
        ros::Publisher pub1;
        ros::Publisher pub2;
        ros::Publisher pubXZ;
        ros::Subscriber m_prev_state;
        ros::Publisher markers_pub_;
        int a , b ;
        const std::string point_cloud = "/zed2/zed_node/point_cloud/cloud_registered";
        float filterRes = 0.04;
        Eigen::Vector4f minPoint = Eigen::Vector4f(0 , -.4 , -1, 1);

        Eigen::Vector4f maxPoint = Eigen::Vector4f(2, .4 , 1 ,  1);
        int maxIter = 500;
        float clusterTolerance = 1;
        int minClusterSize = 300;
        int maxClusterSize = 1000;
        ros::ServiceServer get_normals_srv_;
        


};

#endif