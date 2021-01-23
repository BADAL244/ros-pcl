#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>


ros::Publisher pub;
ros::Publisher pub1;
void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    
    pcl::PCLPointCloud2* unfilterCloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr unfilterCloudPtr(unfilterCloud);
    pcl_conversions::toPCL(*input , *unfilterCloud);


    pcl::PCLPointCloud2::Ptr filteredCloud(new pcl::PCLPointCloud2);
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(unfilterCloudPtr);
    sor.setLeafSize(.05f , .05f , .05f);

    sor.filter(*filteredCloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr PCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*filteredCloud , *PCloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr msg(new pcl::PointCloud<pcl::PointXYZ>);

    msg->header.frame_id = "base_link";
    msg->height = PCloud->height;
    msg->width = PCloud->width;

    for (size_t i = 0; i < PCloud->size (); i++) {
        float x_cloud = PCloud->points[i].x;
        float y_cloud = PCloud->points[i].y;
        float z_cloud = PCloud->points[i].z;
  msg->points.push_back (pcl::PointXYZ (x_cloud, y_cloud, z_cloud));
  }

    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(*filteredCloud, output);

  // Publish the data
    pub1.publish (output);
    pub.publish (msg);


    pcl::ExtractIndices<pcl::PointXYZ> extract;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>),
			cloud2(new pcl::PointCloud<pcl::PointXYZ>);




}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/zed2/zed_node/point_cloud/cloud_registered", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub1 = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  pub  = nh.advertise<pcl::PointCloud<pcl::PointXYZ>> ("points2", 1);

  // Spin
  ros::spin ();
}