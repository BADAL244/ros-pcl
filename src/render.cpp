/* \author Aaron Brown */
// Functions and structs used to render the enviroment
// such as cars and the highway


#include "../include/imagecontrol/render.h"



void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer,
                      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                      std::string name,
                      Color color) {

  viewer->addPointCloud<pcl::PointXYZ>(cloud, name);
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, name);
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b,
      name);
}

void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer,
                      const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                      std::string name,
                      Color color) {

  if (color.r == -1) {
    // Select color based off of cloud intensity
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>
        intensity_distribution(cloud, "intensity");
    viewer->addPointCloud<pcl::PointXYZI>(cloud, intensity_distribution, name);
  } else {
    // Select color based off input value
    viewer->addPointCloud<pcl::PointXYZI>(cloud, name);
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b,
        name);
  }

  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
}


