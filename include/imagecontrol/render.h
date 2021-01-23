/* \author Aaron Brown */
// Functions and structs used to render the environment
// such as cars and the highway

#ifndef RENDER_H
#define RENDER_H
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <string>
#include <vector>

struct Color {

  float r, g, b;

  Color(float setR, float setG, float setB) : r(setR), g(setG), b(setB) {}
};


enum CameraAngle { XY, TopDown, Side, FPS };


void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer,
                      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                      std::string name,
                      Color color = Color(1, 1, 1));
void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer,
                      const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                      std::string name,
                      Color color = Color(-1, -1, -1));

#endif
