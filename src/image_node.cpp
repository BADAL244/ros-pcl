#include "imagecontrol/image_process.h"
#include "../include/imagecontrol/processPointClouds.h"

int main(int argc , char **argv)
{
    ros::init(argc , argv , "control");
    ros::NodeHandle nh;
    Controller controller(nh);
    
    
    ros::spin();
    return 0;
}