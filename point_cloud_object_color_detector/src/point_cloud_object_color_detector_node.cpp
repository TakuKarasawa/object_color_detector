#include "point_cloud_object_color_detector/point_cloud_object_color_detector.h"

using namespace object_color_detector;

int main(int argc,char** argv)
{
    ros::init(argc,argv,"point_cloud_object_color_detector");
    PointCloudObjectColorDetector point_cloud_object_color_detector;
    point_cloud_object_color_detector.process();
    return 0;
}
