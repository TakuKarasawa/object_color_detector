#include "object_color_detector/object_color_detector.h"

int main(int argc,char** argv)
{
	ros::init(argc,argv,"object_color_detector");
	ObjectColorDetector object_color_detector;
	object_color_detector.process();
	return 0;
}