#include "color_extractor/color_extractor.h"

int main(int argc,char** argv)
{
	ros::init(argc,argv,"color_extractor");
	ColorExtractor color_extractor;
	color_extractor.process();
	return 0;
}