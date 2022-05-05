#include "mask_image_creator/mask_image_creator.h"

int main(int argc,char** argv)
{
	ros::init(argc,argv,"mask_image_creator");
	MaskImageCreator mask_image_creator;
	mask_image_creator.process();
	return 0;
}