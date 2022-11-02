#include "mask_image_creator/mask_image_creator.h"

using namespace object_color_detector;

int main(int argc,char** argv)
{
    ros::init(argc,argv,"mask_image_creator");
    
    if(argc != 3){
        std::cerr << "Usage: rosrun mask_image_creator_node 'mode' 'target_color'" << std::endl;
        std::cout << "Start: rosrun mask_image_creator_node viwer GREEN" << std::endl; 
        MaskImageCreator mask_image_creator(Mode::VIWER,std::string("GREEN"));
        mask_image_creator.process();
    }
    else{
        if(argv[1] == std::string("viwer")){
            MaskImageCreator mask_image_creator(Mode::VIWER,argv[2]);
            mask_image_creator.process();
        }
        else if(argv[1] == std::string("extractor")){
            MaskImageCreator mask_image_creator(Mode::EXTRACTOR,argv[2]);
            mask_image_creator.process();
        }
        else{
            std::cerr << "No applicable mode. Please select 'viwer' or 'extractor'" << std::endl;
            std::cout << "Start: rosrun mask_image_creator_node viwer " << argv[2] << std::endl; 
            MaskImageCreator mask_image_creator(Mode::VIWER,argv[2]);
            mask_image_creator.process();
        }
    }

    return 0;
}
