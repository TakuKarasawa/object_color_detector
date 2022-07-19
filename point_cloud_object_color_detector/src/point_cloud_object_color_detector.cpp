#include "point_cloud_object_color_detector/point_cloud_object_color_detector.h"

PointCloudObjectColorDetector::PointCloudObjectColorDetector() :
    private_nh_("~"),
    cloud_(new pcl::PointCloud<pcl::PointXYZRGB>()),
    has_received_pc_(false)
{
    // load parameter
    private_nh_.param("CAMERA_FRAME_ID",CAMERA_FRAME_ID_,{std::string("base_link")});
    private_nh_.param("TARGET_OBJECT_NAME",TARGET_OBJECT_NAME_,{std::string("roomba")});
    private_nh_.param("IS_PCL_TF",IS_PCL_TF_,{false});
    private_nh_.param("HZ",HZ_,{10});
    private_nh_.param("COLOR_TH",COLOR_TH_,{1000});
    private_nh_.param("CLUSTER_TOLERANCE",CLUSTER_TOLERANCE_,{0.02});
    private_nh_.param("MIN_CLUSTER_SIZE",MIN_CLUSTER_SIZE_,{100});

    // load color_param
    color_loadr_ptr_ = std::make_shared<ColorLoader>(nh_,private_nh_);
    color_loadr_ptr_->output_color_params(color_params_);

    pc_sub_ = nh_.subscribe("/camera/depth_registered/points",1,&PointCloudObjectColorDetector::pc_callback,this);
    bbox_sub_ = nh_.subscribe("/bounding_boxes",1,&PointCloudObjectColorDetector::bbox_callback,this);

    pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/target_cloud",1);
    target_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/clustered_target_cloud",1);
    mask_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/mask_cloud",1);

    buffer_.reset(new tf2_ros::Buffer);
    listener_.reset(new tf2_ros::TransformListener(*buffer_));
    broadcaster_.reset(new tf2_ros::TransformBroadcaster);
}

void PointCloudObjectColorDetector::pc_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    cloud_->clear();
    pcl::fromROSMsg(*msg,*cloud_);
    pc_frame_id_ = msg->header.frame_id;
    if(IS_PCL_TF_){
        geometry_msgs::TransformStamped transform_stamped;
        try{
            transform_stamped = buffer_->lookupTransform(CAMERA_FRAME_ID_,msg->header.frame_id,ros::Time(0));
        }
        catch(tf2::TransformException& ex){
            ROS_WARN("%s", ex.what());
            return;
        }
        Eigen::Matrix4f transform = tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
        pcl::transformPointCloud(*cloud_,*cloud_,transform);
    }
    has_received_pc_ = true;
}

void PointCloudObjectColorDetector::bbox_callback(const darknet_ros_msgs::BoundingBoxesConstPtr& msg)
{
    if(has_received_pc_){
        for(const auto &bbox : msg->bounding_boxes){
            std::vector<pcl::PointXYZRGB> points;
            std::vector<std::vector<pcl::PointXYZRGB>> rearranged_points(cloud_->height,std::vector<pcl::PointXYZRGB>());
            std::vector<pcl::PointXYZRGB> values;

            for(const auto &p : cloud_->points) points.push_back(p);

            if(points.size() == cloud_->width*cloud_->height){
                for(size_t i = 0; i < cloud_->height; i++){
                    for(size_t j = 0; j < cloud_->width; j++){
                        rearranged_points.at(i).push_back(points.at(i*cloud_->width + j));
                    }
                }
            }

            if(!(bbox.xmin == 0 && bbox.xmax == 0)){
                if(bbox.Class == TARGET_OBJECT_NAME_){
                    for(int x = bbox.xmin; x < bbox.xmax; x++){
                        for(int y = bbox.ymin; y < bbox.ymax; y++){
                            values.push_back(rearranged_points.at(y).at(x));
                        }
                    }
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rearranged_cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
                    rearranged_cloud->width = bbox.xmax - bbox.xmin;
                    rearranged_cloud->height = bbox.ymax - bbox.ymin;
                    rearranged_cloud->points.resize(rearranged_cloud->width*rearranged_cloud->height);

                    int c = 0;
                    for(const auto &value : values){
                        if(!std::isnan(value.x) && !std::isnan(value.y) && !std::isnan(value.z)){
                            rearranged_cloud->points.at(c).x = value.x;
                            rearranged_cloud->points.at(c).y = value.y;
                            rearranged_cloud->points.at(c).z = value.z;
                            rearranged_cloud->points.at(c).r = value.r;
                            rearranged_cloud->points.at(c).g = value.g;
                            rearranged_cloud->points.at(c).b = value.b;
                            c ++;
                        }
                    }

                    clustering(rearranged_cloud);

                    sensor_msgs::PointCloud2 pc_msg;
                    pcl::toROSMsg(*rearranged_cloud,pc_msg);
                    pc_msg.header.frame_id = pc_frame_id_;
                    pc_pub_.publish(pc_msg);
                }
            }
        }
    }
}

void PointCloudObjectColorDetector::clustering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> indices;
    pcl::shared_ptr<pcl::EuclideanClusterExtraction<pcl::PointXYZRGB>> ec(new pcl::EuclideanClusterExtraction<pcl::PointXYZRGB>);
    ec->setClusterTolerance(CLUSTER_TOLERANCE_);
    ec->setMinClusterSize(MIN_CLUSTER_SIZE_);
    ec->setMaxClusterSize(cloud->points.size());
    ec->setSearchMethod(tree);
    ec->setInputCloud(cloud);
    ec->extract(indices);

    pcl::shared_ptr<pcl::ExtractIndices<pcl::PointXYZRGB>> ex(new pcl::ExtractIndices<pcl::PointXYZRGB>);
    ex->setInputCloud(cloud);
    ex->setNegative(false);

    pcl::PointIndices::Ptr tmp_clustered_indices (new pcl::PointIndices);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    if(indices.size() <= 1) return;
    *tmp_clustered_indices = indices[0];
    ex->setIndices(tmp_clustered_indices);
    ex->filter(*tmp_cloud);

    mask_color_param(tmp_cloud);

    sensor_msgs::PointCloud2 pc_msg;
    pcl::toROSMsg(*tmp_cloud,pc_msg);
    pc_msg.header.frame_id = pc_frame_id_;
    target_pc_pub_.publish(pc_msg);
}

void PointCloudObjectColorDetector::mask_color_param(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> mask_clouds;
    mask_clouds.resize(color_params_.size());
    for(size_t i = 0; i < mask_clouds.size(); i++){
        mask_clouds[i].reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    }

    std::vector<int> counts;
    counts.resize(color_params_.size());
    for(size_t i = 0; i < counts.size(); i++){
        counts[i] = 0;
    }

    for(size_t i = 0; i < color_params_.size(); i++){
        for(const auto& rgb_pc : cloud->points){
            pcl::PointXYZHSV hsv_pc;
            pcl::PointXYZRGBtoXYZHSV(rgb_pc,hsv_pc);

            if((color_params_[i].lower.h <= color_params_[i].upper.h &&
               color_params_[i].lower.h <= hsv_pc.h/2 &&
               hsv_pc.h/2 <= color_params_[i].upper.h ||
               color_params_[i].lower.h > color_params_[i].upper.h &&
               (color_params_[i].lower.h <= hsv_pc.h/2 || hsv_pc.h/2 <= color_params_[i].upper.h)) &&
               color_params_[i].lower.s <= hsv_pc.s*255. && hsv_pc.s*255. <= color_params_[i].upper.s &&
               color_params_[i].lower.v <= hsv_pc.v*255. && hsv_pc.v*255. <= color_params_[i].upper.v &&
               isfinite(hsv_pc.x) && isfinite(hsv_pc.y) && isfinite(hsv_pc.z)){
                mask_clouds[i]->push_back(rgb_pc);
                counts[i] += 1;
            }
        }
    }

    std::vector<int>::iterator it = std::max_element(counts.begin(),counts.end());
    size_t max_cost = std::distance(counts.begin(),it);
    if(counts[max_cost] < COLOR_TH_){
        std::cout << "Unknown" << std::endl;
    }
    else{
        std::cout << "COLOR: " << color_params_[max_cost].name << std::endl;
    }
    sensor_msgs::PointCloud2 pc_msg;
    pcl::toROSMsg(*mask_clouds[max_cost],pc_msg);
    pc_msg.header.frame_id = pc_frame_id_;
    mask_pc_pub_.publish(pc_msg);
}

void PointCloudObjectColorDetector::process()
{
    ros::Rate rate(HZ_);
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }
}
