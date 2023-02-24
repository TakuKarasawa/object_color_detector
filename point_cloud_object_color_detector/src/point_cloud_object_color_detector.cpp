#include "point_cloud_object_color_detector/point_cloud_object_color_detector.h"

using namespace object_color_detector;

PointCloudObjectColorDetector::PointCloudObjectColorDetector() :
    private_nh_("~"),
    cloud_(new pcl::PointCloud<pcl::PointXYZRGB>()),
    target_objects(new TargetObjects(private_nh_)), 
    color_params_(new ColorParams(private_nh_)),
    pc_frame_id_(std::string("")), has_received_pc_(false)
{
    private_nh_.param("CAMERA_FRAME_ID",CAMERA_FRAME_ID_,{std::string("base_link")});
    private_nh_.param("HZ",HZ_,{10});
    private_nh_.param("COLOR_TH",COLOR_TH_,{800});
    private_nh_.param("CLUSTER_TOLERANCE",CLUSTER_TOLERANCE_,{0.02});
    private_nh_.param("MIN_CLUSTER_SIZE",MIN_CLUSTER_SIZE_,{100});

    pc_sub_ = nh_.subscribe("pc_in",1,&PointCloudObjectColorDetector::pc_callback,this);
    bbox_sub_ = nh_.subscribe("bbox_in",1,&PointCloudObjectColorDetector::bbox_callback,this);

    private_nh_.param("IS_DEBUG",IS_DEBUG_,{false});
    if(IS_DEBUG_) target_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pc_out",1);
    obj_color_pub_ =  nh_.advertise<object_color_detector_msgs::ObjectColorPositions>("ocd_out",1);

    private_nh_.param("IS_PCL_TF",IS_PCL_TF_,{false});
    if(IS_PCL_TF_){
        buffer_.reset(new tf2_ros::Buffer);
        listener_.reset(new tf2_ros::TransformListener(*buffer_));
        broadcaster_.reset(new tf2_ros::TransformBroadcaster);
    }
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
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
        object_color_detector_msgs::ObjectColorPositions ocps;
        ocps.header.frame_id = CAMERA_FRAME_ID_;
        for(const auto &bbox : msg->bounding_boxes){
            std::vector<pcl::PointXYZRGB> points;
            for(const auto &p : cloud_->points) points.emplace_back(p);

            std::vector<std::vector<pcl::PointXYZRGB>> rearranged_points(cloud_->height,std::vector<pcl::PointXYZRGB>());
            if(points.size() == cloud_->width*cloud_->height){
                for(size_t i = 0; i < cloud_->height; i++){
                    for(size_t j = 0; j < cloud_->width; j++){
                        rearranged_points.at(i).emplace_back(points.at(i*cloud_->width + j));
                    }
                }
            }else{
                ROS_WARN("points size is not cloud size");
                return;
            }

            if(!(bbox.xmin == 0 && bbox.xmax == 0)){
                std::vector<pcl::PointXYZRGB> values;
                for(int x = bbox.xmin; x < bbox.xmax; x++){
                    for(int y = bbox.ymin; y < bbox.ymax; y++){
                        values.emplace_back(rearranged_points.at(y).at(x));
                    }
                }

                pcl::PointCloud<pcl::PointXYZRGB>::Ptr rearranged_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
                rearranged_cloud->width = bbox.xmax - bbox.xmin;
                rearranged_cloud->height = bbox.ymax - bbox.ymin;
                rearranged_cloud->points.resize(rearranged_cloud->width*rearranged_cloud->height);
                convert_from_vec_to_pc(values,rearranged_cloud);

                pcl::PointCloud<pcl::PointXYZRGB>::Ptr clu_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
                cluster_pc(rearranged_cloud,clu_cloud);

                if(target_objects->is_included(bbox.Class)){
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr masked_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
                    std::string color;
                    if(mask_color_params(clu_cloud,masked_cloud,color)){
                        *merged_cloud += *masked_cloud;
                        object_color_detector_msgs::ObjectColorPosition ocp;
                        ocp.name = bbox.Class;
                        ocp.color = color;
                        ocp.probability = bbox.probability;
                        calc_position(masked_cloud,ocp.x,ocp.y,ocp.z);
                        ocps.object_color_position.emplace_back(ocp);
                    }
                }
            }
            else{
                ROS_WARN("No bbox range");
                return;
            }

        }
        ros::Time now_time = ros::Time::now();
        if(IS_DEBUG_){
            sensor_msgs::PointCloud2 cloud_msg;
            pcl::toROSMsg(*merged_cloud,cloud_msg);
            cloud_msg.header.frame_id = pc_frame_id_;
            cloud_msg.header.stamp = now_time;
            target_pc_pub_.publish(cloud_msg);
        }

        if(ocps.object_color_position.size() != 0){
            ocps.header.stamp = now_time;
            print_ocps(ocps);
            obj_color_pub_.publish(ocps);
        }
        has_received_pc_ = false;
    }
}

void PointCloudObjectColorDetector::cluster_pc(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& clusterd_cloud)
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

    clusterd_cloud = tmp_cloud;
}

void PointCloudObjectColorDetector::convert_from_vec_to_pc(std::vector<pcl::PointXYZRGB>& vec,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc)
{
    int count = 0;
    for(const auto &v : vec){
        if(!std::isnan(v.x) && !std::isnan(v.y) && !std::isnan(v.z)){
            pc->points.at(count).x = v.x;
            pc->points.at(count).y = v.y;
            pc->points.at(count).z = v.z;
            pc->points.at(count).r = v.r;
            pc->points.at(count).g = v.g;
            pc->points.at(count).b = v.b;
            count++;
        }
    }
}

void PointCloudObjectColorDetector::calc_position(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,double& x,double& y,double& z)
{
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;

    int count = 0;
    for(const auto &p : cloud->points){
        sum_x += p.x;
        sum_y += p.y;
        sum_z += p.z;
        count++;
    }
    x = sum_x/(double)count;
    y = sum_y/(double)count;
    z = sum_z/(double)count;
}

bool PointCloudObjectColorDetector::mask_color_params(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& masked_cloud,std::string& color)
{
    class MaskCloud
    {
    public:
        MaskCloud() :
            cloud(new pcl::PointCloud<pcl::PointXYZRGB>),
            score(0) {}

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
        int score;
    private:
    };

    class Scores : public std::map<ColorParam*,MaskCloud*>
    {
    public:
        Scores(ColorParams* cp)
        {
            for(auto it = cp->begin(); it != cp->end(); it++){
                ColorParam* color_param (new ColorParam(it->name,it->lower,it->upper));
                MaskCloud* mask_cloud (new MaskCloud());
                this->insert(std::map<ColorParam*,MaskCloud*>::value_type(color_param,mask_cloud));
            }
        }

        void calc_score(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
        {
            for(auto it = this->begin(); it != this->end(); it++){
                for(auto &rgb_p : cloud->points){
                    pcl::PointXYZHSV hsv_p;
                    pcl::PointXYZRGBtoXYZHSV(rgb_p,hsv_p);

                    if((it->first->lower.h <= it->first->upper.h &&
                        it->first->lower.h <= hsv_p.h/2 &&
                        hsv_p.h/2 <= it->first->upper.h || it->first->lower.h > it->first->upper.h &&
                       (it->first->lower.h <= hsv_p.h/2 || hsv_p.h/2 <= it->first->upper.h)) &&
                        it->first->lower.s <= hsv_p.s*255. && hsv_p.s*255. <= it->first->upper.s &&
                        it->first->lower.v <= hsv_p.v*255. && hsv_p.v*255. <= it->first->upper.v &&
                        std::isfinite(hsv_p.x) && std::isfinite(hsv_p.y) && std::isfinite(hsv_p.z)){
                            it->second->cloud->emplace_back(rgb_p);
                            it->second->score++;
                    }
                }
            }
        }

        void print_result()
        {
            std::cout << "=== RESULT ===" <<  std::endl;
            for(auto it = this->begin(); it != this->end(); it++){
                std::cout << it->first->name << ": "<< it->second->score << std::endl;
            }
            std::cout << std::endl;
        }

    private:
    };

    Scores scores(color_params_);
    scores.calc_score(cloud);
    scores.print_result();
    auto max_score = std::max_element(scores.begin(),scores.end(),[](const auto& x,const auto&  y){ return x.second->score < y.second->score; });
    masked_cloud = max_score->second->cloud;

    if(max_score->second->score >= COLOR_TH_){
        color = max_score->first->name;
        return true;
    }
    else{
        color = std::string(""); // Unknown
        return false;
    }
}

void PointCloudObjectColorDetector::print_ocps(object_color_detector_msgs::ObjectColorPositions ocps)
{
    for(const auto &ocp : ocps.object_color_position){
        double dist = std::sqrt(ocp.x*ocp.x + ocp.z*ocp.z);
        double angle = std::atan2(ocp.z,ocp.x) - 0.5*M_PI;
        std::cout << ocp.color << ": ("
                  << dist << "," <<angle << ")" << std::endl;
    }
}

void PointCloudObjectColorDetector::process()
{
    ros::Rate rate(HZ_);
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }
}
