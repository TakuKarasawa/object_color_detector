#ifndef OBJECT_COLOR_DETECTOR_H_
#define OBJECT_COLOR_DETECTOR_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types_conversion.h>
#include <pcl_ros/transforms.h>

#include <Eigen/Dense>

#include "color_param/color_param.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/BoundingBoxes.h"

class ObjectColorDetector
{
public:
    ObjectColorDetector();
    void process();

private:
    void pc_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
	void bbox_callback(const darknet_ros_msgs::BoundingBoxesConstPtr& msg);

    void load_color_param();
    void clustering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
    void mask_color_param(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);

	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;

	ros::Subscriber pc_sub_;
	ros::Subscriber bbox_sub_;
	ros::Publisher pc_pub_;
    ros::Publisher target_pc_pub_;
    ros::Publisher mask_pc_pub_;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;

	boost::shared_ptr<tf2_ros::Buffer> buffer_;
    boost::shared_ptr<tf2_ros::TransformListener> listener_;
    boost::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

    XmlRpc::XmlRpcValue color_param_list_;
    std::vector<ColorParam> color_params_;

	std::string target_object_name_;
	std::string base_link_frame_id_;
	std::string pc_frame_id_;

	bool has_received_pc_;
    bool is_pcl_tf_;

    int HZ_;
    int COLOR_TH_;
    static const int CLUSTER_NUM_ = 3;
    double CLUSTER_TOLERANCE_;
    double MIN_CLUSTER_SIZE_;
};

#endif  // OBJECT_COLOR_DETECTOR_H_