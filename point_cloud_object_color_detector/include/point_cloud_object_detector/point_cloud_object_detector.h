#ifndef POINT_CLOUD_OBJECT_DETECTOR_H_
#define POINT_CLOUD_OBJECT_DETECTOR_H_

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

// PCL
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types_conversion.h>

// Eigen
#include <Eigen/Dense>

// Custom msg
#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "object_detector_msgs/ObjectPosition.h"
#include "object_detector_msgs/ObjectPositions.h"
#include "object_color_detector_msgs/ObjectColorPosition.h"
#include "object_color_detector_msgs/ObjectColorPositions.h"

// for loading yaml file
#include "target_objects/target_objects.h"
#include "color_params/color_params.h"

class PointCloudObjectDetector
{
public:
    PointCloudObjectDetector();
    void process();

private:
    void pc_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void bbox_callback(const darknet_ros_msgs::BoundingBoxesConstPtr& msg);

    void convert_from_vec_to_pc(std::vector<pcl::PointXYZRGB>& vec,
                                pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc);
    void calc_position(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                       double& x,double& y,double& z);
    void cluster_pc(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& clu_cloud);
    bool mask_color_params(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr& masked_cloud,
                           std::string& color);

    // for debug
    void print_ops(object_detector_msgs::ObjectPositions ops);
    void print_ocps(object_color_detector_msgs::ObjectColorPositions ocps);

    // node handler
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // subscriber
    ros::Subscriber pc_sub_;
    ros::Subscriber bbox_sub_;

    // publisher
    ros::Publisher ops_pub_;
    ros::Publisher ocps_pub_;
    ros::Publisher ops_pc_pub_;
    ros::Publisher ocps_pc_pub_;

    // point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;

    // tf
    boost::shared_ptr<tf2_ros::Buffer> buffer_;
    boost::shared_ptr<tf2_ros::TransformListener> listener_;
    boost::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

    // target objects (objects to determine color)
    TargetObjects* target_objects;

    // color parameters
    ColorParams* color_params_;

    // for clustering
    // static const int CLUSTER_NUM_ = 3;

    // dynamic parameters
    std::string pc_frame_id_;
    bool has_received_pc_;

    // parameters
    std::string CAMERA_FRAME_ID_;
    bool IS_PCL_TF_;
    bool IS_DEBUG_;
    int HZ_;
    int COLOR_TH_;
    double CLUSTER_TOLERANCE_;
    double MIN_CLUSTER_SIZE_;
};

#endif	// POINT_CLOUD_OBJECT_DETECTOR_H_
