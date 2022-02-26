//
// Created by vickylzy on 2020/5/19.
//

#ifndef SRC_FUSED_ODOMETRY_H
#define SRC_FUSED_ODOMETRY_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pclomp/ndt_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
// ros_time
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <boost/circular_buffer.hpp>


#define STEADY  0;
#define WALKING 1;
#define WALK_END 2;
#define TURNING 3;
#define TURN_END 4
namespace fuse_odom_ns {

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> lidar_wheel_sync_policy;


    using namespace std;

    class FusedOdometry {

    public:
        FusedOdometry();

        virtual  ~FusedOdometry();

        void onInit();

        void
        sensor_callback(const sensor_msgs::PointCloud2ConstPtr &lidar_msg, const nav_msgs::OdometryConstPtr &wheel_msg);


    private:
        void lidar_callback(const sensor_msgs::PointCloud2ConstPtr &pc_msg);

        void lidar_odom_callback(const nav_msgs::OdometryConstPtr &odom_msg);

        void wheel_callback(const nav_msgs::OdometryConstPtr &wheel_msg);

        pcl::PointCloud<pcl::PointXYZ>::Ptr trimInputCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
                                                           float disNear, float disFar, float low, float high);


    private:
        //ros
        ros::NodeHandle nh;
        ros::NodeHandle p_nh;
        //suber and puber
//    ros::Publisher streeing_motion_puber;
        ros::Subscriber lidar_suber;
        ros::Subscriber lidar_odom_suber;
        ros::Subscriber wheel_suber;
        ros::Publisher loca_odom_puber;
        tf::TransformBroadcaster transformBroadcaster;
//        std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> lidar_sub_;
//        std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>> wheel_sub_;
        message_filters::Subscriber<sensor_msgs::PointCloud2> *lidar_sub_;
        message_filters::Subscriber<nav_msgs::Odometry> *wheel_sub_;

//        std::unique_ptr<message_filters::Synchronizer<lidar_wheel_sync_policy>> sync_;
        message_filters::Synchronizer<lidar_wheel_sync_policy> *sync_;

        //service
        ros::ServiceServer reloca_service;

        //parameter
        float downsample_res;
        double time_gap_thresh_;
        string map_tf;
        string lidar_tf;
        string base_tf;
//        float rotation_threshold;
//        float forward_threshold;
        float time_gap;
        pcl::PointCloud<pcl::PointXYZ>::Ptr former_cloud_;
        nav_msgs::Odometry former_wheel_odom_;
        ros::Time former_lidar_stamp_;
        nav_msgs::Odometry former_lidar_odom_;
        //stamp test
//        double former_lidar_stamp_;
//        double former_wheel_stamp_;
        // runtime status
        Eigen::Matrix4f start_pose;
//        Eigen::Matrix4f steady_pose;
        boost::circular_buffer<nav_msgs::Odometry> wheel_buffer_;
        //utilitys
        pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Ptr registration;
        pcl::VoxelGrid<pcl::PointXYZ> downSampler;

        //flag
        uint8_t lidar_counts;
        uint8_t robo_status;
        // move for more than 0.5 meters
    };
}

#endif //SRC_FUSED_ODOMETRY_H
