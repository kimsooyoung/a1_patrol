//
// Created by vickylzy on 20-5-19.
//

#include <ros/ros.h>
// #include "time.h"

// pcl
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pclomp/ndt_omp.h>
// cpp
#include <climits>
#include <cmath>
#include <ctime>
#include <mutex>
// untility
#include "prm_localization/fused_odometry.h"
#include "prm_localization/stamp_matcher.hpp"
#include "prm_localization/transform_utility.hpp"

namespace fuse_odom_ns {

    inline bool time_gap_judge(const ros::Time &stamp1, const ros::Time &stamp2,
                               double gap_threshold) {
        return abs(stamp1.toSec() - stamp2.toSec()) > gap_threshold;
    }

    FusedOdometry::FusedOdometry() {}

    FusedOdometry::~FusedOdometry() {}

    void FusedOdometry::onInit() {
        // param
        p_nh = ros::NodeHandle("~");

        double TransformationEpsilon;
        float ndt_resolution;
        p_nh.param<float>("downsample_resolution", downsample_res, 1.0f);
        p_nh.param<string>("map_tf", map_tf, "map");
        p_nh.param<string>("base_lidar_tf", lidar_tf, "lidar");
        p_nh.param<double>("TransformationEpsilon", TransformationEpsilon, 1e-5);
        p_nh.param<float>("ndt_resolution", ndt_resolution, 0.05f);
        p_nh.param<double>("time_gap_thresh", time_gap_thresh_, 0.5); // 1 second
        //        forward_threshold = 0.2; //m in 0.25s;
        //        rotation_threshold = 0.26; //reg 15degree in 0.25s
        //        p_nh.getParam("base_foot_tf",base_tf);
        // suber puber service
        //        lidar_suber =
        //        nh.subscribe("/localization/odom",1,&FusedOdometry::lidar_callback,this);
        lidar_odom_suber = nh.subscribe("/localization/odom", 1,
                                        &FusedOdometry::lidar_odom_callback, this);
        wheel_suber = nh.subscribe("/odom", 1, &FusedOdometry::wheel_callback, this);
        //        lidar_sub_ =
        //        std::make_unique<message_filters::Subscriber<sensor_msgs::PointCloud2>>(nh,"/lslidar_point_cloud",1);
        //        wheel_sub_ =
        //        std::make_unique<message_filters::Subscriber<nav_msgs::Odometry>>(nh,"odom",1);
        //        sync_ =
        //        std::make_unique<message_filters::Synchronizer<lidar_wheel_sync_policy>>(lidar_wheel_sync_policy(10),*lidar_sub_,*wheel_sub_);
        //        lidar_sub_ = new
        //        message_filters::Subscriber<sensor_msgs::PointCloud2>(nh,"/lslidar_point_cloud",5);
        //      sync_listener  _start
        //        wheel_sub_ =new
        //        message_filters::Subscriber<nav_msgs::Odometry>(nh,"odom",5); sync_
        //        = new
        //        message_filters::Synchronizer<lidar_wheel_sync_policy>(lidar_wheel_sync_policy(10),*lidar_sub_,*wheel_sub_);
        //        sync_->registerCallback(boost::bind(&FusedOdometry::sensor_callback,this,_1,_2));
        //      sync_listener _end
        loca_odom_puber =
                nh.advertise<nav_msgs::Odometry>("/localization/local_postioning", 5);
        // setup utility
        pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt(
                new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
        ndt->setTransformationEpsilon(TransformationEpsilon);
        ndt->setResolution(ndt_resolution);
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
        registration = ndt;
        //        downSampler.setLeafSize(downsample_res,downsample_res,downsample_res);

        //        status reset
        wheel_buffer_.set_capacity(200); // wheel 20hz  lidar 10 hz, sync to 10;
        //        robo_status=STEADY;
        ROS_INFO("waiting for init wheel odometry, lidar point to init ...");
        nav_msgs::OdometryConstPtr start_odom =
                ros::topic::waitForMessage<nav_msgs::Odometry>("/odom");

        former_lidar_odom_ = *start_odom; // lidar_odom prepare
        start_pose = odom2rotm(start_odom);
        former_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        sensor_msgs::PointCloud2ConstPtr init_pc_msg =
                ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
                        "/lslidar_point_cloud");
        pcl::fromROSMsg(*init_pc_msg, *former_cloud_);
        //        while (!wheel_buffer_.full()){
        //            wheel_buffer_.push_back(start_pose);
        //        }
        ROS_INFO("Init completed!");

        // test stamp
        //        former_lidar_stamp_ = ros::Time::now().toSec();
        //        former_wheel_stamp_ = ros::Time::now().toSec();
    }

    void FusedOdometry::sensor_callback(
            const sensor_msgs::PointCloud2ConstPtr &lidar_msg,
            const nav_msgs::OdometryConstPtr &wheel_msg) {
        clock_t start = clock();

        // test stamp
        //        std::cout << "lidar_time_gap: " << lidar_msg->header.stamp.toSec() -
        //        former_lidar_stamp_ << "\t"; std::cout << "wheel_time_gap: " <<
        //        wheel_msg->header.stamp.toSec() - former_wheel_stamp_ << std::endl;
        //        std::cout << "differ_time_gap: " << wheel_msg->header.stamp.toSec()
        //        - lidar_msg->header.stamp.toSec() << std::endl; former_lidar_stamp_
        //        = lidar_msg->header.stamp.toSec(); former_wheel_stamp_ =
        //        wheel_msg->header.stamp.toSec();
        // judge regis or pass
        if (time_gap_judge(lidar_msg->header.stamp, former_lidar_stamp_,
                           time_gap_thresh_)) {
            std::cout << "------------new frame-----------\n";
            // trim cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ori(
                    new pcl::PointCloud<pcl::PointXYZ>());
            pcl::PointCloud<pcl::PointXYZ>::Ptr pc_clear(
                    new pcl::PointCloud<pcl::PointXYZ>());
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(
                    new pcl::PointCloud<pcl::PointXYZ>());

            pcl::fromROSMsg(*lidar_msg, *pc_ori);
            std::vector<int> indices_index;
            pcl::removeNaNFromPointCloud(*pc_ori, *pc_clear, indices_index);
            //            downSampler.setInputCloud(pc_clear);
            //            downSampler.filter(*filtered_cloud);

            pcl::PointCloud<pcl::PointXYZ>::Ptr trimmed_cloud =
                    trimInputCloud(pc_clear, 0.01, 60, -0.3, 25);
            std::cout << "ori_pc points: " << pc_clear->size()
                      << "\t downsample points: " << pc_clear->size()
                      << "\t trimmed_cloud points: " << trimmed_cloud->size() << "\n";
            // regis former
            Eigen::Matrix4f prefict_motion =
                    odom2rotm(former_wheel_odom_).inverse() * odom2rotm(wheel_msg);
            registration->setInputTarget(former_cloud_);
            std::cout << std::endl;
            registration->setInputSource(trimmed_cloud);
            pcl::PointCloud<pcl::PointXYZ> result_cloud;
            // registration start
            registration->align(result_cloud, prefict_motion); //,curr_pose

            std::cout << "fitness score =" << registration->getFitnessScore() << "\n";

            // final update if regis
            if (registration->getFitnessScore() > 1)
                start_pose *= prefict_motion;
            else
                start_pose *= registration->getFinalTransformation();
            former_cloud_ = trimmed_cloud;
            former_lidar_stamp_ = lidar_msg->header.stamp;
            former_wheel_odom_ = *wheel_msg;

            // pub message tf
            transformBroadcaster.sendTransform(
                    matrix2transform(lidar_msg->header.stamp, start_pose, map_tf,
                                     lidar_msg->header.frame_id));
            nav_msgs::Odometry odom_curr =
                    rotm2odometry(start_pose, lidar_msg->header.stamp, map_tf,
                                  lidar_msg->header.frame_id);
            loca_odom_puber.publish(odom_curr);
        } else {
            // pub with wheel data
            return;
        }

        clock_t end = clock();
        std::cout << "registration regis time = "
                  << (double) (end - start) / CLOCKS_PER_SEC << " seconds"
                  << std::endl;
    }

//    void FusedOdometry::lidar_callback(const sensor_msgs::PointCloud2ConstPtr&
//    pc_msg){
//        if (lidar_counts<10){
//            ++lidar_counts;
//            return;
//        }else{
//
//        }
//    }
    void FusedOdometry::lidar_odom_callback(
            const nav_msgs::OdometryConstPtr &odom_msg) {
        nav_msgs::OdometryPtr former_wheel_msg;
        nav_msgs::OdometryPtr curr_wheel_msg;
        if (rt_localization_ns::closest_stamp_match(
                wheel_buffer_, former_lidar_odom_.header.stamp, former_wheel_msg) &&
            rt_localization_ns::closest_stamp_match(
                    wheel_buffer_, odom_msg->header.stamp, curr_wheel_msg)) {
            Eigen::Matrix4f wheel_motion =
                    odom2rotm(former_wheel_msg).inverse() * odom2rotm(curr_wheel_msg);
            Eigen::Matrix4f lidar_motion =
                    odom2rotm(former_lidar_odom_).inverse() * odom2rotm(odom_msg);
            print_err(wheel_motion, lidar_motion);
        } else {
            //
        }

        former_lidar_odom_ = *odom_msg;
    }

    void FusedOdometry::wheel_callback(
            const nav_msgs::OdometryConstPtr &wheel_msg) {

        wheel_buffer_.push_back(*wheel_msg);
        //        Eigen::Matrix4f relative_motion = wheel_buffer_.begin()->inverse() *
        //        *(--wheel_buffer_.end());
        //
        //        // TODO: get relative motion check curr status,comments below
        //        std::cout<<"relative_motion\n"<<relative_motion<<std::endl;
        //        nav_msgs::Odometry curr_odom;
        //        curr_odom.header=wheel_msg->header;
        //        curr_odom.pose.pose = rotm2geomo_pose(relative_motion);
        //        loca_odom_puber.publish(curr_odom);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr FusedOdometry::trimInputCloud(
            const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, const float disNear,
            const float disFar, const float low, const float high) {

        //            clock_t start = clock();

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTrimmed(
                new pcl::PointCloud<pcl::PointXYZ>());
        cloudTrimmed->reserve(cloud->size() / 2);
        std::copy_if(cloud->begin(), cloud->end(),
                     std::back_inserter(cloudTrimmed->points),
                     [&](const pcl::PointXYZ &p) {
                         double d = p.getVector3fMap().norm();
                         return d > disNear && d < disFar && p.z > low && p.z < high;
                     });
        cloudTrimmed->width = cloudTrimmed->points.size();
        cloudTrimmed->height = 1;
        cloudTrimmed->header = cloud->header;
        //            clock_t end = clock();
        //            time_sum+=(double)(end  - start) / CLOCKS_PER_SEC;
        //            NODELET_INFO("curr avg trim time = %f
        //            seconds",time_sum/(++trim_time));
        return cloudTrimmed;
    }

} // namespace fuse_odom_ns

int main(int argc, char **argv) {
    // Initiate ROS
    ros::init(argc, argv, "fused_odometry");
    fuse_odom_ns::FusedOdometry fusedOdometry;
    fusedOdometry.onInit();
    ros::spin();

    return 0;
}
// TODO: 1. use plane motion 2. failure cut fix 3 . local map