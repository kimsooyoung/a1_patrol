//
// Created by vickylzy on 19-12-20.
//

#include <ros/ros.h>
#include <ros/timer.h>
// #include "time.h"
#include <std_msgs/String.h>
// ros_time

// ros_msg
#include <nav_msgs/Odometry.h>
#include <prm_localization/DriveInfo.h>
// ros srv
#include <prm_localization/locali.h>
#include <std_srvs/Trigger.h>
// pcl
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pclomp/ndt_omp.h>

// eigen
#include <Eigen/Dense>
// tf
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
// cpp
#include <boost/circular_buffer.hpp>
#include <boost/make_unique.hpp>
#include <ctime>
#include <limits.h>
#include <math.h>
#include <mutex>
// untility
#include "eigMatch/eigMatch.hpp"
#include "eigMatch/extractEig.hpp"
#include "prm_localization/csv_transform_reader.hpp"
#include "prm_localization/transform_utility.hpp"

using namespace std;
using wut_slam::extractEig;
using wut_slam::eigMatch;

class Localizer_global {

public:
    Localizer_global() {}

    virtual ~Localizer_global() {}

    void onInit() {
        // param
        p_nh = ros::NodeHandle("~");
        string global_pcd_path;
        string csv_path;
        p_nh.param("downsample_resolution", downsample_res, 0.1f);
        p_nh.param("TransformationEpsilon", TransformationEpsilon, 0.0001f);
        p_nh.param("ndt_resolution", ndt_resolution, 0.5f);
        p_nh.param("search_radius", search_radius, 45.0f);
        p_nh.param("eig_gridstep", eig_gridstep_, 1.0f);
        p_nh.param<string>("map_tf", map_tf, "parkinglot_world");
        p_nh.param<string>("global_map_pcd_path", global_pcd_path,
                           "/home/vickylzy/workspaceROS/MAP_BAG/wuhan/gongkong_front_andbackyard/"
                           "merged/gongkong_front_andbackyard/wuhan_gongkong_front_merged.pcd");
        p_nh.param<string>("base_lidar_tf", lidar_tf, "laser_link");
        p_nh.param<string>("base_foot_tf", base_tf, "base_foot_tf");
        p_nh.param<string>("Transform_csv_path", csv_path,
                           "/home/vickylzy/workspaceROS/MAP_BAG/wuhan/gongkong_front_andbackyard/"
                           "merged/gongkong_front_andbackyard/"
                           "wuhan_gongkong_front_M_map_GNSS.csv");
        // read transform
        prm_localization::CSV_reader csvReader(csv_path);
        Motion_mg = csvReader.getTransformMg();
        //        std::cout<<"init_Motion_mg\n"<<Motion_mg<<std::endl;

        // suber puber service
        reloca_service = nh.advertiseService("/localization/relocalization", &Localizer_global::relocalization_callback,
                                             this);
        //        gnss_suber =
        //        nh.subscribe("/drive",2,&Localizer_global::location_retrans_callback,this);
        reloca_pc_puber = nh.advertise<sensor_msgs::PointCloud2>("reloca_pointcloud", 2, true);

        // map
        full_map.reset(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::io::loadPCDFile(global_pcd_path, *full_map);
        full_map->header.frame_id = map_tf;
        downSampler.setInputCloud(full_map);
        downSampler.setLeafSize(downsample_res, downsample_res, downsample_res);
        // boost::shared_ptr<pcl::VoxelGrid<pcl::PointXYZ>> voxelgrid(new
        // pcl::VoxelGrid<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(
                new pcl::PointCloud<pcl::PointXYZ>());
        downSampler.filter(*filtered);
        full_map = filtered;
        kdtree.setInputCloud(full_map);
        // registrition ready
        pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt(
                new pclomp::NormalDistributionsTransform<pcl::PointXYZ,
                        pcl::PointXYZ>());
        ndt->setTransformationEpsilon(TransformationEpsilon);
        ndt->setResolution(ndt_resolution);
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
        registration = ndt;
        // fine alignment
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-5);
        icp.setRANSACOutlierRejectionThreshold(0.04f);
        icp.setMaxCorrespondenceDistance(100 * 0.04f);
        map_pub = nh.advertise<sensor_msgs::PointCloud2>("map_cloud", 1);

        eig_matcher_.reset(new eigMatch);
        map_des_ptr_.reset(new extractEig);
        src_des_ptr_.reset(new extractEig);

        eig_matcher_->init_param(0.9, eig_gridstep_); // MAP has full curr cloud

        // full_map->header.frame_id=map_tf;
        // map_pub_click =
        // nh.advertiseService("/localiztion/map_pub_click",&Localizer_global::click_pub_map_callback,this);
        // odom_pub = nh.advertise<nav_msgs::Odometry>("trans_odom",3);
        ROS_INFO("Relocalizer node initial succeed! ");
    }

private:
    bool relocalization_callback(prm_localization::localiRequest &req,
                                 prm_localization::localiResponse &res) {
        ROS_INFO("starting global localization, waiting for gps and lidar...");
        prm_localization::DriveInfoConstPtr drive_msg = ros::topic::waitForMessage<prm_localization::DriveInfo>(
                "/drive");
        sensor_msgs::PointCloud2ConstPtr point_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
                "/lslidar_point_cloud");
        ROS_INFO("gps and lidar acquired !");
        Vector4f gps_pos;
        if (drive_msg->gnss_flag != 1)
            ROS_WARN("Bad GPS_Singal, relocation maybe wrong!");
        gps_pos << drive_msg->gnss_x, drive_msg->gnss_y, 0, 1;
        //        ROS_INFO("gps input:");
        //        std::cout<<gps_pos<<std::endl;
        //        std::cout<<"Motion_mg\n"<<Motion_mg<<std::endl;
        Vector4f map_curr_pos = Motion_mg * gps_pos;
        ROS_INFO("gps position: ");
        std::cout << map_curr_pos << std::endl;

        // trim localmap
        pcl::PointXYZ searchPoint(map_curr_pos(0), map_curr_pos(1), 0);
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        pcl::PointCloud<pcl::PointXYZ>::Ptr trimmed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (kdtree.radiusSearch(searchPoint, search_radius, pointIdxRadiusSearch,
                                pointRadiusSquaredDistance) > 0) {
            //                NODELET_INFO("trimmed_cloud init
            //                points_num:%ld",trimmed_cloud->width);
            //                NODELET_INFO("search
            //                points_num:%ld",pointIdxRadiusSearch.size());
            trimmed_cloud->points.reserve(size_t(full_map->width / 3));
            for (int i : pointIdxRadiusSearch) {
                trimmed_cloud->points.push_back(full_map->points[i]);
            }
            trimmed_cloud->width = trimmed_cloud->points.size();
            trimmed_cloud->height = 1;
            cout << "full_map.size()\t:" << full_map->size()
                << "\ntrimmed_cloud->width:\t" << trimmed_cloud->width << endl;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr curr_cloud_wnan(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr curr_cloud_dense(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr curr_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*point_msg, *curr_cloud_wnan);
        // remove nan
        std::vector<int> indi_index;
        pcl::removeNaNFromPointCloud(*curr_cloud_wnan, *curr_cloud_dense, indi_index);
        downSampler.setInputCloud(curr_cloud_dense);
        downSampler.filter(*curr_cloud);

        // perform eig match
        map_des_ptr_->extract_2(trimmed_cloud, eig_gridstep_);
        src_des_ptr_->extract_2(curr_cloud, eig_gridstep_);
        eig_matcher_->set_target(map_des_ptr_->srcDesp, map_des_ptr_->srcSeed_m, map_des_ptr_->srcNorm);
        eig_matcher_->set_source(src_des_ptr_->srcDesp, src_des_ptr_->srcSeed_m, src_des_ptr_->srcNorm);
        Eigen::Matrix4d result = eig_matcher_->match();
        //        eig_matcher_->set_source(map_des_ptr_->srcDesp, map_des_ptr_->srcSeed_m, map_des_ptr_->srcNorm);
//        eig_matcher_->set_target(src_des_ptr_->srcDesp, src_des_ptr_->srcSeed_m, src_des_ptr_->srcNorm);
//        Eigen::Matrix4d result = eig_matcher_->match().inverse();
        map_des_ptr_->clear();
        src_des_ptr_->clear();
        if (isnan(result(0, 0))) {
            ROS_ERROR("relocalization failed, getting nan in eig result, try changing gridstep ");
            return false;
        }
        icp.setInputTarget(trimmed_cloud);
        icp.setInputSource(curr_cloud);
//        pcl::io::savePCDFileASCII ("/home/vickylzy/Desktop/map_cloud.pcd", *trimmed_cloud);
//        pcl::io::savePCDFileASCII ("/home/vickylzy/Desktop/curr_cloud.pcd", *curr_cloud);
//        std::cout<<"result: \n"<<result<<std::endl;
        std::cout << "registrion in progress" << std::endl;
        pcl::PointCloud<pcl::PointXYZ> result_cloud;
        icp.align(result_cloud, result.cast<float>());
        double refined_objfuns = icp.getFitnessScore();

        ROS_INFO(" motion with refined_objfuns: %f",
                 refined_objfuns); // min objfun
        //        cout<<"refined_objfuns: "<<refined_objfuns<<endl;
        //        result_motions[int(index)];
        //(optional) show regis pc
        //        {
        //            pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud(new
        //            pcl::PointCloud<pcl::PointXYZ>());
        //            pcl::transformPointCloud(*curr_cloud, *result_cloud,
        //            refined_movement[(int) index]);
        //            pcl_conversions::toPCL(point_msg->header,
        //            result_cloud->header); result_cloud->header.frame_id = map_tf;
        //            reloca_pc_puber.publish(result_cloud);
        //            map_pub.publish(full_map);
        //            ROS_INFO("cloud republished");
        //        }
        Eigen::Matrix4f final_localization = icp.getFinalTransformation();
        res.location = rotm2geomo_pose(final_localization);
        res.location = rotm2geomo_pose(result.cast<float>());
        return true;
    }

    //    void location_retrans_callback(const
    //    prm_localization::DriveInfoConstPtr& drive_msg){
    ////        if(drive_msg->gnss_flag)
    //        Vector4f curr_pos;
    //        curr_pos << drive_msg->gnss_x,drive_msg->gnss_y,0,1;
    //        Vector4f map_curr_pos = Motion_mg * curr_pos;
    //        nav_msgs::Odometry odom;
    //        odom.header.frame_id=map_tf;
    //        odom.pose.pose.position.x=map_curr_pos(0);
    //        odom.pose.pose.position.y=map_curr_pos(1);
    //        odom.pose.pose.position.z=0;
    //        odom_pub.publish(odom);
    //    }

private:
    // ros
    ros::NodeHandle nh;
    ros::NodeHandle p_nh;
    // suber and puber
    //    ros::Publisher streeing_motion_puber;
    ros::Publisher reloca_pc_puber;
    ros::Publisher map_pub; // optional

    // service
    ros::ServiceServer reloca_service;

    // parameter
    float downsample_res;
    float TransformationEpsilon;
    float ndt_resolution;
    float search_radius;
    string map_tf;
    string lidar_tf;
    string base_tf;
    Matrix4f Motion_mg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr full_map;
    // utility
    pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Ptr registration;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    pcl::VoxelGrid<pcl::PointXYZ> downSampler;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    boost::unique_ptr<extractEig> map_des_ptr_;
    boost::unique_ptr<extractEig> src_des_ptr_;
    boost::unique_ptr<eigMatch> eig_matcher_;
    float eig_gridstep_;
    // flag
};

int main(int argc, char **argv) {
    // Initiate ROS
    ros::init(argc, argv, "relocalizaer");
    Localizer_global localizer_global;
    localizer_global.onInit();
    ros::spin();

    return 0;
}
/**init**/
// check transform
// map_tf = "map_f";
// full_map.reset(new pcl::PointCloud<pcl::PointXYZ>());
// pcl::io::loadPCDFile("/home/vickylzy/workspaceROS/MAP_BAG/wuhan/Transform_map_GNSS_3dbag_ls700b_1204_1/wuhan_ls_great.pcd",
// *full_map); float downsample_resolution=0.1;
// boost::shared_ptr<pcl::VoxelGrid<pcl::PointXYZ>> voxelgrid(new
// pcl::VoxelGrid<pcl::PointXYZ>());
// voxelgrid->setLeafSize(downsample_resolution, downsample_resolution,
// downsample_resolution); voxelgrid->setInputCloud(full_map);
// pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new
// pcl::PointCloud<pcl::PointXYZ>()); voxelgrid->filter(*filtered); full_map =
// filtered; map_pub = nh.advertise<sensor_msgs::PointCloud2>("map_cloud",3);
// full_map->header.frame_id=map_tf;
// map_pub_click =
// nh.advertiseService("/localiztion/map_pub_click",&Localizer_global::click_pub_map_callback,this);
// odom_pub = nh.advertise<nav_msgs::Odometry>("trans_odom",3);

/**map pub srv**/
// bool click_pub_map_callback(std_srvs::TriggerRequest & req,
// std_srvs::TriggerResponse &res){
//    map_pub.publish(full_map);
//    res.message="map_pubbed";
//    res.success=1;
//    return true;
//}

/**transform check param**/
// ros::ServiceServer map_pub_click;
// ros::Publisher map_pub;
// ros::Publisher odom_pub;
