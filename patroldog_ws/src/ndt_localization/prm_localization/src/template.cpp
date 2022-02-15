//
// Created by vickylzy on 19-12-20.
//

#include <ros/ros.h>
#include <ros/timer.h>
// #include "time.h"
#include "std_msgs/String.h"
// ros_time

// ros_msg
#include <nav_msgs/Odometry.h>
#include <prm_localization/DriveInfo.h>
// ros srv
#include <prm_localization/locali.h>
// pcl
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
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
#include <ctime>
#include <limits.h>
#include <math.h>
#include <mutex>
// untility
#include <prm_localization/transform_utility.hpp>

using namespace std;

class Localizer_global {

public:
    Localizer_global() {}

    virtual ~Localizer_global() {}

    void onInit() {
        // param
        p_nh = ros::NodeHandle("~");
        downsample_res = p_nh.param("downsample_resolution", 0.05f);

        string global_pcd_path;
        p_nh.getParam("downsample_resolution", global_pcd_path);
        p_nh.getParam("map_tf", map_tf);
        p_nh.getParam("base_lidar_tf", lidar_tf);
        p_nh.getParam("base_foot_tf", base_tf);
        // suber puber service
        reloca_service =
                nh.advertiseService("/localization/relocalization",
                                    &Localizer_global::relocalization_callback, this);
    }

private:
    bool relocalization_callback(prm_localization::localiRequest &req,
                                 prm_localization::localiResponse &res) {
        sensor_msgs::PointCloud2ConstPtr point_msg =
                ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
                        "/velodyne_points");
        prm_localization::DriveInfoConstPtr drive_msg =
                ros::topic::waitForMessage<prm_localization::DriveInfo>("/drive");
    }

private:
    // ros
    ros::NodeHandle nh;
    ros::NodeHandle p_nh;
    // suber and puber
    //    ros::Publisher streeing_motion_puber;
    ros::Subscriber gnss_suber;
    // service
    ros::ServiceServer reloca_service;

    // parameter
    float downsample_res;
    string map_tf;
    string lidar_tf;
    string base_tf;

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