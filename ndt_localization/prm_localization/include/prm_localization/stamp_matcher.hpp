//
// Created by vickylzy on 2020/7/5.
//

#ifndef SRC_STAMP_MATCHER_HPP
#define SRC_STAMP_MATCHER_HPP

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <Eigen/Dense>
#include <boost/circular_buffer.hpp>

namespace rt_localization_ns {


    bool
    closest_stamp_match(const boost::circular_buffer<nav_msgs::Odometry> &wheel_buffer_, const ros::Time &time_stamp,
                        nav_msgs::OdometryPtr &odometryPtr) {
        // TODO: stable way,use better find method when system done
        if (!wheel_buffer_.empty()) {
            double time_distance = INT8_MAX;
            for (const auto &i : wheel_buffer_) {
                if (fabs((time_stamp - i.header.stamp).toSec()) < time_distance) {
                    time_distance = (time_stamp - i.header.stamp).toSec();
                    odometryPtr = boost::make_shared<nav_msgs::Odometry>(i);
                }
            }
            return true;
        } else {
            std::cout << "empty buffer!" << std::endl;
            return false;
        }
    }

}

#endif //SRC_STAMP_MATCHER_HPP
