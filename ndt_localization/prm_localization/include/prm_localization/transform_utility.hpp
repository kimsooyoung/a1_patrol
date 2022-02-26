
#ifndef TRANSFORM_UTILITY
#define TRANSFORM_UTILITY

#include <Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>

/**function for conversation**/

using namespace Eigen;

inline double angleBetweenVectors(Eigen::Vector3d &a, Eigen::Vector3d &b) {
    return std::atan2(a.cross(b).norm(), a.dot(b));
}

inline double angleBetweenVectors(Eigen::Vector3f &a, Eigen::Vector3f &b) {
    return std::atan2(a.cross(b).norm(), a.dot(b));
}

Eigen::Vector3d calculate_err_R(const Eigen::Matrix4d &GroundTruth, const Eigen::Matrix4d &sample) {
    Eigen::Vector3d x(1, 0, 0), y(0, 1, 0), z(0, 0, 1);
    Eigen::Matrix3d t_a_m = GroundTruth.block(0, 0, 3, 3);
    Eigen::Matrix3d t_b_m = sample.block(0, 0, 3, 3);
    Eigen::Vector3d x_a = t_a_m * x;
    Eigen::Vector3d y_a = t_a_m * y;
    Eigen::Vector3d z_a = t_a_m * z;
    Eigen::Vector3d x_b = t_b_m * x;
    Eigen::Vector3d y_b = t_b_m * y;
    Eigen::Vector3d z_b = t_b_m * z;
    Eigen::Vector3d result(angleBetweenVectors(x_a, x_b) / M_PI * 180, angleBetweenVectors(y_a, y_b) / M_PI * 180,
                           angleBetweenVectors(z_a, z_b) / M_PI * 180);
    return result;
}

Eigen::Vector3d calculate_err_T(const Eigen::Matrix4d &GroundTruth, const Eigen::Matrix4d &sample) {
    Eigen::Vector3d gt_t = GroundTruth.block(0, 3, 3, 1);
    Eigen::Vector3d t = sample.block(0, 3, 3, 1);
    Eigen::Vector3d result = gt_t - t;
    return result;
}

Eigen::Vector3f calculate_err_R(const Eigen::Matrix4f &GroundTruth, const Eigen::Matrix4f &sample) {
    Eigen::Vector3f x(1, 0, 0), y(0, 1, 0), z(0, 0, 1);
    Eigen::Matrix3f t_a_m = GroundTruth.block(0, 0, 3, 3);
    Eigen::Matrix3f t_b_m = sample.block(0, 0, 3, 3);
    Eigen::Vector3f x_a = t_a_m * x;
    Eigen::Vector3f y_a = t_a_m * y;
    Eigen::Vector3f z_a = t_a_m * z;
    Eigen::Vector3f x_b = t_b_m * x;
    Eigen::Vector3f y_b = t_b_m * y;
    Eigen::Vector3f z_b = t_b_m * z;
    Eigen::Vector3f result(angleBetweenVectors(x_a, x_b) / M_PI * 180, angleBetweenVectors(y_a, y_b) / M_PI * 180,
                           angleBetweenVectors(z_a, z_b) / M_PI * 180);
    return result;
}

Eigen::Vector3f calculate_err_T(const Eigen::Matrix4f &GroundTruth, const Eigen::Matrix4f &sample) {
    Eigen::Vector3f gt_t = GroundTruth.block(0, 3, 3, 1);
    Eigen::Vector3f t = sample.block(0, 3, 3, 1);
    Eigen::Vector3f result = gt_t - t;
    return result;
}

void print_err(const Eigen::Matrix4f &GroundTruth, const Eigen::Matrix4f &sample) {
    Eigen::Vector3f rot_err = calculate_err_R(GroundTruth, sample);
    Eigen::Vector3f trans_err = calculate_err_T(GroundTruth, sample);
    //print independently
//    std::cout<<"r_err: dx: "<<rot_err[0]<<"\tdy: "<<rot_err[1]<<"\tdz: "<<rot_err[2]<<"\tt_err: x: "<<trans_err[0]<<"\ty: "<<trans_err[1]<<"\tz: "<<trans_err[2]<<std::endl;
    //print all
    std::cout << "r_err: " << sqrt(pow(rot_err[0], 2) + pow(rot_err[1], 2) + pow(rot_err[2], 2)) << "\tt_err: "
              << sqrt(pow(trans_err[0], 2) + pow(trans_err[1], 2) + pow(trans_err[2], 2)) << std::endl;
}

Matrix3f euler2rot(const float x_pi, const float y_pi, const float z_pi) {
    Matrix3f m;
    m = AngleAxisf(z_pi, Vector3f::UnitZ())
        * AngleAxisf(y_pi, Vector3f::UnitY())
        * AngleAxisf(x_pi, Vector3f::UnitX());
    return m;
}

Vector3f rot2euler(const Matrix3f &m) {
    return m.eulerAngles(2, 1, 0);
}

Quaternionf euler2quat(const float xr_pi, const float yp_pi, const float zy_pi) {
    Quaternionf q;
    q = AngleAxisf(xr_pi, Vector3f::UnitX())
        * AngleAxisf(yp_pi, Vector3f::UnitY())
        * AngleAxisf(zy_pi, Vector3f::UnitZ());
    return q;
}

Quaternionf rot2quat(const Eigen::Matrix4f &pose) {
    Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
    quat.normalize();
    return quat;
}

Vector3f quat2euler(const Quaternionf &quaternionf) {
    return quaternionf.toRotationMatrix().eulerAngles(0, 1, 2);
}

Matrix3f quat2rot(const Quaternionf &quaternionf) {
    return quaternionf.toRotationMatrix();
}

nav_msgs::Odometry rotm2odometry(const Eigen::Matrix4f &pose, const ros::Time &stamp, const std::string &frame_id,
                                 const std::string &child_frame_id) {
    Quaternionf q = rot2quat(pose);
    nav_msgs::Odometry odometry;
    odometry.header.frame_id = frame_id;
    odometry.child_frame_id = child_frame_id;
    odometry.header.stamp = stamp;
    odometry.pose.pose.position.x = pose(0, 3);
    odometry.pose.pose.position.y = pose(1, 3);
    odometry.pose.pose.position.z = pose(2, 3);
    odometry.pose.pose.orientation.x = q.x();
    odometry.pose.pose.orientation.y = q.y();
    odometry.pose.pose.orientation.z = q.z();
    odometry.pose.pose.orientation.w = q.w();
    return odometry;
}

Matrix4f odom2rotm(const nav_msgs::OdometryConstPtr &odom_msg) {
    Matrix4f m4f;
    m4f.setIdentity();
    Quaternionf qf;
    m4f(0, 3) = odom_msg->pose.pose.position.x;
    m4f(1, 3) = odom_msg->pose.pose.position.y;
    m4f(2, 3) = odom_msg->pose.pose.position.z;
    qf.x() = odom_msg->pose.pose.orientation.x;
    qf.y() = odom_msg->pose.pose.orientation.y;
    qf.z() = odom_msg->pose.pose.orientation.z;
    qf.w() = odom_msg->pose.pose.orientation.w;
    m4f.block(0, 0, 3, 3) = quat2rot(qf);
    return m4f;
}

Matrix4f odom2rotm(const nav_msgs::Odometry &odom_msg) {
    Matrix4f m4f;
    m4f.setIdentity();
    Quaternionf qf;
    m4f(0, 3) = odom_msg.pose.pose.position.x;
    m4f(1, 3) = odom_msg.pose.pose.position.y;
    m4f(2, 3) = odom_msg.pose.pose.position.z;
    qf.x() = odom_msg.pose.pose.orientation.x;
    qf.y() = odom_msg.pose.pose.orientation.y;
    qf.z() = odom_msg.pose.pose.orientation.z;
    qf.w() = odom_msg.pose.pose.orientation.w;
    m4f.block(0, 0, 3, 3) = quat2rot(qf);
    return m4f;
}

Matrix4f tftransform2rotm(const tf::StampedTransform &stampedTransform) {
    Matrix4f m4f;
    m4f.setIdentity();
    tf::Quaternion tfquat = stampedTransform.getRotation();
    tf::Vector3 tfvec3 = stampedTransform.getOrigin();
    Eigen::Quaternionf eigenQuat(tfquat.w(), tfquat.x(), tfquat.y(), tfquat.z());
    m4f.block(0, 0, 3, 3) = quat2rot(eigenQuat);
    m4f(0, 3) = tfvec3.x();
    m4f(1, 3) = tfvec3.y();
    m4f(2, 3) = tfvec3.z();
    return m4f;
}

Matrix3f geomo_oritention2rotm(const geometry_msgs::Quaternion &geo_quaternion) {
    Eigen::Quaternionf equaternion(geo_quaternion.w, geo_quaternion.x, geo_quaternion.y, geo_quaternion.z);
    return quat2rot(equaternion);
}

geometry_msgs::Pose rotm2geomo_pose(const Eigen::Matrix4f &rotm) {
    geometry_msgs::Pose result;
    result.position.x = rotm(0, 3);
    result.position.y = rotm(1, 3);
    result.position.z = rotm(2, 3);
    Eigen::Quaternionf eigen_quatf = rot2quat(rotm);
    result.orientation.x = eigen_quatf.x();
    result.orientation.y = eigen_quatf.y();
    result.orientation.z = eigen_quatf.z();
    result.orientation.w = eigen_quatf.w();
    return result;
}

/**
 * from hdl_localization
 * @brief convert a Eigen::Matrix to TransformedStamped
 * @param stamp           timestamp
 * @param pose            pose matrix
 * @param frame_id        frame_id
 * @param child_frame_id  child_frame_id
 * @return transform
 */
geometry_msgs::TransformStamped
matrix2transform(const ros::Time &stamp, const Eigen::Matrix4f &pose, const std::string &frame_id,
                 const std::string &child_frame_id) {
    Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
    quat.normalize();
    geometry_msgs::Quaternion odom_quat;
    odom_quat.w = quat.w();
    odom_quat.x = quat.x();
    odom_quat.y = quat.y();
    odom_quat.z = quat.z();

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = stamp;
    odom_trans.header.frame_id = frame_id;
    odom_trans.child_frame_id = child_frame_id;

    odom_trans.transform.translation.x = pose(0, 3);
    odom_trans.transform.translation.y = pose(1, 3);
    odom_trans.transform.translation.z = pose(2, 3);
    odom_trans.transform.rotation = odom_quat;

    return odom_trans;
}

#endif //TRANSFORM_UTILITY
