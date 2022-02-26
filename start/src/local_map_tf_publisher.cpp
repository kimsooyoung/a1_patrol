#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

// 将全局的 map 坐标系转换为 局部 local_map 坐标系，x, y 相同，但是 local_map 的 z 轴坐标与机器狗的z 坐标相同

int main(int argc, char *argv[]){

    // ros 初始化
    ros::init(argc, argv, "local_map_tf_pubulisher");
    ros::NodeHandle n;
    // 设置 ros 循环的频率
    ros::Rate loop_rate(50);

    // 用于存储一个序列的 tf 变换
    tf2_ros::Buffer tfBuffer;
    // 监听坐标变换
    tf2_ros::TransformListener tfListener(tfBuffer);
    // 发布坐标变换
    tf2_ros::TransformBroadcaster tfBroadcaster;
    // 收到的坐标变换的内容
    geometry_msgs::TransformStamped transformStampedRec;
    // 要发送的坐标变换的内容
    geometry_msgs::TransformStamped transformStampedPub;

    // 当 ros 正常工作的时候进行循环
    while (ros::ok()){
        // 尝试获取坐标 map 到 base_link 的变换，如果失败，则返回错误信息，并暂停 1 秒
        try {
            transformStampedRec = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        // 根据收到的坐标变换，构建要发布的坐标变换
        transformStampedPub.header.stamp = transformStampedRec.header.stamp;
        transformStampedPub.header.frame_id = "map";
        transformStampedPub.child_frame_id = "local_map";
        transformStampedPub.transform.translation.z = transformStampedRec.transform.translation.z;
        transformStampedPub.transform.rotation.w = 1;
        // 发布 map 到 local_map 坐标变换
        tfBroadcaster.sendTransform(transformStampedPub);
        // ros::spinOnce();
        // 根据之前设置的频率进行适当的暂停
        loop_rate.sleep(); 
    }
    return 0;
}
