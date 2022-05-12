#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>


class LaserScanToPointCloud{

private:
    ros::NodeHandle n_;

    laser_geometry::LaserProjection projector_;

    ros::Publisher pcl_pub_;
    ros::Subscriber laserscan_sub_;
    
public:
    LaserScanToPointCloud(ros::NodeHandle n) : n_(n){

        pcl_pub_ = n_.advertise<sensor_msgs::PointCloud2>("/my_cloud", 1);
        laserscan_sub_ = n_.subscribe("/scan", 1, &LaserScanToPointCloud::subCallback, this);

        ROS_INFO("my_scan_to_cloud node started");
    }

    void subCallback(const sensor_msgs::LaserScan::ConstPtr &scan_in){
        sensor_msgs::PointCloud2 cloud;

        std::cout<< "sdfd" << std::endl;

        try{
            projector_.projectLaser(*scan_in,cloud);
            cloud.header = scan_in->header;
            cloud.header.frame_id = "pointcloud_lidar";
        }
        catch (tf::TransformException& e){
            std::cout << e.what();
            return;
        }
    
        pcl_pub_.publish(cloud);

        // for(unsigned int i=0; i<cloud.fields.size(); i++)
        //     {cloud.fields[i].count = 1;}
    
        // if (::frame_count==0){
        //     wall=cloud;	
        //     scan_pub_.publish(wall); //save 1st pointcloud as wall sensor_msg
        //     ROS_INFO("published wall");
        // }
        // else{
        //     scan_pub_.publish(cloud);
        //     //ROS_INFO("published frame %f ",::frame_count);
        // }
        // ::frame_count++;
    }

};

int main(int argc, char **argv){

    ros::init(argc, argv, "scan_to_point_cloud");
    ros::NodeHandle n;
    LaserScanToPointCloud lstopc(n);

    ros::spin();

    return 0;
}