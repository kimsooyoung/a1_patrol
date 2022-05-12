// referenced from : http://wiki.ros.org/laser_pipeline/Tutorials/IntroductionToWorkingWithLaserScannerData

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <laser_geometry/laser_geometry.h>

class LaserScanToPointCloud
{
public:
    ros::NodeHandle n_;
    ros::Publisher scan_pub_;
    
    laser_geometry::LaserProjection projector_;
    
    tf::TransformListener listener_;
    tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
    
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;

    LaserScanToPointCloud(ros::NodeHandle n) : n_(n),
                                               laser_sub_(n_, "scan", 0),
                                               laser_notifier_(laser_sub_, listener_, "base_link", 10)
    {
        laser_notifier_.registerCallback(
            boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
        laser_notifier_.setTolerance(ros::Duration(0.01));
        scan_pub_ = n_.advertise<sensor_msgs::PointCloud>("/my_cloud", 1);

        ROS_INFO("my_scan_to_cloud node started");
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_in)
    {
        sensor_msgs::PointCloud cloud;

        std::cout << "Transform strat " << std::endl;

        try
        {
            projector_.transformLaserScanToPointCloud(
                "base_link", *scan_in, cloud, listener_);

            std::cout << "Transform succeed" << std::endl;
        }
        catch (tf::TransformException &e)
        {
            std::cout << e.what();
            return;
        }

        // Do something with cloud.

        scan_pub_.publish(cloud);
    }
};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "my_scan_to_cloud");
    ros::NodeHandle n;
    LaserScanToPointCloud lstopc(n);

    ros::spin();

    return 0;
}