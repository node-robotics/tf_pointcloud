#include <ros/ros.h>
#include <ros/console.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>


ros::Publisher pub;
tf::StampedTransform transform;

void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Create a container for the data.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

    pcl_ros::transformPointCloud(*temp_cloud, *cloud_transformed, transform);

    sensor_msgs::PointCloud2 cloud_publish;
    pcl::toROSMsg(*cloud_transformed,cloud_publish);
    cloud_publish.header = input->header;
    cloud_publish.header.frame_id = "map";

    pub.publish(cloud_publish);
}

int main (int argc, char** argv){

    // Initialize ROS
    ROS_INFO("Node started");

    ros::init (argc, argv, "point_cloud_transform");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/pointcloud_unified", 1, cloud_callback);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("/pointcloud_unified/map_frame", 1);

    tf::TransformListener listener;
    ros::Rate rate(10.0);
    while (nh.ok()){

        try{
            listener.lookupTransform("map", "base_link", ros::Time(), transform);
        }
        catch (tf::TransformException ex){
            ROS_WARN("%s",ex.what());
        }

        ros::spinOnce();
        rate.sleep();
    }

    ros::spin();
    return 0;

}
