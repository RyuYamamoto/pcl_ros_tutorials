#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher points_pub;
ros::Subscriber points_sub;

void points_callback(const sensor_msgs::PointCloud2::ConstPtr & msg)
{
  sensor_msgs::PointCloud2 output;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg(*msg, *cloud);

  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(0.5, 0.5, 0.5);
  sor.filter(*cloud_filtered);

  pcl::toROSMsg(*cloud_filtered, output);
  output.header = msg->header;
  output.header.stamp = ros::Time::now();

  points_pub.publish(output);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "voxel_grid_filter_node");
  ros::NodeHandle nh;

  points_pub = nh.advertise<sensor_msgs::PointCloud2>("filtered_points", 5);
  points_sub = nh.subscribe("points_raw", 5, points_callback);

  ros::spin();
  return 0;
}
