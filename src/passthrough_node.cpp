#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher point_pub;
ros::Subscriber point_sub;

void points_callback(const sensor_msgs::PointCloud2::ConstPtr & msg)
{
  sensor_msgs::PointCloud2 output;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg(*msg, *cloud);

  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, 1.0);
  //pass.setFilterLimitsNegative (true); // trueにするとLimitsの範囲内の点群を返す(default false)
  pass.filter(*cloud_filtered);

  pcl::toROSMsg(*cloud_filtered, output);
  output.header = msg->header;
  output.header.stamp = ros::Time::now();

  point_pub.publish(output);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "passthrough_node");
  ros::NodeHandle nh;

  point_sub = nh.subscribe("points_raw", 5, points_callback);
  point_pub = nh.advertise<sensor_msgs::PointCloud2>("filtered_points", 5);

  ros::spin();

  return 0;
}
