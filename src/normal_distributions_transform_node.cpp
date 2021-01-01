#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>

ros::Subscriber points_cloud_subscriber_1;
ros::Subscriber points_cloud_subscriber_2;
ros::Publisher points_cloud_publisher;

sensor_msgs::PointCloud2::ConstPtr points_cloud1_msg = nullptr;
sensor_msgs::PointCloud2::ConstPtr points_cloud2_msg = nullptr;

void callbackPoints1(const sensor_msgs::PointCloud2::ConstPtr & cloud)
{
  points_cloud1_msg = cloud;
}

void callbackPoints2(const sensor_msgs::PointCloud2::ConstPtr & cloud)
{
  points_cloud2_msg = cloud;
}

void downsample(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & in, pcl::PointCloud<pcl::PointXYZ>::Ptr & out)
{
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> filter;
  filter.setLeafSize(0.2, 0.2, 0.2);
  filter.setInputCloud(in);
  filter.filter(*out);
}

void calcNDT(const ros::TimerEvent & e)
{
  if (points_cloud1_msg == nullptr && points_cloud2_msg == nullptr) return;

  pcl::PointCloud<pcl::PointXYZ>::Ptr points_cloud1(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr points_cloud2(new pcl::PointCloud<pcl::PointXYZ>());

  pcl::fromROSMsg(*points_cloud1_msg, *points_cloud1);
  pcl::fromROSMsg(*points_cloud2_msg, *points_cloud2);

  downsample(points_cloud1, filtered_cloud);

  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  ndt.setTransformationEpsilon(0.01);
  ndt.setStepSize(0.1);
  ndt.setResolution(1.0);
  ndt.setMaximumIterations(35);

  ndt.setInputSource(filtered_cloud);
  ndt.setInputTarget(points_cloud2);

  Eigen::AngleAxisf init_rotation(0.6931, Eigen::Vector3f::UnitZ());
  Eigen::Translation3f init_translation(1.79387, 0.720047, 0);
  Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

  pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
  ndt.align(*output, init_guess);

  pcl::transformPointCloud(*points_cloud1, *output, ndt.getFinalTransformation());

  sensor_msgs::PointCloud2 output_msg;
  pcl::toROSMsg(*output, output_msg);
  output_msg.header = points_cloud1_msg->header;
  points_cloud_publisher.publish(output_msg);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "normal_distributions_transform_node");

  ros::NodeHandle nh;

  points_cloud_subscriber_1 = nh.subscribe("pointcloud_1", 5, callbackPoints1);
  points_cloud_subscriber_2 = nh.subscribe("pointcloud_2", 5, callbackPoints2);
  points_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("output", 5);

  ros::Timer timer = nh.createTimer(ros::Duration(0.1), calcNDT);
  ros::spin();
  return 0;
}
