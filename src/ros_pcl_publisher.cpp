#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver2/CustomMsg.h>

#include <std_msgs/String.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/frustum_culling.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/transforms.h>

typedef pcl::PointXYZI         Point;
typedef pcl::PointCloud<Point> PointCloud;

class LivoxPCLPublisher {

  // ROS properties
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber point_cloud_sub_;

  // Publisher to send out the filtered point cloud
  ros::Publisher point_cloud_filtered_;

  // Filter parameters
  double robot_radius;

  // Variables
  PointCloud point_cloud;

public:

  /**
   * Class constructor
   */
  LivoxPCLPublisher() : nh_private_("~")
  {
    
    // Subscription to the point cloud result from stereo_image_proc
    point_cloud_sub_ = nh_private_.subscribe<livox_ros_driver2::CustomMsg>("input", 1, &LivoxPCLPublisher::pointCloudCb, this);

    // Declare the point cloud filtered topic
    point_cloud_filtered_ = nh_private_.advertise<PointCloud>("output", 1);
  }

  /**
   * Callback executed when a point cloud is recieved from topic "input".
   */
  void pointCloudCb(const livox_ros_driver2::CustomMsg::ConstPtr &msg)
  {

    point_cloud.clear();
    point_cloud.resize(msg->point_num);

    for(uint i=1; i < msg->point_num; i++)
    {
      point_cloud[i].x = msg->points[i].x;
      point_cloud[i].y = msg->points[i].y;
      point_cloud[i].z = msg->points[i].z;
      point_cloud[i].intensity = msg->points[i].reflectivity;

    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(point_cloud, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time::now();
    laserCloudmsg.header.frame_id = "livox";
    point_cloud_filtered_.publish(laserCloudmsg);
    // point_cloud_filtered_.publish(point_cloud);
  }

};

/**
 * Main entry point of the code
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "livox_pcl_publisher");
  LivoxPCLPublisher node;
  ros::spin();
  return 0;
}
