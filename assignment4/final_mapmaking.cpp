#include <iostream>
#include <cmath>
#include <cstdlib>
#include <vector>
#include <string>
#include <fstream>

// pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//tf
#include "tf/transform_datatypes.h"
#include <tf2_msgs/TFMessage.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// ros
#include "ros/ros.h"
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>

class MapMaking
{
    public:
        MapMaking(ros::NodeHandle& nh);        
        ~MapMaking();

        // void LaserData(const sensor_msgs::LaserScan);
        // void IMUData(const sensor_msgs::Imu);

        void scanData(const sensor_msgs::LaserScan, const sensor_msgs::Imu);

        ros::NodeHandle nh_;

    private:
        ros::Subscriber subLaser:
        ros::Subscriber subImu:
        ros::Publisher pubOccupancyGrid;

}

MapMaking::MapMaking(ros::NodeHandle& nh) : nh_(nh)
{
    // subLaser = nh_.subscribe("/scan", 1, &MapMaking::LaserData, this);
    // subImu = nh_.subscribe("/imu/data_raw", 1, &MapMaking::IMUData, this);
    subLaser = nh_.subscribe("/scan", 1, &MapMaking::scanData, this);
    subImu = nh_.subscribe("/imu/data_raw", 1, &MapMaking::scanData, this);
    pubOccupancyGrid = nh_.advertise<nav_msgs::OccupancyGrid>("/OccupancyGrid/gaussian_points", 1, true);
}


void MapMaking::scanData(const sensor_msgs::LaserScan& msg1, const sensor_msgs::Imu& msg2)
{
    // Generate MAP
}
