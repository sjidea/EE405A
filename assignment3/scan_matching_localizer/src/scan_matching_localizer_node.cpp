/*
 * scan_matching_localizer_node.cpp
 *
 *  Created on: Apr 30, 2021
 *      Author: Daegyu Lee
 */

// NOTE
// 1. Please submit a report discussing the result of registration algorithm,
// 2. Please feel the code where I wrote as "TODO:#".
// 3. Import 2-D occupancy grid map using map_server(please refer to launch file.) 
// 4. You should give a initial pose using RVIZ 2D Pose Estimate tool.
// 5. If you have query things, feel free to send me a mail : "lee.dk@kaist.ac.kr"
// 6. Run a bag file and compare the result.


// headers in ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
// headers in STL
#include <memory>
#include <cmath>
#include <type_traits>
#include <stdio.h>
#include <float.h>
#include <vector>
#include <queue>
#include <deque>
#include <algorithm>
#include <unordered_map>
#include <bits/stdc++.h>
#include <mutex>
#include <thread>

// PCL Libraries
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/don.h>

#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pclomp/ndt_omp.h>

class ScanLocalizer
{
    public:
        ScanLocalizer(ros::NodeHandle& nh);        
        ~ScanLocalizer();
        void LaserScanCallback(const sensor_msgs::LaserScanConstPtr& msg);
        void OccupancyGrid2DMapCallback(const nav_msgs::OccupancyGridConstPtr& msg);
        void InitPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);
        void ICPMatching(pcl::PointCloud<pcl::PointXYZI>::Ptr &InputData, pcl::PointCloud<pcl::PointXYZI>::Ptr &TargetData);
        void NDTMatching(pcl::PointCloud<pcl::PointXYZI>::Ptr &InputData, pcl::PointCloud<pcl::PointXYZI>::Ptr &TargetData);
        
        ros::NodeHandle nh_;
        ros::Subscriber subScan;
        ros::Subscriber subOccupancyGrid;
        ros::Subscriber subInitPose;

        ros::Publisher pubPointCloud;
        ros::Publisher pubStaticMapPointCloud;
        ros::Publisher pubTransformedCloud;
        ros::Publisher pubOdometry;

        nav_msgs::Odometry m_odomScan;
    
        pcl::PointCloud<pcl::PointXYZI>::Ptr m_StaticMap_ptr;
        bool bRvizInit;
        Eigen::Matrix4f prev_guess, init_guess;
};

ScanLocalizer::ScanLocalizer(ros::NodeHandle& nh) : nh_(nh), bRvizInit(false)
{
    subScan = nh_.subscribe("/scan",1, &ScanLocalizer::LaserScanCallback, this);
    subOccupancyGrid = nh_.subscribe("/map",1, &ScanLocalizer::OccupancyGrid2DMapCallback, this);
    subInitPose = nh.subscribe("/initialpose", 1, &ScanLocalizer::InitPoseCallback, this);

    pubPointCloud = nh_.advertise<sensor_msgs::PointCloud2>("/scan_to_pointcloud2", 1, true);
    pubStaticMapPointCloud = nh_.advertise<sensor_msgs::PointCloud2>("/map_to_pointcloud2", 1, true);
    pubTransformedCloud = nh_.advertise<sensor_msgs::PointCloud2>("/registered_points", 1, true);
    pubOdometry = nh_.advertise<nav_msgs::Odometry>("/odom", 1, true);
};

ScanLocalizer::~ScanLocalizer() 
{    
    ROS_INFO("ScanLocalizer destructor.");
}

void ScanLocalizer::InitPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    Eigen::Matrix3f mat3 = Eigen::Quaternionf(msg.pose.pose.orientation.w, 
                                              msg.pose.pose.orientation.x, 
                                              msg.pose.pose.orientation.y, 
                                              msg.pose.pose.orientation.z).toRotationMatrix();
    prev_guess.block(0,0,3,3) = mat3;
    prev_guess(0,3) = msg.pose.pose.position.x;
    prev_guess(1,3) = msg.pose.pose.position.y;    

    
    prev_guess(0, 2) = 0;
    prev_guess(1, 2) = 0;
    prev_guess(2, 0) = 0;
    prev_guess(2, 1) = 0;
    prev_guess(2, 3) = 0;
    prev_guess(3, 0) = 0;
    prev_guess(3, 1) = 0;
    prev_guess(3, 2) = 0;
    prev_guess(3, 3) = 1;

    bRvizInit = true;
}

void ScanLocalizer::LaserScanCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    
    if(msg->ranges.empty())
    {
        ROS_ERROR("Empty scan data");
        return;
    }

    // Feel the code to convert the laser scan data to the pointcloud type(pcl::PointCloud<pcl::PointXYZI>)    
    for(int i = 0; i < msg->ranges.size(); i++)
    {
        pcl::PointXYZI pointBuf;
        //TODO:#1----------------------------------------------------------------------------------------------------------
        // feel the code here
        if (msg->ranges[i] < msg->range_min){ 
            pointBuf.x = std::numeric_limits<float>::quiet_NaN();
            pointBuf.y = std::numeric_limits<float>::quiet_NaN();
            pointBuf.z = std::numeric_limits<float>::quiet_NaN();
            cloud_in_ptr->is_dense = false;
        }
        else if( msg->ranges[i] > msg->range_max){
            pointBuf.x = (msg->range_max+0.001) * std::cos(msg->angle_min + i*msg->angle_increment);
            pointBuf.y = (msg->range_max+0.001) * std::sin(msg->angle_min + i*msg->angle_increment);
            pointBuf.z = 0; 
        } else {
            pointBuf.x = msg->ranges[i] * std::cos(msg->angle_min + i*msg->angle_increment);
            pointBuf.y = msg->ranges[i] * std::sin(msg->angle_min + i*msg->angle_increment);
            pointBuf.z = 0;
        }
        cloud_in_ptr->points.push_back(pointBuf);
        //----------------------------------------------------------------------------------------------------------
    }

    sensor_msgs::PointCloud2 LaserToPointCloud2Msg;
    pcl::toROSMsg(*cloud_in_ptr, LaserToPointCloud2Msg);
    LaserToPointCloud2Msg.header  = msg->header;
    pubPointCloud.publish(LaserToPointCloud2Msg);


    /*Scan matching algorithm*/
    if(cloud_in_ptr->points.empty() || m_StaticMap_ptr->points.empty())
        return;
    

    // Implement below two algorithm and compare the result.
    // You can switch the registration algorithm between ICP and NDT_OMP
    // TODO:#2
    // 1. ICP(No multi-thread)
    // ICPMatching(cloud_in_ptr, m_StaticMap_ptr); //Too slow
    // 2. NDT_OMP(multi-thread)
    NDTMatching(cloud_in_ptr, m_StaticMap_ptr);
}

void ScanLocalizer::OccupancyGrid2DMapCallback(const nav_msgs::OccupancyGridConstPtr& msg)
{
    // Feel the code to convert the laser scan data to the pointcloud type(pcl::PointCloud<pcl::PointXYZI>)    
    m_StaticMap_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>());
    for (unsigned int width = 0; width < msg->info.width; width++) {
        for (unsigned int height = 0; height < msg->info.height; height++) {
		    pcl::PointXYZI pointBuf;
            //TODO:#3----------------------------------------------------------------------------------------------------------
            // feel the code here.
            //Convert occupied grid to the x-y coordinates to put the target(pcl::PointCloud<pcl::PointXYZI>)
		    if(msg->data[height * msg->info.width + width] > 80 ) {
                pointBuf.x =  width * msg->info.resolution + msg->info.resolution/2 + msg->info.origin.position.x;
                pointBuf.y = height * msg->info.resolution + msg->info.resolution/2 + msg->info.origin.position.y;
                m_StaticMap_ptr->points.push_back(pointBuf);
            }
            //----------------------------------------------------------------------------------------------------------
        }
    }

    sensor_msgs::PointCloud2 StaticMapToPointCloud2Msg;
    pcl::toROSMsg(*m_StaticMap_ptr, StaticMapToPointCloud2Msg);
    StaticMapToPointCloud2Msg.header.frame_id = "map";
    StaticMapToPointCloud2Msg.header.stamp = msg->header.stamp;
    pubStaticMapPointCloud.publish(StaticMapToPointCloud2Msg);

    ROS_INFO("MAP IS LOADED");  
}


void ScanLocalizer::ICPMatching(pcl::PointCloud<pcl::PointXYZI>::Ptr &InputData, pcl::PointCloud<pcl::PointXYZI>::Ptr &TargetData)
{
    // You should initialize the init pose using RVIZ 2D pose estimate tool.
    if (!bRvizInit)
    {
        init_guess = Eigen::Matrix4f::Identity();
    }
    else
    {
        init_guess = prev_guess;
    }
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
    // set the MaximumIterations, InputSource, InputTarget
    // TODO:#4----------------------------------------------------------------------------------------------------------
    // feel the code here
    icp.setMaximumIterations(32); 
    icp.setInputSource(InputData);
    icp.setInputTarget(TargetData);
    icp.setMaxCorrespondenceDistance (1);

    //----------------------------------------------------------------------------------------------------------

    // Run registration algorithm, and put the transformation matrix of previous step.
    pcl::PointCloud<pcl::PointXYZI>::Ptr result(new pcl::PointCloud<pcl::PointXYZI>());
    icp.align(*result, init_guess);

    if (icp.hasConverged())
    {
        std::cout << "converged." << std::endl
                << "The score is " << icp.getFitnessScore() << std::endl;
        std::cout << "bRbizInit:" << bRvizInit << std::endl;
        std::cout << "Transformation matrix:" << std::endl;
        std::cout << icp.getFinalTransformation() << std::endl;
    }
    else 
    {   
        return;
    }

    init_guess.block<3, 3>(0, 0) = icp.getFinalTransformation().block<3, 3>(0, 0);
    init_guess.block<3, 1>(0, 3) = icp.getFinalTransformation().block<3, 1>(0, 3);
    
    //publish registered cloud 
    //Convert transformed(registered) pointcloud using ICP algorithm.
    pcl::PointCloud<pcl::PointXYZI>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    
    //TODO:#5
    //feel the code here --> convert pcl pointcloud to the ros msg.  
    sensor_msgs::PointCloud2 FinalCloudToPointCloud2Msg;
   
    pcl::toROSMsg(*result, FinalCloudToPointCloud2Msg);
    FinalCloudToPointCloud2Msg.header.frame_id = "map";
    FinalCloudToPointCloud2Msg.header.stamp = ros::Time::now();
    pubTransformedCloud.publish(FinalCloudToPointCloud2Msg);// topic name: "/registered_points"

    //----------------------------------------------------------------------------------------------------------

    //publish Odometry
    //TODO:#6----------------------------------------------------------------------------------------------------------
    //feel the code here --> Publish the result using nav_msgs/Odometry topic.  

    m_odomScan.header.frame_id = "map";
    m_odomScan.header.stamp = ros::Time::now();
    
    m_odomScan.pose.pose.position.x = init_guess(0,3);
    m_odomScan.pose.pose.position.y = init_guess(1,3); 
    m_odomScan.pose.pose.position.z = 0;
    
    Eigen::Quaternionf quat(init_guess.block<3, 3>(0, 0));       
    quat.normalize(); 

    m_odomScan.pose.pose.orientation.x = quat.x();
    m_odomScan.pose.pose.orientation.y = quat.y();
    m_odomScan.pose.pose.orientation.z = quat.z();
    m_odomScan.pose.pose.orientation.w = quat.w();
    
    //----------------------------------------------------------------------------------------------------------
    pubOdometry.publish(m_odomScan); //topic name: "/odom"

    prev_guess = init_guess;
}

void ScanLocalizer::NDTMatching(pcl::PointCloud<pcl::PointXYZI>::Ptr &InputData, pcl::PointCloud<pcl::PointXYZI>::Ptr &TargetData)
{
    if (!bRvizInit)
    {
        init_guess = Eigen::Matrix4f::Identity();
    }
    else
    {
        init_guess = prev_guess;
    }

    boost::shared_ptr<pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>> ndt(new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    ndt->setInputSource(InputData);
    ndt->setInputTarget(TargetData);    
    ndt->setTransformationEpsilon(0.01); 
    ndt->setMaximumIterations(32); 
    ndt->setResolution(1.0);
    ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    ndt->setNumThreads(5);

    //TODO:#7----------------------------------------------------------------------------------------------------------
    //Feel the code here
    // 1. Run registration algorithm(align)
	pcl::PointCloud<pcl::PointXYZI>::Ptr result(new pcl::PointCloud<pcl::PointXYZI>());
    ndt->align(*result, init_guess);

    // 2. Evaluate the registration algorithm using threshold(score)
	if (ndt->hasConverged())
    {
        std::cout << "converged." << std::endl
            << "The score is " << ndt->getFitnessScore() << std::endl;
        std::cout << "Transformation matrix:" << std::endl;
        std::cout << ndt->getFinalTransformation() << std::endl;
    }
    else 
    {   
        return;
    }

    // 3. Convert a Final transformation matrix to the init_guess in order to run a align function.
    init_guess.block<3, 3>(0, 0) = ndt->getFinalTransformation().block<3, 3>(0, 0);
    init_guess.block<3, 1>(0, 3) = ndt->getFinalTransformation().block<3, 1>(0, 3);

    // 4. Publish registered cloud
    sensor_msgs::PointCloud2 FinalCloudToPointCloud2Msg;
    pcl::toROSMsg(*result, FinalCloudToPointCloud2Msg);
    FinalCloudToPointCloud2Msg.header.frame_id = "map";
	FinalCloudToPointCloud2Msg.header.stamp = ros::Time::now();
    pubTransformedCloud.publish(FinalCloudToPointCloud2Msg);// topic name: "/registered_points"

    // 5. Publish Odometry
    m_odomScan.header.frame_id = "map";
    m_odomScan.header.stamp = ros::Time::now();

    m_odomScan.pose.pose.position.x = init_guess(0,3);
    m_odomScan.pose.pose.position.y = init_guess(1,3); 
    m_odomScan.pose.pose.position.z = 0;

    Eigen::Quaternionf quat(init_guess.block<3, 3>(0, 0));       
    quat.normalize(); 

    m_odomScan.pose.pose.orientation.x = quat.x();
    m_odomScan.pose.pose.orientation.y = quat.y();
    m_odomScan.pose.pose.orientation.z = quat.z();
    m_odomScan.pose.pose.orientation.w = quat.w();
	  
    pubOdometry.publish(m_odomScan); //topic name: "/odom"
    //----------------------------------------------------------------------------------------------------------
    prev_guess = init_guess;
}

int main(int argc, char** argv)
{    
    // node name initialization
    ros::init(argc, argv, "scan_matching_localizer_node");

    ros::NodeHandle nh;
    ScanLocalizer localizer(nh);

    ros::spin();

    return 0;
}

