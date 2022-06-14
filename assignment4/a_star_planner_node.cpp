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

#include "a_star_planner/AStar.hpp"

class AstarPlanner
{
    public:
        AstarPlanner(ros::NodeHandle& nh);        
        ~AstarPlanner();
        // void CallbackOccupancyGrid(const nav_msgs::OccupancyGrid& msg);
        void CallbackGoalRviz(const geometry_msgs::PoseStamped& msg);
        void Gaussian(const sensor_msgs::PointCloud2& msg);

        ros::NodeHandle nh_;
       
    private:
        // ros::Subscriber subOccupancyGrid;
        ros::Subscriber subLocalPath;
        ros::Subscriber subGoalPose;

        ros::Publisher pubAstarPath; 
        ros::Publisher pubCollisionPoints;
        ros::Publisher pubGaussianPoints; //for gaussian points

        nav_msgs::Path m_LocalPath;
        geometry_msgs::PoseStamped m_GoalPose;
        bool bNewGoalPose;
};

AstarPlanner::AstarPlanner(ros::NodeHandle& nh) : nh_(nh), bNewGoalPose(false)
{
    // subOccupancyGrid = nh_.subscribe("/semantics/costmap_generator/occupancy_grid",1, &AstarPlanner::CallbackOccupancyGrid, this);
    subGoalPose = nh_.subscribe("/move_base_simple/goal",1, &AstarPlanner::CallbackGoalRviz, this);
    subLocalPath = nh_.subscribe("/velodyne_points", 1,&AstarPlanner::Gaussian, this);
    

    pubAstarPath = nh_.advertise<nav_msgs::Path>("/Path/LocalWaypoint/a_star", 1, true);
    pubCollisionPoints = nh_.advertise<sensor_msgs::PointCloud2>("/PointCloud2/a_star_collision_points", 1, true);
    pubGaussianPoints = nh_.advertise<nav_msgs::OccupancyGrid>("/OccupancyGrid/gaussian_points", 1, true);
};

AstarPlanner::~AstarPlanner() 
{    
    ROS_INFO("AstarPlanner destructor.");
}

void AstarPlanner::CallbackGoalRviz(const geometry_msgs::PoseStamped& msg)
{
    m_GoalPose = msg;
    bNewGoalPose = true;
}

// void AstarPlanner::CallbackOccupancyGrid(const nav_msgs::OccupancyGrid& msg)
// {
//     if(!bNewGoalPose)
//         return;
//     // std::cout << bNewGoalPose << std::endl;
    
//     double target_x = m_GoalPose.pose.position.x; //longitudinal
//     double target_y = m_GoalPose.pose.position.y; //lateral

//     int row, col, rotate_row, rotate_col;
//     int grid_x_size = msg.info.width;
//     int grid_y_size = msg.info.height;

//     AStar::Generator generator;
//     // Set 2d map size.
//     generator.setWorldSize({grid_x_size, grid_y_size});
//     // You can use a few heuristics : manhattan, euclidean or octagonal.
//     generator.setHeuristic(AStar::Heuristic::manhattan);
//     generator.setDiagonalMovement(false);

//     generator.clearCollisions();

//     pcl::PointCloud<pcl::PointXYZI>::Ptr collision_point_ptr(new pcl::PointCloud<pcl::PointXYZI>);

//     // TODO #1 Feel the code
//     // 1. set a origin and target in the grid map.  
//     // 2. Add collision using generator.addCollision()
//     // 3. Visualize a collision points using "collisionCloudMsg"

//     int origin_grid_x = grid_x_size/2; //200
//     int origin_grid_y = grid_y_size/2; //150

//     double target_grid_x = grid_x_size/2 + 10*target_x; 
//     double target_grid_y = grid_y_size/2 + 10*target_y;


//     for (int width = 0; width < grid_x_size; width++) {
//         for (int height = 0; height < grid_y_size; height++) {
// 		    if(msg.data[height * grid_x_size + width] > 10 ) {
//                 generator.addCollision({width, height},  1);
//             }
//         }
//     }

//     //visualize the collision points
//     sensor_msgs::PointCloud2 collisionCloudMsg;
//     pcl::toROSMsg(*collision_point_ptr, collisionCloudMsg);
//     collisionCloudMsg.header.frame_id = "base_link";
//     collisionCloudMsg.header.stamp = ros::Time::now();
//     pubCollisionPoints.publish(collisionCloudMsg);

//     // TODO #2: Feel the code
//     // 1. Implement A* using generator.findPath({origin_grid_x, origin_grid_y}, {target_grid_x, target_grid_y})
    
//     auto path = generator.findPath({origin_grid_x, origin_grid_y}, {target_grid_x, target_grid_y});

//     nav_msgs::Path AStartPathMsg;
//     AStartPathMsg.header.stamp = ros::Time::now();
//     AStartPathMsg.header.frame_id = "base_link";
    
//     for(int i= 0; i < path.size(); i++){
//         geometry_msgs::PoseStamped PS;
//         PS.pose.position.x = (path[i].x  - grid_x_size/2 )/10.0;
//         PS.pose.position.y = (path[i].y  - grid_y_size/2 )/10.0;
//         AStartPathMsg.poses.push_back(PS);
//     }

//     // TODO #3: Feel the code
//     // 1. Publish the result using nav_msgs/Path.msg  
    
//     pubAstarPath.publish(AStartPathMsg);

// }


// OPTIONAL------------------------------------------------------------------------------------
// 1. generate map
// 2. collision(gaussian)
// 3. find path & publish
//----------------------------------------------------------------------------------------------

void AstarPlanner::Gaussian(const sensor_msgs::PointCloud2& msg)
{
    if(!bNewGoalPose)
        return;


//GENERATE MAP

    // since the subsribed data has frame id of 'velodyne', clone the data and publish PointCloud2 type
    sensor_msgs::PointCloud2 CloudMsg;

    CloudMsg.header.frame_id = "base_link";
    CloudMsg.header.stamp = ros::Time::now();
    CloudMsg.height = msg.height;
    CloudMsg.width = msg.width;
    CloudMsg.fields = msg.fields;
    CloudMsg.is_bigendian = msg.is_bigendian;
    CloudMsg.point_step = msg.point_step;
    CloudMsg.row_step = msg.row_step;
    CloudMsg.data = msg.data;
    CloudMsg.is_dense = msg.is_dense;
    pubCollisionPoints.publish(CloudMsg);


    // build map of size 400*300 (40*30)
     nav_msgs::OccupancyGrid mymap;   

    int grid_x_size = 400;
    int grid_y_size = 300;

    mymap.info.resolution = 0.05;
    mymap.info.width = grid_x_size;
    mymap.info.height = grid_y_size;
    mymap.header.frame_id = "base_link";
    mymap.header.stamp = ros::Time::now();
    mymap.info.origin.position.x = -grid_x_size/(2*20);
    mymap.info.origin.position.y = -grid_y_size/(2*20);
    mymap.info.origin.position.z = 0; 
    // mymap.data not defined, will be difined after giving gaussian collisions


// COLLISION 1) make gaussian distribution 2) add collision 3) publish   

    // set origin and target
    int origin_grid_x = grid_x_size/2;
    int origin_grid_y = grid_y_size/2;

    double target_x = m_GoalPose.pose.position.x; //longitudinal
    double target_y = m_GoalPose.pose.position.y; //lateral
    double target_grid_x = grid_x_size/2 + 20*target_x; 
    double target_grid_y = grid_y_size/2 + 20*target_y;    

    // extract the (desired) points and store in arbitrary array 
    pcl::PointCloud<pcl::PointXYZI>::Ptr coll_pt(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg (CloudMsg, *coll_pt);


    int coll[120000] = {0};  // 400*300
    for (int i= 0; i< coll_pt->points.size(); i++) {  // extract every points in the data

        int PointBuf[2] = {0}; 
        PointBuf[0] = coll_pt->points[i].x *20 + grid_x_size/2; // convert the points to the affordable value
        PointBuf[1] = coll_pt->points[i].y *20 + grid_y_size/2;
        
        // store points only in the 400*300 range and points with z value of -2 < z < 2
            // storing is dome by giving value 100 to the desired element in the array
        if( PointBuf[0] < 0 || PointBuf[0] >= grid_x_size || PointBuf[1] < 0 || PointBuf[1] >= grid_y_size) continue;
        else if (coll_pt->points[i].z <-2 || coll_pt->points[i].z >2 ) continue;
        else{
            coll[PointBuf[1]*grid_x_size + PointBuf[0]] = 100;
        }
    }

    // create gaussian distribution 
    for (int width = 0; width < grid_x_size; width++) {
        for (int height = 0; height < grid_y_size; height++) {

            // create disbribution only for the neighboring 40*40 points
                // this can make the process faster, compared to giving value for the whole map
		    if(coll[height * grid_x_size + width]  == 100 ) {
                int w = std::max(0, width-20);
                int h = std::max(0, height-20);
                int w_M = std::min(grid_x_size, width+20);
                int h_M = std::min(grid_y_size, height+20);

                for (int x = w; x < w_M; x++){
                    for (int y = h; y < h_M; y++) {
                        int a = pow(x-width, 2); //from 0 to 400
                        int b = pow(y-height, 2);
                        int score =  50*exp(- a/20 - b/20) ;
                        score = (score > 100) ? 100 : score; // set maximum value to 100
                        if(score > coll[x + grid_x_size* y]){ 
                            // replace value if the value exceeds the original value
                            coll[x + grid_x_size* y] =  score;
                        }
                    }
                }
            }
        }
    }

    // push back the data into map data, map is all ready
    for (int i = 0; i< 120000; i++) {
        mymap.data.push_back(coll[i]);
    }
    // publish map
    pubGaussianPoints.publish(mymap);

    // add collision    
    AStar::Generator generator;
    generator.setWorldSize({grid_x_size, grid_y_size});
    generator.setHeuristic(AStar::Heuristic::manhattan);
    generator.setDiagonalMovement(false);
    generator.clearCollisions();


    for (int width = 0; width < grid_x_size; width++) {
        for (int height = 0; height < grid_y_size; height++) {
		    if(mymap.data[height * grid_x_size + width] > 10 ) { // add collision points with value exceeding 10
                generator.addCollision({width, height},  1);
            }
        }
    }

// FIND PATH AND PUBLISH
    auto path = generator.findPath({origin_grid_x, origin_grid_y}, {target_grid_x, target_grid_y});

    nav_msgs::Path AStartPathMsg;
    AStartPathMsg.header.stamp = ros::Time::now();
    AStartPathMsg.header.frame_id = "base_link";
    
    for(int i= 0; i < path.size(); i++){
        geometry_msgs::PoseStamped PS;
        PS.pose.position.x = (path[i].x  - grid_x_size/2 )/20.0;
        PS.pose.position.y = (path[i].y  - grid_y_size/2 )/20.0;
        AStartPathMsg.poses.push_back(PS);
    }
    pubAstarPath.publish(AStartPathMsg);



}

//---------------------------------------------------------------------------------------------


int main(int argc, char* argv[])
{   
    ros::init(argc, argv, "a_star_planner");
    // for subscribe
    ros::NodeHandle nh;
    AstarPlanner planner(nh);

    ros::spin();
    return 0;

}

