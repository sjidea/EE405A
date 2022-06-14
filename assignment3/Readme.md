### What I've done and learned
Matching the pre-built map and the scan points can be done by using PCL library. The input and target data is both pointcloud format. This assignment was to make bothICP matching and NDT matching. Both are able to align the data, produce the transform matrix, and calculate the score. Since ndt is using multi threads, it is faster and has higher performance. 
Running the code and debugging helped me use various ros commands such as rosmsg find, rosmsg show, rostopic list, rostopic echo. I learned how to run rviz and add data to the window. 


Todo 1 was to convert laser scan data(msg) to pointcloud type(cloud_in_ptr). 

Todo 2 We can uncomment the one which we want to use. 

Todo3 is converting nav_msgs/OccupancyGridConstPtr(msg) into pointcloud type(m_StaticMap_ptr). 

Todo4 is to set icp interfaces. MaximumIteration should be large if initial alignment is poor and small if initial alignment is moderate. InputSource and InputTarget is given as InputData and TargetData. MaxCorrespondencdDistance should be also set to 1. If not, the distance to the wall is too far for the icp to read. 
todo5

Todo5 is converting the pointcloud data(result) to pointcloud2 type(FinalCloudToPointCloud2Msg). This process is to convert the data to ros message type. 
todo6

Todo6 is pubsishing nav_msgs/Odometry type data(m_odomScan) from the aligned data(init_guess). Init_guess is a 4 x 4 matrix which has angle information and position information. The rightmost column is the positions, so put it into m_odomScan.pose.pose.position. The top left 3 x 3 matrix is the angle information, so put it into an arbitrary Eigen/Quaternionf variable and normalize and put it into m_odomScan.pose.pose.orientation. 
