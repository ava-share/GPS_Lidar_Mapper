# GPS_Lidar_Mapper
This script takes in Pointcloud2 and Odometry messages and computes a .pcd map that is globally located and oriented correctly.

# How to use
1. record data in a rosbag - topics needed are any odometry message and any pointcloud2 message
2. replay rosbag very slowly (rosbag play -r 0.1 *bagname.bag*) worked well for our testing.
  We recommend using some filter (distance filter or other filter that downsamples # of points) to be used as input into this script
3. Let it run until rosbag is finished 
4. The .pcd can now be used

# TO DO 
1. DONE - change code to automatically replace the "insert_num_points" with the number of points on exit
2. Make this a ROS node and allow for parameters to be passed (so changing topic names etc. is easier)
3. name pcds based on timestamp rather than always name it test1.pcd

# TBD improvements
1. Using Kalman Filter 
2. Handle memory better or significantly optimize the code so it can run faster (currently need to slow down rosbag significantly
3. extract road or lane markings where possible similiar to this https://github.com/kwh950724/lidar_lane_detector


