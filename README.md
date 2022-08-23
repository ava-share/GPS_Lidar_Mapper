# GPS_Lidar_Mapper
This script takes in Pointcloud2 and Odometry messages and computes a .pcd map that is globally located and oriented correctly.

# How to use
1. record data in a rosbag - topics needed are any odometry message and any pointcloud2 message
2. replay rosbag very slowly (rosbag play -r 0.2 *bagname.bag*) worked well for our testing.
  We recommend using some filter (distance filter or other filter that downsamples # of points) to be used as input into this script
3. Let it run until rosbag is finished - check if the number that is printed stops changing. 
4. Once the number in the terminal stops changing processing is done you will need to change the Header information for the pcd 
5. in terminal type in: sed -i '10,%, s/insert_num_points/*copy the last number from terminal and paste here*/' test1.pcd 
6. The .pcd can now be used
