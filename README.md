# GPS_Lidar_Mapper
This script takes in Pointcloud2 and Odometry messages and computes a .pcd map that is globally located and oriented correctly.

# Update
new_gpsmapper.py is a revised version of the previous code (optimize_gpsmapper.py).
The new version runs at roughly 10 Hz (around 0.1 seconds per callback loop) whereas the previous version took 3+ seconds per callback loop. 
The new version requires a new package: ros-numpy (sudo apt install ros-melodic-ros-numpy). 

There are a few parameters that can be changed - x, y, z limits to crop the input pointcloud down to a Area of Interest which can reduce load. Ultimately the input to this map should likely be a ground segmented pointcloud or otherwise pre-filtered pointcloud such as voxel filter or similiar to further reduce computational load. 

by default they are: 
lim_x=[3, 80] (starts 3m ahead of the sensor up to 80 m way)
lim_y=[-15,15] (15 m to the left and right of the sensor)
lim_z=[-10,5] (10 meters below and up to 5 meters above the sensor)

# How to use (updated)
1. record data in a rosbag - topics needed are any odometry message and any pointcloud2 message
2. replay rosbag (rosbag play --pause -r xx *bagname.bag*) - the lidar pointcloud should be published at 10 Hz, if the lidar is published at 20 Hz reduce the rate (xx) to 0.5 otherwise you can leave it at 1)
3. change lim_x, lim_y, lim_z to desired values
4. python new_gpsmapper.py
5. unpause rosbag by hitting the spacebar in it's terminal
6. Let it run until rosbag is finished or pause the rosbag, this will let the mapper catch up in case it was behind.
7. In the terminal where the gpsmapper is running hit Ctrl+c - wait until it shows the pointcloud was saved and the program exited fully
8. The .pcd can now be used

# How to use (original)
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
2. DONE (in new_gpsmapper.py) Handle memory better or significantly optimize the code so it can run faster (currently need to slow down rosbag significantly
3. extract road or lane markings where possible similiar to this https://github.com/kwh950724/lidar_lane_detector (won't do this as not always applicable)


