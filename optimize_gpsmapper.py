#!/usr/bin/env python
'''Author: Jonas Lossner
Description: Simple GPS based Lidar Mapper
Takes Lidar scan, filters out points that are too close or too high/low and transforms them based on GPS odometry (in UTM) 
once the rosbag is done playing run the following the exit_hook will write the points to a .pcd file

Notes: the callback functions takes around 0.25 seconds to execute, depending on the amount of points per scan it may be 0.18 to 0.5 seconds - taking out the for loop would speed this up but cannot filter out close points if we do - so the data needs to be slowed down a lot (or buffer needs to be huge) 
'''

import rospy
#import tf
from tf.transformations import  quaternion_matrix
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

import sensor_msgs.point_cloud2 as pc2
import numpy as np
import std_msgs.msg
import message_filters
import math

class GPSMapper():

    def exit_hook(self):

	print("Shutting down and saving to .pcd file:")
	rospy.sleep(2)
	print("Saved")
	num_points = self.points_counter
	cloud = self.test
	print("Saved " +str(num_points) + " to test1.pcd")
	with open('test1.pcd', 'w') as f: #write points to text file..
		f.write("# .PCD v0.7 - Point Cloud Data file format")
		f.write('\n')
		f.write("VERSION 0.7")
		f.write('\n')
		f.write("FIELDS x y z intensity")
		f.write('\n')
		f.write("SIZE 4 4 4 4")
		f.write('\n')
		f.write("TYPE F F F F")
		f.write('\n')
		f.write("COUNT 1 1 1 1")
		f.write('\n')
		f.write("WIDTH " + str(num_points))
		f.write('\n')
		f.write("HEIGHT 1")
		f.write('\n')
		f.write("VIEWPOINT 0 0 0 1 0 0 0")
		f.write('\n')
		f.write("POINTS "+ str(num_points))
		f.write('\n')
		f.write("DATA ascii")
		f.write('\n')
		f.write(cloud)

    def __init__(self):

        rospy.init_node('gps_mapper')
        rospy.on_shutdown(self.exit_hook)
	print("Started Mapping Tool")
        self.scaled_polygon_pcl = PointCloud2()
	#visualization_param = rospy.get_param('viz_param', 'True')
        self.pcl_pub = rospy.Publisher("/my_pcl_topic", PointCloud2, queue_size=1)
        self.test = ''
	self.points_counter = 0
        self.x = 0
        self.y = 0
        self.z = 0
        self.xr = 0
        self.yr = 0
        self.zr = 0
        self.wr = 0
        self.pc_list = []
        self.header = std_msgs.msg.Header()
        self.header.frame_id = 'map'

        odom_sub = message_filters.Subscriber('/novatel/oem7/odom', Odometry) # change to desired odometry topic name
        lidar_sub = message_filters.Subscriber('/filtered_points', PointCloud2) #change to desired pointcloud topic name /filtered_points
        ts = message_filters.ApproximateTimeSynchronizer([lidar_sub, odom_sub], 10,0.75) #queue = 100, allowed slop between timestamps = 0.05 
        ts.registerCallback(self.callback)

        self.rate = rospy.Rate(60.0)

        while not rospy.is_shutdown():
            self.rate.sleep()

    def create_cloud_xyzi32(self,header, points):

    	fields = [pc2.PointField(name='x', offset=0,
                         datatype=pc2.PointField.FLOAT32, count=1),
    	pc2.PointField(name='y', offset=4,
                         datatype=pc2.PointField.FLOAT32, count=1),
    	pc2.PointField(name='z', offset=8,
                         datatype=pc2.PointField.FLOAT32, count=1),
   	 pc2.PointField(name='intensity', offset=12,
                         datatype=pc2.PointField.FLOAT32, count=1)]
    	return pc2.create_cloud(header, fields, points)

    def callback(self, msg_lidar, msg_odom):

            self.x = msg_odom.pose.pose.position.x 
            self.y = msg_odom.pose.pose.position.y
	    self.z = msg_odom.pose.pose.position.z

            self.xr = msg_odom.pose.pose.orientation.x
            self.yr = msg_odom.pose.pose.orientation.y
            self.zr = msg_odom.pose.pose.orientation.z
            self.wr = msg_odom.pose.pose.orientation.w
	  
	    #get rotation matrix from quaternion
	    self.rot = [self.xr, self.yr, self.zr, self.wr]
            self.rotation_matrix = quaternion_matrix(self.rot)
            self.rotation_matrix = np.delete(self.rotation_matrix, 3, axis=0)
            self.rotation_matrix1 = np.delete(self.rotation_matrix, 3, axis=1)
            pc = pc2.read_points(msg_lidar, skip_nans=True)

            for p in pc:

		if ( math.sqrt(p[0]**2 + p[1]**2) < 2.8 and -1.5 < p[2] <1.5 ): #filter out points that are too close (e.g. other sensors or ego vehicle)
			pass

		elif ( -5 < p[2] > 1 ): #filter out points that are below 5 meters or above 1 meter from the sensor
			pass
		
		else:
			rotated_pc = self.rotation_matrix1 * np.asmatrix([[p[0]+1.529],[p[1]],[p[2]+1.311]]) # rotate to global orientation - p[3] doesn't need to be transformed (just intensity)
			transformed_pc = np.array(rotated_pc + ([[self.x],[self.y],[self.z]])) #transform to global position

			if False: #will use if for later implementation to choose if visualization is desired or not
				pass
			else:
				self.points_counter = self.points_counter + 1
				#print(self.points_counter)
				self.pc_list.append([transformed_pc[0][0], transformed_pc[1][0], transformed_pc[2][0], p[3]])
				self.test = ' '.join((self.test,str(transformed_pc[0][0]), str(transformed_pc[1][0]), str(transformed_pc[2][0]), str(p[3]), '\n'))


            self.header.stamp = rospy.Time.now()
            self.scaled_polygon_pcl = self.create_cloud_xyzi32(self.header, self.pc_list)
            self.pcl_pub.publish(self.scaled_polygon_pcl)

if __name__ == '__main__':

    try:
	 
        GPSMapper()

    except rospy.ROSInterruptException:
        pass
