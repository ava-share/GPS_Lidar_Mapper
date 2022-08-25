#!/usr/bin/env python
'''Author: Jonas Lossner
Description: Simple GPS based Lidar Mapper
Takes Lidar scan, filters out points that are too close or too high/low and transforms them based on GPS odometry (in UTM) 
once the rosbag is done playing run the following command in a seperate terminal to save into .pcd: 
rosrun --once pcl_ros pointcloud_to_pcd input:=/my_pcl_topic

Notes: the callback functions takes around 0.25 seconds to execute, depending on the amount of points per scan it may be 0.18 to 0.5 seconds - taking out the for loop would speed this up but cannot filter out close points if we do - so the data needs to be slowed down a lot (or buffer needs to be huge) 
Limitations: I have not figured out how to include the intensity value in the published/saved map
after you are done with processing and the file is saved run sed -i 's/insert_num_points/actual_number_of_points/' test1.pcd - the actual_number_of_points is the last number the print statement below shows in the terminal '''

import rospy
#import tf
from tf.transformations import  quaternion_matrix
from nav_msgs.msg import Odometry
#from geometry_msgs.msg import TwistStamped, PoseStamped
import math

from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import std_msgs.msg
import message_filters

class GPSMapper():

    def __init__(self):

        rospy.init_node('gps_mapper')
        self.scaled_polygon_pcl = PointCloud2()
        self.pcl_pub = rospy.Publisher("/my_pcl_topic", PointCloud2, queue_size=1)
#
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
        lidar_sub = message_filters.Subscriber('/filtered_points', PointCloud2) #change to desired pointcloud topic name /segmenter/points_ground
        ts = message_filters.ApproximateTimeSynchronizer([lidar_sub, odom_sub], 10,0.75) #queue = 100, allowed slop between timestamps = 0.05 
        ts.registerCallback(self.callback)

        self.rate = rospy.Rate(60.0)

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
		f.write("WIDTH insert_num_points")
		f.write('\n')
		f.write("HEIGHT 1")
		f.write('\n')
		f.write("VIEWPOINT 0 0 0 1 0 0 0")
		f.write('\n')
		f.write("POINTS insert_num_points")
		f.write('\n')
		f.write("DATA ascii")
		f.write('\n')

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

	    #adjust for offset between lidar and base_link since localization gives position for base_link, lidar is + 1.529 meters towards the front and 1.311 m higher than base_link
            self.x = msg_odom.pose.pose.position.x 
            self.y = msg_odom.pose.pose.position.y
	    self.z = msg_odom.pose.pose.position.z

            self.xr = msg_odom.pose.pose.orientation.x
            self.yr = msg_odom.pose.pose.orientation.y
            self.zr = msg_odom.pose.pose.orientation.z
            self.wr = msg_odom.pose.pose.orientation.w
	  
	    #compute rotation matrix from quaternion
	    self.rot = [self.xr, self.yr, self.zr, self.wr]
            self.rotation_matrix = quaternion_matrix(self.rot)
            self.rotation_matrix = np.delete(self.rotation_matrix, 3, axis=0)
            self.rotation_matrix1 = np.delete(self.rotation_matrix, 3, axis=1)
            pc = pc2.read_points(msg_lidar, skip_nans=True)

	    test = ''

            for p in pc:

		if ( math.sqrt(p[0]**2 + p[1]**2) < 2.8 and -1.5 < p[2] <1.5 ): #filter out points that are too close (e.g. other sensors or ego vehicle)
			pass#print("point too close - dropped")

		elif ( -5 < p[2] > 1 ):
			pass
		
		else:
			rotated_pc = self.rotation_matrix1 * np.asmatrix([[p[0]+1.529],[p[1]],[p[2]+1.311]]) #p[3] doesn't need to be transformed (just intensity)
			transformed_pc = np.array(rotated_pc + ([[self.x],[self.y],[self.z]]))

			if False:#[transformed_pc[0][0], transformed_pc[1][0], transformed_pc[2][0], []] in self.pc_list:
				pass
			else:
				self.points_counter = self.points_counter + 1
				print(self.points_counter)
				self.pc_list.append([transformed_pc[0][0], transformed_pc[1][0], transformed_pc[2][0], p[3]])
				test = ' '.join((test,str(transformed_pc[0][0]), str(transformed_pc[1][0]), str(transformed_pc[2][0]), str(p[3]), '\n'))

            with open('test1.pcd', 'a') as f: #note a instead of w to append and not overwrite the previous points
            	f.write(test)

            self.header.stamp = rospy.Time.now()

            self.scaled_polygon_pcl = self.create_cloud_xyzi32(self.header, self.pc_list)
            self.pcl_pub.publish(self.scaled_polygon_pcl)
if __name__ == '__main__':

    try:
	 
        GPSMapper()

    except rospy.ROSInterruptException:
        pass

