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
import ros_numpy

import sensor_msgs.point_cloud2 as pc2
import numpy as np
import std_msgs.msg
import message_filters
import math

lim_x=[3, 50]
lim_y=[-15,15]
lim_z=[-5,5]

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
		#self.transformed_pc = []
		odom_sub = message_filters.Subscriber('/novatel/oem7/odom', Odometry) # change to desired odometry topic name
		lidar_sub = message_filters.Subscriber('/lidar_tc/velodyne_points', PointCloud2) #change to desired pointcloud topic name /filtered_points
		ts = message_filters.ApproximateTimeSynchronizer([lidar_sub, odom_sub], 10,0.75) #queue = 100, allowed slop between timestamps = 0.05 
		ts.registerCallback(self.callback)

		self.rate = rospy.Rate(60.0)

		while not rospy.is_shutdown():
			self.rate.sleep()

	def crop_pointcloud(self, pointcloud):
		# remove points outside of detection cube defined in 'configs.lim_*'
		mask = np.where((pointcloud[:, 0] >= lim_x[0]) & (pointcloud[:, 0] <= lim_x[1]) & (pointcloud[:, 1] >=lim_y[0]) & (pointcloud[:, 1] <= lim_y[1]) & (pointcloud[:, 2] >= lim_z[0]) & (pointcloud[:, 2] <= lim_z[1]))
		pointcloud = pointcloud[mask]
		return pointcloud

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

            start_time = rospy.Time.now().to_sec()
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
            #pc = pc2.read_points(msg_lidar, skip_nans=True)
            pc = ros_numpy.numpify(msg_lidar)
            points=np.zeros((pc.shape[0],4))
            points[:,0]=pc['x']
            points[:,1]=pc['y']
            points[:,2]=pc['z']
            points[:,3]=1.0
            rotated_pc = []
            p = self.crop_pointcloud(points)
            rotated_pc = self.rotation_matrix1.dot((np.vstack(([p[:,0]+1.529],[p[:,1]],[p[:,2]+1.311]))))
            #rotated_pc = np.dot(self.rotation_matrix1,(np.vstack(([p[:,0]+1.529],[p[:,1]],[p[:,2]+1.311]))))
            newlines = []
            self.transformed_pc = (np.array([rotated_pc[0] + self.x,rotated_pc[1] +self.y,rotated_pc[2] +self.z])) #transform to global position
            #print(self.transformed_pc)
            self.pc_list.extend(np.vstack([self.transformed_pc[0], self.transformed_pc[1], self.transformed_pc[2], p[:,3]]).T)#self.pc_list.append([self.transformed_pc[0,:], self.transformed_pc[1,:], self.transformed_pc[2,:], p[:,3]])
            #print(self.pc_list)
            #self.test = ' '.join((self.test,str(self.transformed_pc[:][0].T), str(self.transformed_pc[:][1].T), str(self.transformed_pc[:][2].T), str(p[:,3].T), '\n'))
            #self.test = ' '.join((self.test,str(self.transformed_pc[:].T),str(p[:,3]), '\n'))
            #self.test =  ''.join((self.test,(str((np.vstack([self.transformed_pc[0], self.transformed_pc[1], self.transformed_pc[2], p[:,3], newlines]).T)))))
            #newlines = ['\n'] * self.transformed_pc.shape[1]
            my_array = (np.array([self.transformed_pc[0], self.transformed_pc[1], self.transformed_pc[2], p[:,3]]).T)
            #self.test_arr =  [format(num, '.10f') for num in my_array[:]]# [str.format(('{:.10f}', num) for num in my_array)]#
            #self.test_arr  = ''.join(((str((np.array([self.transformed_pc[0], self.transformed_pc[1], self.transformed_pc[2], p[:,3]]).T.flatten())))))
            self.test = "\n".join(" ".join(str(x) for x in row) for row in my_array)#''.join((str(self.test),str(self.test_arr)))
            #print(self.test)#self.test = str(self.test.split())
            self.points_counter += self.transformed_pc.shape[1]

            output_rviz_pc = False
            if output_rviz_pc == True:
                final = (np.asarray(self.pc_list)[:][:])
                self.header.stamp = rospy.Time.now()
                self.scaled_polygon_pcl = self.create_cloud_xyzi32(self.header, final)
                self.pcl_pub.publish(self.scaled_polygon_pcl)

            end_time = rospy.Time.now().to_sec()
            print("Loop execution time: ", end_time-start_time)
           

if __name__ == '__main__':

    try:
	 
        GPSMapper()

    except rospy.ROSInterruptException:
        pass
