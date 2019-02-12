#!/usr/bin/env python

import rospy
import csv
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from fdcc.msg import *



class loadPath():
	def __init__(self):
		self.pathPub = rospy.Publisher('/path_pose', PoseStamped, queue_size=10)
		self.desiredXPub = rospy.Publisher('/pose_desired', Pose, queue_size=10)


		
		self.results = []
		self.poses = []

		self.FDCCState = fdcc_state()

		self.rotQuaternion = Quaternion()
		self.rotQuaternion.y = 1



	def loadFromFile(self):
		
		with open('/home/bmaric/kuka_ws/src/FDCC/scripts/DOORS_trajektorije.txt') as inputfile:
			for row in csv.reader(inputfile, delimiter=' '):
				self.results.append(row)

				tmp_pose = PoseStamped()
				tmp_x_desired = Pose()

				tmp_pose.header.stamp=rospy.Time.now()
				tmp_pose.header.frame_id='base'
				
				tmp_pose.pose.position.x = float(row[0])+0.1
				tmp_pose.pose.position.y = float(row[1])
				tmp_pose.pose.position.z = float(row[2]) + 0.3

				tmp_pose.pose.orientation.x = -float(row[5])  #-z
				tmp_pose.pose.orientation.y =  float(row[6])  # w
				tmp_pose.pose.orientation.z =  float(row[3])  # x
				tmp_pose.pose.orientation.w = -float(row[4])  #-y
				'''

				tmp_pose.pose.position.x = float(row[0])-0.8
				tmp_pose.pose.position.y = float(row[1])
				tmp_pose.pose.position.z = float(row[2])-0.15

				tmp_pose.pose.orientation.x = float(row[6])
				tmp_pose.pose.orientation.y = float(row[5])
				tmp_pose.pose.orientation.z = -float(row[4])
				tmp_pose.pose.orientation.w = -float(row[3])
				'''
				self.poses.append(tmp_pose)



	def publishMsg(self):
		
		for tmp_pose in self.poses:
			print tmp_pose
			tmp_pose.header.stamp = rospy.Time.now()
			self.pathPub.publish(tmp_pose)
			self.desiredXPub.publish(tmp_pose.pose)
			rospy.sleep(0.05)



if __name__ == '__main__':

	rospy.init_node('LoadPath')
	node = loadPath()

	node.loadFromFile()
	node.publishMsg()
