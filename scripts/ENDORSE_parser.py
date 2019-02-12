#!/usr/bin/env python

import rospy
import csv
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion

from kuka_rsi_hw_interface.srv import *
from cogni_robot_planner.srv import *
from fdcc.msg import *
from std_msgs.msg import String



class ENDORSE_parser():
    def __init__(self):
        self.pathPub = rospy.Publisher('/path_pose', PoseStamped, queue_size=10)
        self.desiredXPub = rospy.Publisher('/pose_desired', Pose, queue_size=10)

        self.tmpStringPosePub = rospy.Publisher('trigger_temp_pose', String, queue_size = 1)
        
        # services
        self.dig_outputs_srv = rospy.ServiceProxy('/kuka_hardware_interface/write_8_digital_outputs', write_8_outputs)

        rospy.Subscriber("/fdcc/state", fdcc_state, self.FDCC_stateCallback)


        self.results = []
        self.poses = []

        self.rotQuaternion = Quaternion()
        self.rotQuaternion.y = 1

        self.waypoints = []
        self.stopPoints = []

        self.FDCCState = fdcc_state()
        self.triggerPoses = []

        #self.planner_srv = rospy.Service('/RoboCogniPlaner_service', PoseMsgSimpleSrv , self.cartesianPathParserSrvCallback)
        #rospy.spin()


    def FDCC_stateCallback(self, msg):
        self.FDCCState = msg

    def cartesianPathParserSrvCallback(self, request):

        lines = request.toolPose.splitlines()

        point_num = 0

        for onLine in lines:
            elements = onLine.split(" ")

            if len(elements) == 7:
                
                onePose = PoseStamped()
                
                onePose.pose.position.x = float(elements[0])
                onePose.pose.position.y = float(elements[1])
                onePose.pose.position.z = float(elements[2]) + 0.25
                # PROMIJENITI U FORMU X, Y, Z, W
                onePose.pose.orientation.w = float(elements[3])
                onePose.pose.orientation.x = float(elements[4])
                onePose.pose.orientation.y = float(elements[5])
                onePose.pose.orientation.z = float(elements[6])
                '''

                onePose.pose.position.x = float(elements[0])+0.1
                onePose.pose.position.y = float(elements[1])
                onePose.pose.position.z = float(elements[2]) + 0.3

                onePose.pose.orientation.x = -float(elements[5])  #-z
                onePose.pose.orientation.y =  float(elements[6])  # w
                onePose.pose.orientation.z =  float(elements[3])  # x
                onePose.pose.orientation.w = -float(elements[4])  #-y
                '''

                self.waypoints.append((onePose))
            elif len(elements) == 1:
                if (elements[0] == "SCAN"):
                    self.stopPoints.append(point_num)

            point_num = point_num + 1

        self.executeScanPath()
        return "OK"

    def executeScanPath(self):


        for i in range(len(self.waypoints)):
            #print self.waypoints[i]

            if i in self.stopPoints:
                # trigger laser

                # wait for robot to stop
                rospy.sleep(2)

                # trigger laser on
                #self.dig_outputs_srv(True, True, True, False, False, False, False, False)
                print "Trigger ON: " + str(i)
                

                #print self.FDCCState.cartesian_position[0]
                tmpPoseMsg = String()
                tmpPoseMsg.data = str(self.FDCCState.cartesian_position[0]) + " " + str(self.FDCCState.cartesian_position[1]) + " " + str(self.FDCCState.cartesian_position[2]) + " " + str(self.FDCCState.cartesian_position[3]) + " " + str(self.FDCCState.cartesian_position[4]) + " " + str(self.FDCCState.cartesian_position[5]) + " " + str(self.FDCCState.cartesian_position[6])
                self.tmpStringPosePub.publish(tmpPoseMsg)
                
                #rospy.sleep(0.5)
            else:
                # move robot

                self.waypoints[i].header.stamp = rospy.Time.now()
                self.waypoints[i].header.frame_id='base'
                #print "MOVE ROBOT"
                self.pathPub.publish(self.waypoints[i])
                self.desiredXPub.publish(self.waypoints[i].pose)
                rospy.sleep(0.2)

                # turn trigger off
                #self.dig_outputs_srv(True, True, False, False, False, False, False, False)
                #print "Trigger OFF"
        
        print self.triggerPoses        

    def loadFromFile(self):
        

         #read tool path from file
        file = open('/home/bmaric/kuka_ws/src/FDCC/scripts/trajektorijeNADA2.txt', "r")
        request = PoseMsgSimpleSrvRequest();
        request.toolPose=file.read()

        print request.toolPose
        self.cartesianPathParserSrvCallback(request)
        

    def publishMsg(self):
        
        for tmp_pose in self.poses:
            print tmp_pose
            tmp_pose.header.stamp = rospy.Time.now()
            self.pathPub.publish(tmp_pose)
            self.desiredXPub.publish(tmp_pose.pose)
            rospy.sleep(0.05)



if __name__ == '__main__':

    rospy.init_node('LoadPath')
    node = ENDORSE_parser()

    node.loadFromFile()
    #node.executeScanPath()
