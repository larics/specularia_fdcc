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


class trigger():
    def __init__(self):
        
        self.dig_outputs_srv = rospy.ServiceProxy('/kuka_hardware_interface/write_8_digital_outputs', write_8_outputs)
        rospy.Subscriber("/fdcc/state", fdcc_state, self.FDCC_stateCallback)
        self.tmpStringPosePub = rospy.Publisher('trigger_temp_pose', String, queue_size = 1)

        self.period = 0
        self.FDCCState = fdcc_state()


    def FDCC_stateCallback(self, msg):
        self.FDCCState = msg
       

    def run(self):
        
        while not rospy.is_shutdown():

            if (self.period == 0):
                self.dig_outputs_srv(True, True, True, False, False, False, False, False)
                self.period = 1
                print "ON"
                tmpPoseMsg = String()
                if (len(self.FDCCState.cartesian_position) != 0):
                    #print self.FDCCState.cartesian_position
                    #print self.FDCCState.cartesian_position[0]
                    tmpPoseMsg.data = str(self.FDCCState.cartesian_position[0]) + " " + str(self.FDCCState.cartesian_position[1]) + " " + str(self.FDCCState.cartesian_position[2]) + " " + str(self.FDCCState.cartesian_position[3]) + " " + str(self.FDCCState.cartesian_position[4]) + " " + str(self.FDCCState.cartesian_position[5]) + " " + str(self.FDCCState.cartesian_position[6])
                    self.tmpStringPosePub.publish(tmpPoseMsg)
                    #print pose_string
            else:
                self.dig_outputs_srv(True, True, False, False, False, False, False, False)
                self.period = 0
                print "OFF"

            #print "Running!"
            rospy.sleep(0.1)



if __name__ == '__main__':

    rospy.init_node("TriggerExample")
    node = trigger()

    node.run()
