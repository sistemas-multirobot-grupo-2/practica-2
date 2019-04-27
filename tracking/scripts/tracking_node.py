#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates #Just for simulation with Gazebo
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from perception.msg import perception_data


class Tracking:

    def __init__(self):
        self.bridge = CvBridge()
        self.depth_subs = rospy.Subscriber("/robot2/camera/depth/image_raw",Image,self.depthCallback)
        self.processed_image_subs = rospy.Subscriber("/perception_data",perception_data,self.perceptionDataCallback)#("/robot2/processed_data",Image,self.callback)
        self.gazebo_model_position_subs = rospy.Subscriber("/gazebo/model_states",ModelStates,self.gazeboPositionCallback)

        self.k = np.array([[554.254691191187,              0.0, 320.5], 
                           [             0.0, 554.254691191187, 240.5],
                           [             0.0,              0.0,   1.0]])
        self.x = None
        self.y = None
        self.z = None
        self.bounding_box = None
        self.area = None
        self.centroid = None
        
        self.security_distance = 1
        
        self.ego_position = np.zeros(3)
        self.goal_position = np.zeros(4)
        self.ego_yaw = None
        
        self.leader_position = np.zeros(3)
        self.leader_position_gt = np.zeros(3)

    def depthCallback(self,data):
        try:
            self.z = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError, e:
            print(e)
            pass

    def perceptionDataCallback(self,data):
        self.centroid = np.array([data.centroid_x,data.centroid_y])
        self.area = data.area
        
    
    def gazeboPositionCallback(self,data):
        self.ego_position = np.array([data.pose[1].position.x,data.pose[1].position.y,data.pose[1].position.z])
        self.leader_position_gt = np.array([data.pose[2].position.x,data.pose[2].position.y,data.pose[2].position.z])
    
    def computeXYZ(self):
        if self.z is not None:
            sz = self.z.shape
            u = np.ones((sz[0],sz[1])) * range(1,sz[1]+1)
            v = (np.ones((sz[1],sz[0])) * range(1,sz[0]+1)).T
            
            self.x = (u - self.k[0,2]) * self.z / self.k[0,0]
            self.y = (v - self.k[1,2]) * self.z / self.k[0,0]

    def computeLeaderPosition(self):
        if (self.centroid is not None and self.z is not None):
            v,u = self.centroid
            
            #Image reference systen to ego reference system
            self.leader_position = np.array([self.z[int(u),int(v)],-1*self.x[int(u),int(v)],self.y[int(u),int(v)]])
            #Ego reference system to World reference system
            self.leader_position += self.ego_position
            
            print(self.leader_position)
            print("error")
            print(self.leader_position - self.leader_position_gt)
            

    def computeNewGoal(self):
        self.ego_yaw = np.arctan2(self.leader_position[1]-self.ego_position[1],self.leader_position[0]-self.ego_position[0])
        
        object_distance = np.sqrt((self.ego_position[0]-self.leader_position[0])**2+(self.ego_position[1]-self.leader_position[1])**2)
        
        if object_distance > self.security_distance: #security distance
            
            goal_distance = object_distance - self.security_distance 
            
            x_pos = goal_distance * np.cos(self.ego_yaw) 
            y_pos = goal_distance * np.sin(self.ego_yaw)
            
            x_pos += self.ego_position[0]
            y_pos += self.ego_position[1]
            
            print(x_pos,y_pos)
            
        
        
        print(self.ego_yaw*180.0/np.pi)
        print("--------------")
        

def main(args):
    tracker = Tracking()
    rospy.init_node('tracking_node', anonymous=True)
    rate = rospy.Rate(1) # 2hz
    try:
        while not rospy.is_shutdown():
            tracker.computeXYZ()
            tracker.computeLeaderPosition()
            tracker.computeNewGoal()
            
            rate.sleep()

    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
    
    
    
