#!/usr/bin/env python

import rospy
import roslib
import sys
import cv2
import numpy as np

from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates #Just for simulation with Gazebo
from geometry_msgs.msg import Vector3, PoseWithCovarianceStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient

from tf import transformations

from perception.msg import perception_data


class Tracking:

    def __init__(self):
        self.bridge = CvBridge()
        self.depth_subs = rospy.Subscriber("/camera/depth/image_raw",Image,self.depthCallback)
        self.processed_image_subs = rospy.Subscriber("/perception_data",perception_data,self.perceptionDataCallback)
        self.gazebo_model_position_subs = rospy.Subscriber("/gazebo/model_states",ModelStates,self.gazeboPositionCallback)
        self.position_subs = rospy.Subscriber("/amcl_pose",PoseWithCovarianceStamped,self.positionCallback)
        
        self.client = SimpleActionClient('move_base', MoveBaseAction)
        
        self.goal = MoveBaseGoal()
        
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
        
        self.ego_position = None
        self.goal_position = None
        self.ego_yaw = None
        
        self.leader_position = None
        self.leader_position_gt = None

    def depthCallback(self,data):
        """
        Depth topic callback to transform depth to matrix
        :param self: Instance reference
        :param data: Get depth image data
        """
        try:
            self.z = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError, e:
            print(e)
            pass

    def perceptionDataCallback(self,data):
        """
        Perception topic callback
        :param self: Instance reference
        :param data: Processed data of the image to get centroid, area, etc.
        """
        self.centroid = np.array([data.centroid_x,data.centroid_y])
        self.area = data.area
        
    
    def gazeboPositionCallback(self,data):
        """
        Position topic callback
        :param self: Instance reference
        :param data: Get the gazebo objects data to get the position
        """
        self.leader_position_gt = np.array([data.pose[2].position.x,data.pose[2].position.y,data.pose[2].position.z])
#        self.ego_position = np.array([data.pose[1].position.x,data.pose[1].position.y,data.pose[1].position.z])
    
    def positionCallback(self,data):
        """
        Position topic callback
        :param self: Instance reference
        :param data: Get the gazebo objects data to get the position
        """
        self.ego_position = np.array([data.pose.pose.position.x,
                                      data.pose.pose.position.y,
                                      data.pose.pose.position.z])
    
    
    def computeXYZ(self):
        """
        Compute the X and Y coordinates for each pixel 
        :param self: Instance reference
        """
        if self.z is not None:
            sz = self.z.shape
            u = np.ones((sz[0],sz[1])) * range(1,sz[1]+1)
            v = (np.ones((sz[1],sz[0])) * range(1,sz[0]+1)).T
            
            self.x = (u - self.k[0,2]) * self.z / self.k[0,0]
            self.y = (v - self.k[1,2]) * self.z / self.k[0,0]

    def computeLeaderPosition(self):
        """
        Compute the leader position in the world coordinate system
        :param self: Instance reference
        """
        if (self.centroid is not None and self.z is not None):
            v,u = self.centroid
            
            #Image reference systen to ego reference system
            self.leader_position = np.array([self.z[int(u),int(v)],-1*self.x[int(u),int(v)],self.y[int(u),int(v)]])
            #Ego reference system to World reference system
#            self.leader_position += self.ego_position
#            
#            print(self.leader_position)
#            print("error")
#            print(self.leader_position - self.leader_position_gt)
            

    def computeNewGoal(self):
        """
        Compute the next position using a security distance to the leader position
        :param self: Instance reference
        """
        if self.leader_position is not None:
            self.ego_yaw = np.arctan2(self.leader_position[1],self.leader_position[0])
            object_distance = np.sqrt((self.leader_position[0])**2+(self.leader_position[1])**2)
            
            if object_distance > self.security_distance: #security distance
                goal_distance = object_distance - self.security_distance
                x_pos = goal_distance * np.cos(self.ego_yaw) 
                y_pos = goal_distance * np.sin(self.ego_yaw)
                
                quaternion = transformations.quaternion_from_euler(0, 0, self.ego_yaw)
                
                self.goal.target_pose.header.frame_id = "odom"
                self.goal.target_pose.header.stamp = rospy.Time.now()
                self.goal.target_pose.pose.position.x = x_pos
                self.goal.target_pose.pose.position.y = y_pos
#                self.goal.target_pose.pose.position.z = 0.0
#                self.goal.target_pose.pose.orientation.x = quaternion[0]
#                self.goal.target_pose.pose.orientation.y = quaternion[1]
#                self.goal.target_pose.pose.orientation.z = quaternion[2]
#                self.goal.target_pose.pose.orientation.w = quaternion[3]
                self.goal.target_pose.pose.orientation.w = 1.0
                self.client.send_goal(self.goal)
                rospy.loginfo('sent goal')
                #rospy.loginfo(self.goal)
                self.client.wait_for_result()
                rospy.loginfo(self.client.get_result())
                
        
#            self.ego_yaw = np.arctan2(self.leader_position[1]-self.ego_position[1],self.leader_position[0]-self.ego_position[0])
#            
#            object_distance = np.sqrt((self.ego_position[0]-self.leader_position[0])**2+(self.ego_position[1]-self.leader_position[1])**2)
#            
#            if object_distance > self.security_distance: #security distance
#                
#                goal_distance = object_distance - self.security_distance 
#                
#                x_pos = goal_distance * np.cos(self.ego_yaw) 
#                y_pos = goal_distance * np.sin(self.ego_yaw)
#                
#                x_pos += self.ego_position[0]
#                y_pos += self.ego_position[1]
#                
#                print(x_pos,y_pos)
#                
#            
#            
#            print(self.ego_yaw*180.0/np.pi)
#            print("--------------")
        
    def loop(self,rate):
        while not rospy.is_shutdown():
            self.computeXYZ()
            self.computeLeaderPosition()
            self.computeNewGoal()
            rate.sleep()
    
def main():
    """
    Main function
    """
    rospy.init_node('tracking_node', anonymous=True)
    rate = rospy.Rate(1) # 2hz
    tracker = Tracking()
    
    try:
        tracker.client.wait_for_server()
        rospy.loginfo('got server')
        tracker.loop(rate)
        
    except KeyboardInterrupt:
        print("Shutting down")
        pass  
    except rospy.ROSInterruptException:
        print("Shutting down")
        pass  

if __name__ == '__main__':
    main()
    
    
    
