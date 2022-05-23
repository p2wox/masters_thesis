#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import pcl
import pcl.pcl_visualization
from tf.transformations import quaternion_matrix

VERBOSE = False

   
class mapper:
  
  #Initialize the map to an empty point cloud
  self.M = pcl.PointCloud()

  #Initialize location of the robot to zero 
  self.T = [[1,0,0,0],
            [0,1,0,0],
            [0,0,1,0],
            [0,0,0,1]]

  #Return the current map
  def get_map():
    return self.M

  #transform pointcloud data using self.T
  def transform_data(data):
    

  #Update map using data received from depth sensor and current location of the robot
  def update_map(data):

    #Convert data from PointCloud2 to pcl
    points_list = []
  
    for data in pc2.read_points(data, skip_nans=True):
      points_list.append([data[0], data[1], data[2]])

    pcl_data = pcl.PointCloud()
    pcl_data.from_list(points_list)
  
    #if Map does not exist yet, create map from the current data received fromt the depth sensor
    if self.M.empty():
      self.M = pcl_data
    else:
      #transform point cloud to correct location using current location of the robot
      pcl_data_transformed = self.transform_data(pcl_data)

      #Match point cloud to the map

      
      #Combine pointclouds
      self.M += pcl_data_transformed
      
  #Return the current location of the robot    
  def get_location():
    return self.T

  #Update the current location self.T using nav_msgs/Odometry.msg message
  def update_location(data):
    #add position part of the odometry data to transformation matrix
    self.T[0][3] = data.pose.pose.position.x
    self.T[1][3] = data.pose.pose.position.y
    self.T[2][3] = data.pose.pose.position.z

    #add orientation part of the odometry data to transformation matrix
    q = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
    R = quaternion_matrix(q)

    self.T[0][0] = R[0][0]
    self.T[0][1] = R[0][1]
    self.T[0][2] = R[0][2]
    self.T[1][0] = R[1][0]
    self.T[1][1] = R[1][1]
    self.T[1][2] = R[1][2]
    self.T[2][0] = R[2][0]
    self.T[2][1] = R[2][1]
    self.T[2][2] = R[2][2]
#listener listens to 'camera/depth/points' and runs callback function when data is received
def listener():

  rospy.init_node('pointcloud_listener', anonymous=True)
  rospy.init_node('IMU_listener', anonymous=True)
  
  Map = mapper()

  rospy.Subscriber('camera/depth/points', PointCloud2, Map.update_map)
  rospy.Subscriber('IMU', , Map.update_location)

  rospy.spin()

if __name__ == '__main__':
  listener()
