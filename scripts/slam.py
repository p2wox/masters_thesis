#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import pcl
import pcl.pcl_visualization

VERBOSE = False

class localizer:

  def __init__():
  
   
class mapper:
  
  def __init__():
    self.M = pcl.PointCloud()
    self.T = 0

  def get_map():
    return self.M

  def update_map(data):
    if VERBOSE:
      print('received poincloud data from the depth camera')

    points_list = []
  
    for data in pc2.read_points(data, skip_nans=True):
      points_list.append([data[0], data[1], data[2]])

    pcl_data = pcl.PointCloud()
    pcl_data.from_list(points_list)
  
    #if Map does not
    if self.M.empty():
      self.M = pcl_data
    else:
      
      
  def get_location():
    return self.T

  #Update the current location self.T using nav_msgs/Odometry.msg message
  def update_location(data):
 
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
