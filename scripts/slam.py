#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import pcl
import pcl.pcl_visualization

VERBOSE = False

def callback(data):
  if VERBOSE:
    print('received image of type: "%s"' % data.format)

  points_list = []
  
  for data in pc2.read_points(data, skip_nans=True):
    points_list.append([data[0], data[1], data[2]])

  pcl_data = pcl.PointCloud()
  pcl_data.from_list(points_list)

  visual = pcl.pcl_visualization.CloudViewing()

  visual.ShowMonochromeCloud(pcl_data, b'cloud')

  v = True
  while v:
    v=not(visual.WasStopped())
  


def listener():

  rospy.init_node('pointcloud_listener', anonymous=True)
  
  rospy.Subscriber('camera/depth/points', PointCloud2, callback)

  rospy.spin()

if __name__ == '__main__':
  listener()
