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

Target = NULL
Map = NULL
Location = NULL 


def update_map(data):
  Map = data

def update_location(data):
  Location = data

  if target != NULL:
    plan = plan_path(target)
    if (plan != -1):
      move_spot(plan)


def move_spot(plan)
  for move in plan:
    #Send message to move spot according to the plan

def plan_path(target):

#listener listens to the map and location sent by slam and runs callback functions
def listener():

  rospy.init_node('map_listener', anonymous=True)
  rospy.init_node('location_listener', anonymous=True)
 

  rospy.Subscriber('map', PointCloud2, update_map)
  rospy.Subscriber('location', , update_location)

  rospy.spin()

if __name__ == '__main__':
  listener()
