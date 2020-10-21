#!/usr/bin/env python

import numpy as np
from numpy import *

import os


import rospy


from ars_msf_slam_ros import *




def main():

  ars_msf_slam_ros = ArsMsfSlamRos()

  ars_msf_slam_ros.init()
  ars_msf_slam_ros.open()

  try:
    ars_msf_slam_ros.run()
  except rospy.ROSInterruptException:
    pass


  return 0



''' MAIN '''
if __name__ == '__main__':

  main()