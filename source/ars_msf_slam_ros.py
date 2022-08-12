#!/usr/bin/env python

import numpy as np
from numpy import *

import os




# ROS

import rospy

import rospkg

import std_msgs.msg
from std_msgs.msg import Header

import geometry_msgs.msg
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import QuaternionStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import TwistWithCovarianceStamped

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray



import tf2_ros




#
from ars_msf_slam import *


#
import ars_lib_helpers





class ArsMsfSlamRos:

  #######

  # Robot frame
  robot_frame = None
  # World frame
  world_frame = None


  # State Estim loop freq 
  # time step
  state_estim_loop_freq = None
  # Timer
  state_estim_loop_timer = None


  # Meas Robot atti subscriber
  meas_robot_atti_sub = None
  # Meas Robot velocity subscriber
  meas_robot_vel_robot_sub = None
  # Obstacles detected
  meas_obst_detected_robot_sub = None


  # Estim Robot pose pub
  estim_robot_pose_pub = None
  estim_robot_pose_cov_pub = None
  # Estim Robot velocity pub
  estim_robot_vel_robot_pub = None
  estim_robot_vel_robot_cov_pub = None
  #
  estim_robot_vel_world_pub = None
  estim_robot_vel_world_cov_pub = None
  # Estim Map wrt world
  estim_map_world_pub = None


  # MSF SLAM
  msf_slam = None
  


  #########

  def __init__(self):

    # Robot frame
    self.robot_frame = 'robot_estim_base_link'
    # World frame
    self.world_frame = 'world'

    # State Estim loop freq 
    # time step
    self.state_estim_loop_freq = 50.0

    # SLAM component
    self.msf_slam = ArsMsfSlam()
    self.msf_slam.world_frame = self.world_frame


    # end
    return


  def init(self, node_name='ars_msf_slam_node'):
    #

    # Init ROS
    rospy.init_node(node_name, anonymous=True)

    
    # Package path
    pkg_path = rospkg.RosPack().get_path('ars_msf_slam')
    

    #### READING PARAMETERS ###
    
    # 

    ###

    
    # End
    return


  def open(self):

    # Subscribers

    # 
    self.meas_robot_atti_sub = rospy.Subscriber('meas_robot_attitude', QuaternionStamped, self.measRobotAttitudeCallback)
    #
    self.meas_robot_vel_robot_sub = rospy.Subscriber('meas_robot_velocity_robot', TwistStamped, self.measRobotVelRobotCallback)
    # Obstacles detected
    self.meas_obst_detected_robot_sub = rospy.Subscriber('obstacles_detected_robot', MarkerArray, self.measObstaclesDetectedRobotCallback)

    

    # Publishers

    # Pose robot wrt world
    # 
    self.estim_robot_pose_pub = rospy.Publisher('estim_robot_pose', PoseStamped, queue_size=1)
    # 
    self.estim_robot_pose_cov_pub = rospy.Publisher('estim_robot_pose_cov', PoseWithCovarianceStamped, queue_size=1)
    
    # Velocity robot wrt robot
    #
    self.estim_robot_vel_robot_pub = rospy.Publisher('estim_robot_velocity_robot', TwistStamped, queue_size=1)
    #
    self.estim_robot_vel_robot_cov_pub = rospy.Publisher('estim_robot_velocity_robot_cov', TwistWithCovarianceStamped, queue_size=1)
    
    # Velocity robot wrt world
    #
    self.estim_robot_vel_world_pub = rospy.Publisher('estim_robot_velocity_world', TwistStamped, queue_size=1)
    #
    self.estim_robot_vel_world_cov_pub = rospy.Publisher('estim_robot_velocity_world_cov', TwistWithCovarianceStamped, queue_size=1)
    
    # Estim Map wrt world
    #
    self.estim_map_world_pub = rospy.Publisher('estim_map_world', MarkerArray, queue_size=1)


    # Tf2 broadcasters
    self.tf2_broadcaster = tf2_ros.TransformBroadcaster()


    # Timers
    #
    self.state_estim_loop_timer = rospy.Timer(rospy.Duration(1.0/self.state_estim_loop_freq), self.stateEstimLoopTimerCallback)


    # End
    return


  def run(self):

    rospy.spin()

    return


  def measRobotAttitudeCallback(self, robot_attitude_msg):

    # Timestamp
    timestamp = robot_attitude_msg.header.stamp

    # Attitude quat simp
    robot_atti_quat = ars_lib_helpers.Quaternion.zerosQuat()
    robot_atti_quat[0] = robot_attitude_msg.quaternion.w
    robot_atti_quat[1] = robot_attitude_msg.quaternion.x
    robot_atti_quat[2] = robot_attitude_msg.quaternion.y
    robot_atti_quat[3] = robot_attitude_msg.quaternion.z

    robot_atti_quat_simp = ars_lib_helpers.Quaternion.getSimplifiedQuatRobotAtti(robot_atti_quat)

    #
    self.msf_slam.setMeasRobotAttitude(timestamp, robot_atti_quat_simp)

    # Predict
    #self.msf_slam.predict(timestamp)

    # Update
    #self.msf_slam.update()

    #
    return


  def measRobotVelRobotCallback(self, robot_vel_msg):

    # Timestamp
    timestamp = robot_vel_msg.header.stamp

    # Linear
    lin_vel_robot = np.zeros((3,), dtype=float)
    lin_vel_robot[0] = robot_vel_msg.twist.linear.x
    lin_vel_robot[1] = robot_vel_msg.twist.linear.y
    lin_vel_robot[2] = robot_vel_msg.twist.linear.z

    # Angular
    ang_vel_robot = np.zeros((1,), dtype=float)
    ang_vel_robot[0] = robot_vel_msg.twist.angular.z

    #
    self.msf_slam.setMeasRobotVelRobot(timestamp, lin_vel_robot, ang_vel_robot)

    # Predict
    #self.msf_slam.predict(timestamp)

    # Update
    #self.msf_slam.update()

    #
    return


  def measObstaclesDetectedRobotCallback(self, meas_obstacles_detected_robot_msg):

    timestamp = rospy.Time()
    obstacles_detected = []

    for obstacle_detected_msg in meas_obstacles_detected_robot_msg.markers:

        if(obstacle_detected_msg.action == 0):

            if(obstacle_detected_msg.type == 3):

                circle3d = ars_lib_helpers.Circle3D()

                #
                timestamp = obstacle_detected_msg.header.stamp

                # ID
                circle3d.id_lab = obstacle_detected_msg.id

                #
                circle3d.position[0] = obstacle_detected_msg.pose.position.x
                circle3d.position[1] = obstacle_detected_msg.pose.position.y
                circle3d.position[2] = obstacle_detected_msg.pose.position.z

                #
                attitude_quat = ars_lib_helpers.Quaternion.zerosQuat()
                attitude_quat[0] = obstacle_detected_msg.pose.orientation.w
                attitude_quat[1] = obstacle_detected_msg.pose.orientation.x
                attitude_quat[2] = obstacle_detected_msg.pose.orientation.y
                attitude_quat[3] = obstacle_detected_msg.pose.orientation.z
                circle3d.attitude_quat_simp = ars_lib_helpers.Quaternion.getSimplifiedQuatRobotAtti(attitude_quat)

                #
                circle3d.parent_frame = obstacle_detected_msg.header.frame_id

                #
                circle3d.circle_radius = 0.5*obstacle_detected_msg.scale.x


                #
                obstacles_detected.append(circle3d)


    # Set
    self.msf_slam.setMeasCirclesDetected(timestamp, obstacles_detected)

    # End
    return


  def estimRobotPosePublish(self):

    #
    header_msg = Header()
    header_msg.stamp = self.msf_slam.estim_state_timestamp
    header_msg.frame_id = self.world_frame

    #
    robot_pose_msg = Pose()
    #
    robot_pose_msg.position.x = self.msf_slam.estim_robot_posi[0]
    robot_pose_msg.position.y = self.msf_slam.estim_robot_posi[1]
    robot_pose_msg.position.z = self.msf_slam.estim_robot_posi[2]
    #
    robot_pose_msg.orientation.w = self.msf_slam.estim_robot_atti_quat_simp[0]
    robot_pose_msg.orientation.x = 0.0
    robot_pose_msg.orientation.y = 0.0
    robot_pose_msg.orientation.z = self.msf_slam.estim_robot_atti_quat_simp[1]

    #
    # Covariance
    covariance_pose = np.zeros((6,6), dtype=float)
    # Position - Position
    covariance_pose[0:3, 0:3] = self.msf_slam.estim_state_cov[0:3, 0:3]
    # Position - Attitude
    covariance_pose[0:3, 5] = self.msf_slam.estim_state_cov[0:3, 3]
    # Attitude - Attitude
    covariance_pose[5, 5] = self.msf_slam.estim_state_cov[3, 3]
    # Attitude - Position
    covariance_pose[5, 0:3] = self.msf_slam.estim_state_cov[3, 0:3]


    #
    robot_pose_stamped_msg = PoseStamped()
    #
    robot_pose_stamped_msg.header = header_msg
    robot_pose_stamped_msg.pose = robot_pose_msg

    #
    robot_pose_cov_stamped_msg = PoseWithCovarianceStamped()
    #
    robot_pose_cov_stamped_msg.header = header_msg
    robot_pose_cov_stamped_msg.pose.pose = robot_pose_msg
    robot_pose_cov_stamped_msg.pose.covariance = covariance_pose.reshape((36,))

  
    #
    self.estim_robot_pose_pub.publish(robot_pose_stamped_msg)
    # 
    self.estim_robot_pose_cov_pub.publish(robot_pose_cov_stamped_msg)


    # Tf2
    estim_robot_pose_tf2_msg = geometry_msgs.msg.TransformStamped()

    estim_robot_pose_tf2_msg.header.stamp = self.msf_slam.estim_state_timestamp
    estim_robot_pose_tf2_msg.header.frame_id = self.world_frame
    estim_robot_pose_tf2_msg.child_frame_id = self.robot_frame

    estim_robot_pose_tf2_msg.transform.translation.x = self.msf_slam.estim_robot_posi[0]
    estim_robot_pose_tf2_msg.transform.translation.y = self.msf_slam.estim_robot_posi[1]
    estim_robot_pose_tf2_msg.transform.translation.z = self.msf_slam.estim_robot_posi[2]

    estim_robot_pose_tf2_msg.transform.rotation.w = self.msf_slam.estim_robot_atti_quat_simp[0]
    estim_robot_pose_tf2_msg.transform.rotation.x = 0.0
    estim_robot_pose_tf2_msg.transform.rotation.y = 0.0
    estim_robot_pose_tf2_msg.transform.rotation.z = self.msf_slam.estim_robot_atti_quat_simp[1]

    # Broadcast
    self.tf2_broadcaster.sendTransform(estim_robot_pose_tf2_msg)


    # End
    return


  def estimRobotVelocityPublish(self):

    #
    # Robot Velocity Wrt world

    # Header
    header_wrt_world_msg = Header()
    header_wrt_world_msg.stamp = self.msf_slam.estim_state_timestamp
    header_wrt_world_msg.frame_id = self.world_frame

    # Twist
    robot_velocity_world_msg = Twist()
    #
    robot_velocity_world_msg.linear.x = self.msf_slam.estim_robot_velo_lin_world[0]
    robot_velocity_world_msg.linear.y = self.msf_slam.estim_robot_velo_lin_world[1]
    robot_velocity_world_msg.linear.z = self.msf_slam.estim_robot_velo_lin_world[2]
    #
    robot_velocity_world_msg.angular.x = 0.0
    robot_velocity_world_msg.angular.y = 0.0
    robot_velocity_world_msg.angular.z = self.msf_slam.estim_robot_velo_ang_world[0]
    
    # TwistStamped
    robot_velocity_world_stamp_msg = TwistStamped()
    robot_velocity_world_stamp_msg.header = header_wrt_world_msg
    robot_velocity_world_stamp_msg.twist = robot_velocity_world_msg

    # TwistWithCovarianceStamped
    # TODO JL Cov
    robot_velocity_world_cov_stamp_msg = TwistWithCovarianceStamped()
    robot_velocity_world_cov_stamp_msg.header = header_wrt_world_msg
    robot_velocity_world_cov_stamp_msg.twist.twist = robot_velocity_world_msg
    # robot_velocity_world_cov_stamp_msg.twist.covariance


    #
    # Robot velocity wrt robot

    # computation estim robot velocity robot
    estim_robot_vel_lin_robot = ars_lib_helpers.Conversions.convertVelLinFromWorldToRobot(self.msf_slam.estim_robot_velo_lin_world, self.msf_slam.estim_robot_atti_quat_simp, flag_quat_simp=True)
    estim_robot_vel_ang_robot = ars_lib_helpers.Conversions.convertVelAngFromWorldToRobot(self.msf_slam.estim_robot_velo_ang_world, self.msf_slam.estim_robot_atti_quat_simp, flag_quat_simp=True)

    # Header
    header_wrt_robot_msg = Header()
    header_wrt_robot_msg.stamp = self.msf_slam.estim_state_timestamp
    header_wrt_robot_msg.frame_id = self.robot_frame

    # Twist
    robot_velocity_robot_msg = Twist()
    #
    robot_velocity_robot_msg.linear.x = estim_robot_vel_lin_robot[0]
    robot_velocity_robot_msg.linear.y = estim_robot_vel_lin_robot[1]
    robot_velocity_robot_msg.linear.z = estim_robot_vel_lin_robot[2]
    #
    robot_velocity_robot_msg.angular.x = 0.0
    robot_velocity_robot_msg.angular.y = 0.0
    robot_velocity_robot_msg.angular.z = estim_robot_vel_ang_robot[0]

    # TwistStamped
    robot_velocity_robot_stamp_msg = TwistStamped()
    robot_velocity_robot_stamp_msg.header = header_wrt_robot_msg
    robot_velocity_robot_stamp_msg.twist = robot_velocity_robot_msg

    # TwistWithCovarianceStamped
    # TODO JL Cov
    robot_velocity_robot_cov_stamp_msg = TwistWithCovarianceStamped()
    robot_velocity_robot_cov_stamp_msg.header = header_wrt_robot_msg
    robot_velocity_robot_cov_stamp_msg.twist.twist = robot_velocity_robot_msg
    # robot_velocity_robot_cov_stamp_msg.twist.covariance


    # Publish
    #
    self.estim_robot_vel_world_pub.publish(robot_velocity_world_stamp_msg)
    # 
    self.estim_robot_vel_world_cov_pub.publish(robot_velocity_world_cov_stamp_msg)

    #
    self.estim_robot_vel_robot_pub.publish(robot_velocity_robot_stamp_msg)
    # 
    self.estim_robot_vel_robot_cov_pub.publish(robot_velocity_robot_cov_stamp_msg)

    # End
    return
  

  def estimMapWorldPublish(self):

    estim_map_world_msg = MarkerArray()
    estim_map_world_msg.markers = []

    for map_element in self.msf_slam.estim_map_world:
  
        map_element_msg = Marker()


        map_element_msg.header = Header()
        map_element_msg.header.stamp = self.msf_slam.estim_state_timestamp
        map_element_msg.header.frame_id = map_element.parent_frame

        map_element_msg.ns = 'estimated_map'

        map_element_msg.id = map_element.id_lab

        map_element_msg.type = 3

        map_element_msg.action = 0

        map_element_msg.pose.position.x = map_element.position[0]
        map_element_msg.pose.position.y = map_element.position[1]
        map_element_msg.pose.position.z = map_element.position[2]

        map_element_msg.pose.orientation.w = map_element.attitude_quat_simp[0]
        map_element_msg.pose.orientation.x = 0.0
        map_element_msg.pose.orientation.y = 0.0
        map_element_msg.pose.orientation.z = map_element.attitude_quat_simp[1]

        map_element_msg.scale.x = 2.0*map_element.circle_radius
        map_element_msg.scale.y = 2.0*map_element.circle_radius
        map_element_msg.scale.z = 1.0

        map_element_msg.color.r = 0.75
        map_element_msg.color.g = 0.0
        map_element_msg.color.b = 0.75
        map_element_msg.color.a = 0.6

        map_element_msg.lifetime = rospy.Duration(2.0*1.0/self.state_estim_loop_freq)


        estim_map_world_msg.markers.append(map_element_msg)


    self.estim_map_world_pub.publish(estim_map_world_msg)

    return


  def stateEstimLoopTimerCallback(self, timer_msg):

    # Get time
    time_stamp_current = rospy.Time.now()

    # Predict
    self.msf_slam.predict(time_stamp_current)

    # Update
    self.msf_slam.update()


    # Publish
    #
    self.estimRobotPosePublish()
    #
    self.estimRobotVelocityPublish()
    #
    self.estimMapWorldPublish()

     
    # End
    return

  