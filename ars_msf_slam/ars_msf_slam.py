#!/usr/bin/env python3

import numpy as np
from numpy import *

import threading

import scipy
from scipy.linalg import  *


# ROS
import rclpy
from rclpy.time import Time

import nav_msgs.msg
from nav_msgs.msg import Path


#
import ars_lib_helpers.ars_lib_helpers as ars_lib_helpers




class ArsMsfSlam:

  #######

  #
  world_frame = None

  # Meas position
  # z_t = [m_posi_x, m_posi_y, m_posi_z]
  # Dim (z_t) = 3
  flag_set_meas_robot_posi = None
  meas_robot_posi_timestamp = None
  meas_robot_posi = None
  # Meas attitude
  # z_a = [m_atti_yaw]
  # Dim (z_a) = 1
  flag_set_meas_robot_atti = None
  meas_robot_atti_timestamp = None
  meas_robot_atti_quat_simp = None
  # Meas velocity
  # z_v = [m_vel_lin_x_robot, m_vel_lin_y_robot, m_vel_lin_z_robot,
  #       m_vel_ang_z_robot]
  # Dim (z_v) = 4
  flag_set_meas_robot_vel_robot = None
  meas_robot_velo_timestamp = None
  meas_robot_velo_lin_robot = None
  meas_robot_velo_ang_robot = None
  # Meas obstacles detected
  flag_set_meas_circles_detected = None
  meas_circles_detected_timestamp = None
  meas_circles_detected = None

  #
  lock_meas = None


  # Estimated State

  #
  estim_state_timestamp = None

  # Robot
  # x_robot = [ posi_x, posi_y, posi_z, 
  #       atti_yaw, 
  #       vel_lin_x_world, vel_lin_y_world, vel_lin_z_world,
  #       vel_ang_z_world ]
  # Dim(x_robot) = 8
  # Estimated Pose
  estim_robot_posi = None
  estim_robot_atti_quat_simp = None
  # Estimated Velocity
  estim_robot_velo_lin_world = None
  estim_robot_velo_ang_world = None

  # Map elements (i.e. landmarks)
  # x_map_element_i = [posi_x, posi_y, posi_z,
  #                      radius]
  # Dim(x_map_element_i) = 4
  # Estimated map
  estim_map_world = None

  # Cov estimated state total
  estim_state_cov = None


  #
  lock_state = None


  # Covariance of the process model
  cov_proc_mod = None


  # Covariance meas attitude
  cov_meas_atti = None

  # Covariance meas velocity
  cov_meas_velo_lin = None
  cov_meas_velo_ang = None

  # Covariance meas circle detected
  cov_meas_circle_detected_robot = None



  #########

  def __init__(self):

    #
    self.world_frame = 'world'

    # Meas Position
    self.flag_set_meas_robot_posi = False
    self.meas_robot_posi_timestamp = Time()
    self.meas_robot_posi = np.zeros((3,), dtype=float)
    # Meas Attitude
    self.flag_set_meas_robot_atti = False
    self.meas_robot_atti_timestamp = Time()
    self.meas_robot_atti_quat_simp = ars_lib_helpers.Quaternion.zerosQuatSimp()
    # Meas Velocity
    self.flag_set_meas_robot_vel_robot = False
    self.meas_robot_velo_timestamp = Time()
    self.meas_robot_velo_lin_robot = np.zeros((3,), dtype=float)
    self.meas_robot_velo_ang_robot = np.zeros((1,), dtype=float)
    # Meas obstacles detected
    self.flag_set_meas_circles_detected = False
    self.meas_circles_detected_timestamp = Time()
    # A of circle3D
    self.meas_circles_detected = []

    #
    self.lock_meas = threading.Lock()

    # Estimated State
    self.estim_state_timestamp = Time()
    # Estmated Pose
    self.estim_robot_posi = np.zeros((3,), dtype=float)
    self.estim_robot_atti_quat_simp = ars_lib_helpers.Quaternion.zerosQuatSimp()
    # Estimated Velocity
    self.estim_robot_velo_lin_world = np.zeros((3,), dtype=float)
    self.estim_robot_velo_ang_world = np.zeros((1,), dtype=float)
    # Estimated map - As a list of circle3D
    self.estim_map_world = []
    
    # Cov estimated state
    # Only robot from the beginning
    self.estim_state_cov = np.zeros((8,8), dtype=float)

    #
    self.lock_state = threading.Lock()


    # Covariance of the process model
    self.cov_proc_mod = np.zeros((4,4), dtype=float)

    # Covariance meas position
    self.cov_meas_posi = np.zeros((3,3), dtype=float)

    # Covariance meas attitude
    self.cov_meas_atti = np.zeros((1,1), dtype=float)

    # Covariance meas velocity
    self.cov_meas_velo_lin = np.zeros((3,3), dtype=float)
    self.cov_meas_velo_ang = np.zeros((1,1), dtype=float)

    # Covariance meas circle detected
    self.cov_meas_circle_detected_robot = np.zeros((4,4), dtype=float)


    # End
    return


  def setConfigParameters(self, config_param):

    # Estmated Pose
    self.estim_robot_posi = np.array(config_param['estimated_state_init']['state']['robot_position'])
    self.estim_robot_atti_quat_simp = ars_lib_helpers.Quaternion.setQuatSimp(config_param['estimated_state_init']['state']['robot_atti_quat_simp'])
    # Estimated Velocity
    self.estim_robot_velo_lin_world = np.array(config_param['estimated_state_init']['state']['robot_vel_lin_world'])
    self.estim_robot_velo_ang_world = np.array(config_param['estimated_state_init']['state']['robot_vel_ang_world'])

    # Cov estimated state
    self.estim_state_cov = np.diag(config_param['estimated_state_init']['cov_diag'])

    # Covariance of the process model
    self.cov_proc_mod = np.diag(config_param['process_model']['cov_diag'])

    # Covariance meas position
    self.cov_meas_posi = np.diag(config_param['measurements']['meas_position']['cov_diag'])

    # Covariance meas attitude
    self.cov_meas_atti = np.diag(config_param['measurements']['meas_attitude']['cov_diag'])

    # Covariance meas velocity
    self.cov_meas_velo_lin = np.diag(config_param['measurements']['meas_velo_lin']['cov_diag'])
    self.cov_meas_velo_ang = np.diag(config_param['measurements']['meas_velo_ang']['cov_diag'])

    # Covariance meas circle detected
    self.cov_meas_circle_detected_robot = np.diag(config_param['measurements']['meas_circles_detector']['cov_diag'])


    return


  def setMeasRobotPosition(self, timestamp, robot_posi):

    self.lock_meas.acquire()

    self.flag_set_meas_robot_posi = True

    self.meas_robot_posi_timestamp = timestamp

    self.meas_robot_posi = robot_posi

    self.lock_meas.release()

    return

  def setMeasRobotAttitude(self, timestamp, robot_atti_quat_simp):

    self.lock_meas.acquire()

    self.flag_set_meas_robot_atti = True

    self.meas_robot_atti_timestamp = timestamp

    self.meas_robot_atti_quat_simp = robot_atti_quat_simp

    self.lock_meas.release()

    return

  def setMeasRobotVelRobot(self, timestamp, lin_vel_world, ang_vel_world):

    self.lock_meas.acquire()

    self.flag_set_meas_robot_vel_robot = True

    self.meas_robot_velo_timestamp = timestamp

    self.meas_robot_velo_lin_robot = lin_vel_world
    self.meas_robot_velo_ang_robot = ang_vel_world

    self.lock_meas.release()

    return


  def setMeasCirclesDetected(self, timestamp, meas_circles_detected):

    self.lock_meas.acquire()

    self.flag_set_meas_circles_detected = True

    self.meas_circles_detected_timestamp = timestamp

    self.meas_circles_detected = meas_circles_detected

    self.lock_meas.release()

    return


  
  def predict(self, timestamp):

    #
    self.lock_state.acquire()


    # Delta time
    delta_time = 0.0
    estim_state_timestamp = self.estim_state_timestamp.to_msg()
    if(estim_state_timestamp.sec == 0 and estim_state_timestamp.nanosec == 0):
      delta_time = 0.0
    else:
      delta_time = (timestamp - self.estim_state_timestamp).nanoseconds/1e9

    #
    num_mapped_landmarks = len(self.estim_map_world)

    # State
    # Robot
    estim_x_kk_robot_posi = self.estim_robot_posi
    estim_x_kk_robot_atti_quat_simp = self.estim_robot_atti_quat_simp
    estim_x_kk_robot_velo_lin_world = self.estim_robot_velo_lin_world
    estim_x_kk_robot_velo_ang_world = self.estim_robot_velo_ang_world
    # Landmarks
    # Do nothing
    # self.estim_map_world

    # Cov
    estim_P_kk = self.estim_state_cov


    # Process model

    # Robot

    # Position
    estim_x_k1k_robot_posi = estim_x_kk_robot_posi + delta_time * estim_x_kk_robot_velo_lin_world

    # Attitude
    delta_robot_atti_ang = delta_time * estim_x_kk_robot_velo_ang_world
    delta_robot_atti_quat_sim = ars_lib_helpers.Quaternion.quatSimpFromAngle(delta_robot_atti_ang)
    estim_x_k1k_robot_atti_quat_simp = ars_lib_helpers.Quaternion.quatSimpProd(estim_x_kk_robot_atti_quat_simp, delta_robot_atti_quat_sim)

    # Velocity Linear
    # Constant
    estim_x_k1k_robot_velo_lin_world = estim_x_kk_robot_velo_lin_world

    # Velocity Angular
    # Constant
    estim_x_k1k_robot_velo_ang_world = estim_x_kk_robot_velo_ang_world

    # Landmarks
    # Constant
    # Do nothing
    # self.estim_map_world


    # Jacobian - Fx

    #
    jac_Fx = np.zeros((8+(3+1)*num_mapped_landmarks,(8+(3+1)*num_mapped_landmarks)), dtype=float)

    # Robot - robot
    jac_Fx_robot_robot = np.zeros((8,8), dtype=float)
    # Position k+1 - Position k
    # TODO BY STUDENT
    # jac_Fx_robot_robot[0:3, 0:3] = 
    # Position k+1 - Velocity k
    # TODO BY STUDENT
    # jac_Fx_robot_robot[0:3, 4:7] = 
    # Attitude k+1 - Attitude k
    jac_Fx_robot_robot[3, 3] = 1.0
    jac_Fx_robot_robot[3, 7] = delta_time
    # Velocity linear k+1 - Velocity linear k
    # TODO BY STUDENT
    # jac_Fx_robot_robot[4:7, 4:7] = 
    # Velocity angular k+1 - Velocity angular k
    jac_Fx_robot_robot[7, 7] = 1.0
    # Total
    jac_Fx[0:8, 0:8] = jac_Fx_robot_robot

    # Landmarks - Landmarks
    # TODO BY STUDENT
    # jac_Fx_landmarks_landmarks = 
    # Total
    jac_Fx[8:8+(3+1)*num_mapped_landmarks, 8:8+(3+1)*num_mapped_landmarks] = jac_Fx_landmarks_landmarks




    
    # Jacobian - Fn

    #
    jac_Fn = np.zeros((8+(3+1)*num_mapped_landmarks,4), dtype=float)

    # Robot - Robot noise
    jac_Fn_robot_robot_noise = np.zeros((8,4), dtype=float)
    # Velocity linear k+1 - Noise Velocity linear
    # TODO BY STUDENT
    # jac_Fn_robot_robot_noise[4:7, 0:3] = 
    # Velocity angular k+1 - Noise Velocity angular
    jac_Fn_robot_robot_noise[7,3] = 1.0

    #
    jac_Fn[0:8, 0:4] = jac_Fn_robot_robot_noise
    

    # Covariance
    # TODO BY STUDENT
    # estim_P_k1k = 



    # Prepare for next iteration

    #
    self.estim_state_timestamp = timestamp
    
    # State robot
    self.estim_robot_posi = estim_x_k1k_robot_posi
    self.estim_robot_atti_quat_simp = estim_x_k1k_robot_atti_quat_simp
    self.estim_robot_velo_lin_world = estim_x_k1k_robot_velo_lin_world
    self.estim_robot_velo_ang_world = estim_x_k1k_robot_velo_ang_world
    # State Landmarks
    # Do nothing
    # self.estim_map_world

    #
    self.estim_state_cov = estim_P_k1k


    #
    self.lock_state.release()

    #
    return


  def update(self):

    # Lock
    self.lock_meas.acquire()

    # Measurements readings - To avoid races
    #
    flag_set_meas_robot_posi = self.flag_set_meas_robot_posi
    if(flag_set_meas_robot_posi):
      meas_z_robot_posi = self.meas_robot_posi
    #
    flag_set_meas_robot_atti = self.flag_set_meas_robot_atti
    if(flag_set_meas_robot_atti):
      meas_z_robot_atti_quat_simp = self.meas_robot_atti_quat_simp
    #
    flag_set_meas_robot_vel_robot = self.flag_set_meas_robot_vel_robot
    if(flag_set_meas_robot_vel_robot):
      meas_z_robot_velo_lin_robot = self.meas_robot_velo_lin_robot
      meas_z_robot_velo_ang_robot = self.meas_robot_velo_ang_robot
    #
    flag_set_meas_circles_detected = self.flag_set_meas_circles_detected
    if(flag_set_meas_circles_detected):
      meas_z_circles_detected = self.meas_circles_detected
      num_meas_z_circles_detected = len(meas_z_circles_detected)

    # Put flags measurements down once used
    if(self.flag_set_meas_robot_posi == True):
      self.flag_set_meas_robot_posi = False

    if(self.flag_set_meas_robot_atti == True):
      self.flag_set_meas_robot_atti = False

    if(self.flag_set_meas_robot_vel_robot == True):
      self.flag_set_meas_robot_vel_robot = False

    if(self.flag_set_meas_circles_detected == True):
      self.flag_set_meas_circles_detected = False

    # Release
    self.lock_meas.release()


    # Dimension of the measurement for update
    # Init
    dim_meas = 0
    
    #
    if(flag_set_meas_robot_posi == True):
      dim_meas += 3
    #
    if(flag_set_meas_robot_atti == True):
      dim_meas += 1
    #
    if(flag_set_meas_robot_vel_robot == True):
      dim_meas += 4
    #
    if(flag_set_meas_circles_detected == True):
      # TODO BY STUDENT
      # dim_meas += 
      dim_meas


    # Check that there is at least one measurement
    if(dim_meas == 0):
      return


    # State readings - To avoid races

    #
    self.lock_state.acquire()
    
    # Robot
    estim_x_k1k_robot_posi = self.estim_robot_posi
    estim_x_k1k_robot_atti_quat_simp = self.estim_robot_atti_quat_simp
    estim_x_k1k_robot_velo_lin_world = self.estim_robot_velo_lin_world
    estim_x_k1k_robot_velo_ang_world = self.estim_robot_velo_ang_world

    # Landmarks
    # Do nothing
    # self.estim_map_world

    # Cov
    estim_P_k1k = self.estim_state_cov


    # robot atti - angle
    estim_x_k1k_robot_atti_ang = ars_lib_helpers.Quaternion.angleFromQuatSimp(estim_x_k1k_robot_atti_quat_simp)

    # robot atti - Rotation matrix 3d
    estim_x_k1k_robot_atti_rot_mat = np.zeros((3,3), dtype=float)
    estim_x_k1k_robot_atti_rot_mat = ars_lib_helpers.Quaternion.rotMat3dFromQuatSimp(estim_x_k1k_robot_atti_quat_simp)


    # num mapped landmarks
    num_mapped_landmarks = len(self.estim_map_world)

    # dim state
    dim_state = 8 + (3+1)*num_mapped_landmarks
    


    # Measurement matching

    if(flag_set_meas_circles_detected == True):
      # Only needed for meas circles detected. The others are directly associated!
      # Meas circle detected associated and unassociated
      meas_circles_detected_associated_idx = []
      meas_circles_detected_unassociated_idx = []
      
      #
      for meas_circles_detected_i_idx in range(0, num_meas_z_circles_detected):
        #
        meas_circles_detected_i_id = meas_z_circles_detected[meas_circles_detected_i_idx].id_lab

        #
        flag_associated = False
        for map_element_i_idx in range(0, num_mapped_landmarks):
          #
          map_element_i_id = self.estim_map_world[map_element_i_idx].id_lab
          #
          if(meas_circles_detected_i_id == map_element_i_id):
            # Associated
            meas_circles_detected_associated_idx.append([meas_circles_detected_i_idx, map_element_i_idx])
            #
            flag_associated = True
            #
            break
          
        if(flag_associated == False):
          # Not associated
          meas_circles_detected_unassociated_idx.append([meas_circles_detected_i_idx])
          


    # UPDATE WITH ASSOCIATED MEASUREMENTS

    # Dimension of the associated measurement for update
    # Init
    dim_meas_assoc = 0
    
    #
    if(flag_set_meas_robot_posi == True):
      dim_meas_assoc += 3
    #
    if(flag_set_meas_robot_atti == True):
      dim_meas_assoc += 1
    #
    if(flag_set_meas_robot_vel_robot == True):
      dim_meas_assoc += 4
    #
    if(flag_set_meas_circles_detected == True):
      # Loop
      for meas_circles_detected_associated_i_idx in meas_circles_detected_associated_idx:
        # TODO BY STUDENT
        # dim_meas_assoc += 
        dim_meas_assoc



    # Innovation of the associated measurement
    innov_meas = np.zeros((dim_meas_assoc,), dtype=float)
    innov_meas_idx = 0
    
    if(flag_set_meas_robot_posi == True):
      # Predicted measurement
      # TODO BY STUDENT
      # pred_z_robot_posi = 
      # Innovation of the measurement
      # TODO BY STUDENT
      # innov_meas_robot_posi = 
      # To the innovation vector
      innov_meas[innov_meas_idx:innov_meas_idx+3] = innov_meas_robot_posi
      innov_meas_idx += 3

    if(flag_set_meas_robot_atti == True):
      # Predicted measurement
      pred_z_robot_atti_quat_simp = estim_x_k1k_robot_atti_quat_simp
      # Innovation of the measurement
      innov_meas_robot_atti_quat_simp = ars_lib_helpers.Quaternion.computeDiffQuatSimp(pred_z_robot_atti_quat_simp, meas_z_robot_atti_quat_simp)
      # Converting to angle
      innov_meas_robot_atti_angle = ars_lib_helpers.Quaternion.angleFromQuatSimp(innov_meas_robot_atti_quat_simp)
      # To the innovation vector
      innov_meas[innov_meas_idx:innov_meas_idx+1] = innov_meas_robot_atti_angle
      innov_meas_idx += 1

    if(flag_set_meas_robot_vel_robot == True):
      # Predicted measurement
      # TODO BY STUDENT
      # pred_z_robot_velo_lin_robot = 
      pred_z_robot_velo_ang_robot = 1.0 * estim_x_k1k_robot_velo_ang_world
      # Innovation of the measurement
      # TODO BY STUDENT
      # innov_meas_robot_velo_lin_robot = 
      innov_meas_robot_velo_ang_robot = pred_z_robot_velo_ang_robot - meas_z_robot_velo_ang_robot
      # To the innovation vector
      innov_meas[innov_meas_idx:innov_meas_idx+3] = innov_meas_robot_velo_lin_robot
      innov_meas_idx += 3
      innov_meas[innov_meas_idx:innov_meas_idx+1] = innov_meas_robot_velo_ang_robot
      innov_meas_idx += 1

    if(flag_set_meas_circles_detected == True):
      # Only for the associated measurements
      
      # Loop
      for meas_circles_detected_associated_i_idx in meas_circles_detected_associated_idx:
        #
        meas_circles_detected_i_idx = meas_circles_detected_associated_i_idx[0]
        map_element_i_idx = meas_circles_detected_associated_i_idx[1]

        # Predicted measurement
        pred_z_circle_detected_i = ars_lib_helpers.Circle3D()
        # ID
        pred_z_circle_detected_i.id_lab = self.estim_map_world[map_element_i_idx].id_lab
        # Position
        # TODO BY STUDENT
        # pred_z_circle_detected_i.position = 
        # Attitude
        pred_z_circle_detected_i.attitude_quat_simp = self.estim_map_world[map_element_i_idx].attitude_quat_simp
        # Parent frame
        pred_z_circle_detected_i.parent_frame = meas_z_circles_detected[meas_circles_detected_i_idx].parent_frame
        # Radius
        # TODO BY STUDENT
        # pred_z_circle_detected_i.circle_radius = 

        # Innovation of the measurement
        # TODO BY STUDENT
        #innov_meas_circle_detected_posi_robot = 
        #innov_meas_circle_detected_radi = 

        # To the innovation vector
        innov_meas[innov_meas_idx:innov_meas_idx+3] = innov_meas_circle_detected_posi_robot
        innov_meas_idx += 3
        innov_meas[innov_meas_idx:innov_meas_idx+1] = innov_meas_circle_detected_radi
        innov_meas_idx += 1


    # Covariance of the measurement
    cov_meas = np.zeros((dim_meas_assoc, dim_meas_assoc), dtype=float)
    cov_meas_idx = 0
    
    if(flag_set_meas_robot_posi == True):
      cov_meas[cov_meas_idx:cov_meas_idx+3, cov_meas_idx:cov_meas_idx+3] = self.cov_meas_posi
      cov_meas_idx += 3

    if(flag_set_meas_robot_atti == True):
      cov_meas[cov_meas_idx:cov_meas_idx+1, cov_meas_idx:cov_meas_idx+1] = self.cov_meas_atti 
      cov_meas_idx += 1

    if(flag_set_meas_robot_vel_robot == True):
      cov_meas[cov_meas_idx:cov_meas_idx+3, cov_meas_idx:cov_meas_idx+3] = self.cov_meas_velo_lin
      cov_meas_idx += 3
      cov_meas[cov_meas_idx:cov_meas_idx+1, cov_meas_idx:cov_meas_idx+1] = self.cov_meas_velo_ang
      cov_meas_idx += 1

    if(flag_set_meas_circles_detected == True):
      # Loop
      for meas_circles_detected_associated_i_idx in meas_circles_detected_associated_idx:
        cov_meas[cov_meas_idx:cov_meas_idx+4, cov_meas_idx:cov_meas_idx+4] = self.cov_meas_circle_detected_robot
        cov_meas_idx += 4


    # Jacobian Hx
    jac_Hx = np.zeros((dim_meas_assoc, dim_state), dtype=float)
    jac_Hx_meas_idx = 0
    #
    diff_mat_R_ang = ars_lib_helpers.Quaternion.diffRotMat3dWrtAngleFromAngle(estim_x_k1k_robot_atti_ang)

    if(flag_set_meas_robot_posi == True):
      # Meas robot posi - robot posi
      # TODO BY STUDENT
      # jac_Hx[jac_Hx_meas_idx:jac_Hx_meas_idx+3, 0:3] = 
      jac_Hx_meas_idx += 3

    if(flag_set_meas_robot_atti == True):
      # Meas robot atti - robot atti
      jac_Hx[jac_Hx_meas_idx:jac_Hx_meas_idx+1, 3] = 1.0
      jac_Hx_meas_idx += 1

    if(flag_set_meas_robot_vel_robot == True):
      # Meas velo lin - robot atti
      jac_Hx[jac_Hx_meas_idx:jac_Hx_meas_idx+3, 3] = np.matmul(diff_mat_R_ang.T, estim_x_k1k_robot_velo_lin_world)
      # Meas velo lin - robot velo lin
      # TODO BY STUDENT
      # jac_Hx[jac_Hx_meas_idx:jac_Hx_meas_idx+3, 4:7] = 
      jac_Hx_meas_idx += 3
      # Meas velo ang - robot velo ang
      jac_Hx[jac_Hx_meas_idx:jac_Hx_meas_idx+1, 7] = 1.0
      jac_Hx_meas_idx += 1

    if(flag_set_meas_circles_detected == True):
      # Loop
      for meas_circles_detected_associated_i_idx in meas_circles_detected_associated_idx:
        #
        map_element_i_idx = meas_circles_detected_associated_i_idx[1]

        # Meas circle detected i posit - robot posi
        # TODO BY STUDENT
        # jac_Hx[jac_Hx_meas_idx:jac_Hx_meas_idx+3, 0:3] =
        # Meas circle detected i posit - robot atti
        jac_Hx[jac_Hx_meas_idx:jac_Hx_meas_idx+3, 3] = np.matmul(diff_mat_R_ang.T, (self.estim_map_world[map_element_i_idx].position - estim_x_k1k_robot_posi))
        # Meas circle detected i posit - landmark i posi
        jac_Hx_state_idx = 8 + map_element_i_idx*4
        # TODO BY STUDENT
        # jac_Hx[jac_Hx_meas_idx:jac_Hx_meas_idx+3, jac_Hx_state_idx:jac_Hx_state_idx+3] = 
        jac_Hx_meas_idx += 3
        # Meas circle detected i radi - landmark i radi
        jac_Hx_state_idx = 8 + map_element_i_idx*4
        # TODO BY STUDENT
        #jac_Hx[jac_Hx_meas_idx:jac_Hx_meas_idx+1, jac_Hx_state_idx+3] = 
        jac_Hx_meas_idx += 1


    # Covariance of the innovation of the measurement
    # TODO BY STUDENT
    # cov_innov_meas = 


    # Collective Mahalanobis test
    distance_maha_2 = np.matmul(np.matmul(innov_meas.T, np.linalg.inv(cov_innov_meas)), innov_meas)


    # Flag update
    flag_update_enabled = True
    

    # Update
    if(flag_update_enabled):

      # Update enabled


      # Kalman Gain
      # TODO BY STUDENT
      # kalman_gain = 


      # Updated state
      delta_x = np.matmul(kalman_gain, innov_meas)


      # Robot

      # Robot posi
      delta_x_robot_posi = delta_x[0:3]
      estim_x_k1k1_robot_posi = estim_x_k1k_robot_posi - delta_x_robot_posi
      # Robot attitude
      delta_x_robot_atti_ang = delta_x[3]
      delta_x_robot_atti_quat_sim = ars_lib_helpers.Quaternion.quatSimpFromAngle(delta_x_robot_atti_ang)
      estim_x_k1k1_robot_atti_quat_simp = ars_lib_helpers.Quaternion.computeDiffQuatSimp(estim_x_k1k_robot_atti_quat_simp, delta_x_robot_atti_quat_sim) 
      # Velocity linear 
      delta_x_robot_velo_lin_world = delta_x[4:7]
      estim_x_k1k1_robot_velo_lin_world = estim_x_k1k_robot_velo_lin_world - delta_x_robot_velo_lin_world
      # Velocity angular
      delta_x_robot_velo_ang_world = delta_x[7]
      estim_x_k1k1_robot_velo_ang_world = estim_x_k1k_robot_velo_ang_world - delta_x_robot_velo_ang_world
      
      # Map
      delta_x_idx = 8
      for map_element_i_idx in range(0, num_mapped_landmarks):
        #
        self.estim_map_world[map_element_i_idx].position = self.estim_map_world[map_element_i_idx].position - delta_x[delta_x_idx:delta_x_idx+3]
        delta_x_idx += 3
        #
        self.estim_map_world[map_element_i_idx].circle_radius = (self.estim_map_world[map_element_i_idx].circle_radius - delta_x[delta_x_idx:delta_x_idx+1])[0]
        delta_x_idx += 1



      # Updated covariance of state
      # TODO BY STUDENT
      # estim_P_k1k1 = 



      # Prepare for next iteration

      # Robot state
      self.estim_robot_posi = estim_x_k1k1_robot_posi
      self.estim_robot_atti_quat_simp = estim_x_k1k1_robot_atti_quat_simp
      self.estim_robot_velo_lin_world = estim_x_k1k1_robot_velo_lin_world
      self.estim_robot_velo_ang_world = estim_x_k1k1_robot_velo_ang_world

      # Landmarks
      # Do nothing
      # self.estim_map_world

      # Covariance
      self.estim_state_cov = estim_P_k1k1


    else:

      # No update!!

      # Prepare for next iteration

      # Robot state
      self.estim_robot_posi = estim_x_k1k_robot_posi
      self.estim_robot_atti_quat_simp = estim_x_k1k_robot_atti_quat_simp
      self.estim_robot_velo_lin_world = estim_x_k1k_robot_velo_lin_world
      self.estim_robot_velo_ang_world = estim_x_k1k_robot_velo_ang_world

      # Landmarks
      # Do nothing
      # self.estim_map_world

      # Covariance
      self.estim_state_cov = estim_P_k1k




    # MAPPING of unasociated measurements
    flag_mapping_enabled = True

    if(flag_mapping_enabled):

      if(flag_set_meas_circles_detected == True):
        # Loop
        for meas_circles_detected_unassociated_i_idx in meas_circles_detected_unassociated_idx:

          # Measured circle
          meas_z_circle_detected_i = meas_z_circles_detected[meas_circles_detected_unassociated_i_idx[0]]

          # Map function
          self.addMeasMapElementToEstimatedMap(meas_z_circle_detected_i, self.cov_meas_circle_detected_robot)


          pass



    #
    self.lock_state.release()

    
    #
    return



  def addMapElementToEstimatedMap(self, circle3d, circle3d_cov):

    # State
    # Push to the map
    self.estim_map_world.append(circle3d)

    # Covariance
    self.estim_state_cov = scipy.linalg.block_diag(self.estim_state_cov, circle3d_cov)

    # End
    return


  def addMeasMapElementToEstimatedMap(self, meas_circle3d, meas_circle3d_cov):

    # Reading state

    # Robot
    estim_x_robot_posi = self.estim_robot_posi
    estim_x_robot_atti_quat_simp = self.estim_robot_atti_quat_simp
    estim_x_robot_velo_lin_world = self.estim_robot_velo_lin_world
    estim_x_robot_velo_ang_world = self.estim_robot_velo_ang_world

    # Landmarks
    # Do nothing
    # self.estim_map_world

    # Cov
    estim_P_pre = self.estim_state_cov


    # robot atti - angle
    estim_x_robot_atti_ang = ars_lib_helpers.Quaternion.angleFromQuatSimp(estim_x_robot_atti_quat_simp)
    # robot atti - rot mat
    estim_x_robot_atti_rot_mat = ars_lib_helpers.Quaternion.rotMat3dFromQuatSimp(estim_x_robot_atti_quat_simp)


    #
    num_mapped_landmarks = len(self.estim_map_world)

    # 
    dim_meas = 3+1
    dim_state_pre = 8 + (3+1)*num_mapped_landmarks
    dim_state_fin = dim_state_pre + (3+1)


    # Jacobian - Gx
    jac_Gx = np.zeros((dim_state_fin, dim_state_pre), dtype=float)
    jac_Gx_new_state_idx = 0
    jac_Gx_pre_state_idx = 0

    # Gx - robot - robot
    # TODO BY STUDENT
    # jac_Gx[jac_Gx_new_state_idx:jac_Gx_new_state_idx+8, jac_Gx_pre_state_idx:jac_Gx_pre_state_idx+8] = 
    jac_Gx_new_state_idx += 8
    jac_Gx_pre_state_idx += 8

    # Gx - mapped landmarks - mapped landmarks
    for mapped_landmark_i in self.estim_map_world:
      # TODO BY STUDENT
      # jac_Gx[jac_Gx_new_state_idx:jac_Gx_new_state_idx+4, jac_Gx_pre_state_idx:jac_Gx_pre_state_idx+4] = 
      jac_Gx_new_state_idx += 4
      jac_Gx_pre_state_idx += 4

    # Gx - new landmark - robot
    jac_Gx_new_landmark_robot = np.zeros(((3+1),8), dtype=float)
    jac_Gx_pre_state_idx = 0
    # Gx - new landmark posi - robot posi
    # TODO BY STUDENT
    # jac_Gx_new_landmark_robot[0:3, 0:3] = 
    # Gx - new landmark posi - robot atti
    diff_mat_R = ars_lib_helpers.Quaternion.diffRotMat3dWrtAngleFromAngle(estim_x_robot_atti_ang)
    jac_Gx_new_landmark_robot[0:3, 3] = np.matmul(diff_mat_R, meas_circle3d.position)
    # 
    jac_Gx[jac_Gx_new_state_idx:jac_Gx_new_state_idx+4, jac_Gx_pre_state_idx:jac_Gx_pre_state_idx+8] = jac_Gx_new_landmark_robot
    jac_Gx_pre_state_idx += 8
    
    # Gx - new landmark - mapped landmarks
    for mapped_landmark_i in self.estim_map_world:
      # TODO BY STUDENT
      # jac_Gx[jac_Gx_new_state_idx:jac_Gx_new_state_idx+4, jac_Gx_pre_state_idx:jac_Gx_pre_state_idx+4] = 
      jac_Gx_pre_state_idx += 4



    # Jacobian - Gn
    jac_Gn = np.zeros((dim_state_fin, dim_meas), dtype=float)
    #
    jac_Gn_new_state_idx = dim_state_pre
    # New landmark posi - Meas landmark posi
    # TODO BY STUDENT
    # jac_Gn[jac_Gn_new_state_idx:jac_Gn_new_state_idx+3, 0:3] = 
    jac_Gn_new_state_idx += 3
    # New landmark radi - Meas landmark radi
    # TODO BY STUDENT
    # jac_Gn[jac_Gn_new_state_idx:jac_Gn_new_state_idx+1, 3] = 
    jac_Gn_new_state_idx += 1



    # Covariance of measurement
    cov_meas = meas_circle3d_cov


    # State
    # Estimated circle
    estim_circle3d = ars_lib_helpers.Circle3D()
    # ID
    estim_circle3d.id_lab = meas_circle3d.id_lab
    # Position
    # TODO BY STUDENT
    # estim_circle3d.position = 
    # Attitude
    estim_circle3d.attitude_quat_simp = meas_circle3d.attitude_quat_simp
    # Parent frame
    estim_circle3d.parent_frame = self.world_frame
    # Radius
    # TODO BY STUDENT
    # estim_circle3d.circle_radius = 

    

    # Covariance
    # TODO BY STUDENT
    # estim_P_fin = 



    # Push to the map
    self.estim_map_world.append(estim_circle3d)

    # Push to the state covariance
    self.estim_state_cov = estim_P_fin


    # End
    return
