#!/usr/bin/env python

# Standard Python Imports
import os
import numpy as np
import time
import imp
import sys
from argparse import ArgumentParser

# OpenRAVE
from openravepy import *
#RaveInitialize(True, DebugLevel.Debug)

# Import Dfab Python, Watch out for hard coded directory
dfab_pack = imp.load_package('dfab', './dfab-master/python/dfab/')
from dfab.mocap import extract_trajectory, datafiles
from dfab.geometry.quaternion import to_threexform
from dfab.rapid.joint_sequence import single_trajectory_program

curr_path = os.getcwd()
relative_ordata = '/models'
ordata_path_thispack = curr_path + relative_ordata

#this sets up the OPENRAVE_DATA environment variable to include the files we're using
openrave_data_path = os.getenv('OPENRAVE_DATA', '')
openrave_data_paths = openrave_data_path.split(':')
if ordata_path_thispack not in openrave_data_paths:
  if openrave_data_path == '':
      os.putenv('OPENRAVE_DATA', ordata_path_thispack)
  else:
      os.putenv('OPENRAVE_DATA', '%s:%s'%(ordata_path_thispack, openrave_data_path))


class RoboHandler:
  def __init__(self, mode):
    self.env = Environment()
    self.env.SetViewer('qtcoin')
    self.env.GetViewer().SetName('Tutorial Viewer')
    self.env.Load('6640_test.env.xml')
    self.robot = self.env.GetRobots()[0]

    # Init IK Solutions
    self.manip = self.robot.GetActiveManipulator()
    ikmodel = databases.inversekinematics.InverseKinematicsModel(self.robot)
    if not ikmodel.load():
      ikmodel.generate()

    # Storage of real/sim mode
    self.mode = mode
    
  def getMocapData(self, filename, body='6 Point Trowel', subsample=12):
    '''
    Looks up a csv filename for mocap data and if successful, returns times, x, 
    q, ypr in world frame.
    @ Params -> filename : CSV File with Mocap Data
    @ Returns -> times   : vector of times corresponding to readings
                 x       : (x, y, z) of centroid of body relative to world frame
                 q       : quaternion of body relative to world frame
                 ypr     : Yaw, Pitch, and Roll of body relative to world frame 
    '''
    data = extract_trajectory.load_csv_data(filename)
    return extract_trajectory.extract_trajectory(data, body=body)

  def getMocapTraj(self, filename ):
    '''
    Looks up a csv filename for mocap trajectory and if successful, returns times, 
    and 4x4 transforms in world frame.
    @ Params -> filename : CSV File with Mocap Data
    @ Returns -> paths : list of 4x4 transforms found in traj file
                 times : vector of times corresponding to readings
                 
    '''
    paths, times = datafiles.read_frame_trajectory_file(filename)
    transforms = [np.identity(4) for i in paths]
    for i in xrange(0, len(transforms)):
      self.write_frame_record_to_transform(paths[i], transforms[i])

    return (transforms, times)

  def write_frame_record_to_transform( self, frame, transform ):
    """Write a 2-D frame list in file format into a numpy matrix."""
    # write the frame components into the homogeneous transform
    transform[0:3, 3] = frame[0] # origin
    transform[0:3, 0] = frame[1] # X axis
    transform[0:3, 1] = frame[2] # Y axis
    transform[0:3, 2] = frame[3] # Z axis

    # scale the millimeters from the file to meters for the graphics
    transform[0:3, 3] *= 0.001
    # Swap X and Y, as they're flipped between data and sim bot
    temp = transform[1, 3] 
    transform[1, 3] = transform[0, 3] 
    transform[0, 3] = temp

    return

  
  def moveIK(self, Tgoal, move=True): 
    '''
    Attempts to move the robots end effector to a given transform denoted by
    Tgoal.  Returns None if no solution is found or Returns the solution if
    there is one.
    '''
    # Get a solution from IK Solver
    sol = self.manip.FindIKSolution(Tgoal, IkFilterOptions.CheckEnvCollisions)
    # If solution is none, print info
    if sol == None:
      print "No Solution Found!"
      print Tgoal
    # If we're actually supposed to move, do it
    if move:
      self.robot.SetDOFValues(sol, self.manip.GetArmIndices())
    return sol

  def moveTrajectory(self, traj):
    '''
    Takes a trajectory, which is [for now] just a 2xn list of points to put the 
    end effector, where this list is a list of 4x4 transforms in the world frame 
    and i=1:n where n is the number of points.
    '''

    for pos, q in traj:
      # If Sim, then OR works in m, not mm
      if self.mode == 'sim':
        pos = pos /1000
      rot_t = to_threexform(q)
      Tgoal = np.dot(np.eye(4), np.eye(4)) # Start with Orientation
      Tgoal[0:3, 3] = pos                  # Add Position
      self.moveIK(Tgoal)
      time.sleep(.1)

  def moveTransforms(self, transforms, toolframe=False):
    '''
    Takes a list of transforms and moves the robot through the trajectory.
    '''
    # Iterate over all transforms
    for t in transforms:
      # If coordinates passed in tool frame, change to robot frame
      if toolframe:
        t = self.toolToRobotTransform(t)
      # Move the robot
      self.moveIK(t)
      time.sleep(.1)

  def toolToRobotTransform(self, transform):
    '''
    Transforms from the tool frame to the robot frame
    '''
    # Rotate about z by 90 degrees
    t_z_90 = numpy.array([[0, -1, 0, 0], 
                          [1,  0, 0, 0], 
                          [0,  0, 1, 0], 
                          [0, 0, 0, 1]])

    # Rotate about x by -90 degrees
    t_x_m90 = numpy.array([[1,  0, 0, 0], 
                           [0,  0, 1, 0], 
                           [0, -1, 0, 0], 
                           [0, 0, 0, 1]])

    # Frame change is rotate about z by 90, then x by -90
    frame_change = np.dot(t_z_90, t_x_m90)
    # Return H_wt dot H_tr
    return np.dot(transform, frame_change)

  def writeModFile(self, times, transforms, toolframe=False):
    '''
    Writes a mod file for the trajectory described by t (the list of times) and
    transforms (the list of 4x4 homogeneous transforms the robots end effector)
    goes through.  The toolframe parameter indicates whether the Transformations
    are in the tools frame (toolframe=True) or Robots frame (toolframe=False).
    '''
    # Get pruned times and joints from function
    times, joints = self.generateJointTrajectoryFromIK(times, transforms, toolframe=toolframe)

    # Format data
    data = [[time, [0] + j_vals.tolist()] for (time, j_vals) in zip(times, joints)]

    # Write File
    single_trajectory_program(data)

  def generateJointTrajectoryFromIK(self, times, transforms, toolframe=False):
    '''
    Generates a Joint Trajectory from a list of 4x4 homogeneous transforms the robots'
    end effector is supposed to go through and the list of times it is supposed
    to be there.  The toolframe parameter indicates whether the Transformations
    are in the tools frame (toolframe=True) or Robots frame (toolframe=False).
    '''

    # Initialize returned lists
    joint_traj = []
    traj_times = []
    # Iterate over trajectory
    for time, t in zip(times, transforms):
      if toolframe:
        # Transform tool frame to robot frame
        t = self.toolToRobotTransform(t)
      # Get Solution
      sol = self.moveIK(t, move=False)
      # If solution exists, append it to the list
      if sol != None:
        joint_traj.append(sol)
        traj_times.append(time)

    return traj_times, joint_traj



if __name__ == '__main__':

  parser = ArgumentParser( description = """Python Script for Planning or Simulating IRB 6640 with an OpenRAVE Environment.""")
  parser.add_argument('-m', '--mode', default='sim', help='Mode For Script to Run in.  Options are sim and real (default is sim)')
  parser.add_argument('-t', '--trajectory', help='Name of Trajectory File to read as input.')
  parser.add_argument('-c', '--csv', help='Name of Mocap CSV File captured with 6 point trowel to load as data')
  parser.add_argument('-b', '--body', default='6 Point Trowel', help='Name of Body in Mocap CSV File captured to look for (default is 6 Point Trowel')

  args = parser.parse_args()

  robo = RoboHandler(args.mode)

  if args.csv != None:
    data = robo.getMocapData(args.csv, body=args.body)

  if args.trajectory != None:
    (trans, times) = robo.getMocapTraj(args.trajectory)

 

  # Set Camera
  t = np.array([ [0, -1.0, 0, 2.25], [-1.0, 0, 0, 0], [0, 0, -1.0, 4.0], [0, 0, 0, 1.0] ])  
  robo.env.GetViewer().SetCamera(t)

  robo.robot.SetVisible(True)

  # Drop Into Shell
  import IPython
  IPython.embed()
  
  
