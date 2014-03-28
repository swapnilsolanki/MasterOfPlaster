#!/usr/bin/env python

# Standard Python Imports
import os
import copy
import time
import numpy as np
import scipy
import imp
from argparse import ArgumentParser

# OpenRAVE
from openravepy import *
#openravepy.RaveInitialize(True, openravepy.DebugLevel.Debug)

# Import Dfab Python, Watch out for hard coded directory
dfab_pack = imp.load_package('dfab', '../dfab/python/dfab/')
from dfab.mocap import extract_trajectory

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
    # time.sleep(3) # wait for viewer to initialize. May be helpful to uncomment
    self.robot = self.env.GetRobots()[0]

    # Init IK Solutions
    self.manip = self.robot.GetActiveManipulator()
    ikmodel = databases.inversekinematics.InverseKinematicsModel(self.robot)
    if not ikmodel.load():
      ikmodel.generate()

    # Storage of real/sim mode
    self.mode = mode
    
  def getMocapData(self, filename, body='6 Point Trowel'):
    '''
    Looks up a csv filename for mocap data and if successful, returns times, x, 
    q, ypr.
    @ Params -> filename : CSV File with Mocap Data
    @ Returns -> times   : vector of times corresponding to readings
                 x       : (x, y, z) of centroid of body relative to world frame
                 q       : quaternion of body relative to world frame
	         ypr     : Yaw, Pitch, and Roll of body relative to world frame 
    '''
    data = extract_trajectory.load_csv_data(filename)
    return extract_trajectory.extract_trajectory(data, body=body)
  
  def moveIK(self, Tgoal): 
    '''
    Attempts to move the robots end effector to a given transform denoted by
    Tgoal.  Returns False if no IK solution was found, and True if the robot
    Moved.
    '''
    sol = self.manip.FindIKSolution(Tgoal, IkFilterOptions.CheckEnvCollisions)
    if sol == None:
      print "No Solution Found!"
      return False
    self.robot.SetDOFValues(sol, self.manip.GetArmIndices())
    return True

if __name__ == '__main__':

  parser = ArgumentParser( description = """Python Script for Planning or Simulating IRB 6640 with an OpenRAVE Environment.""")
  parser.add_argument('-m', '--mode', default='sim', help='Mode For Script to Run in.  Options are sim and real (default is sim)')
  parser.add_argument('-t', '--trajectory', help='Name of Trajectory File to Follow.')
  parser.add_argument('-c', '--csv', help='Name of Mocap CSV File captured with 6 point trowel to follow')

  args = parser.parse_args()

  robo = RoboHandler(args.mode)

  # Uncomment the following to make the script initialize the RoboHandler
  #  and drop you into an IPython shell.
  t = np.array([ [0, -1.0, 0, 0], [-1.0, 0, 0, 0], [0, 0, -1.0, 5.0], [0, 0, 0, 1.0] ])  
  robo.env.GetViewer().SetCamera(t)

  robo.robot.SetVisible(True)

  import IPython
  IPython.embed()

  # spin forever
  while True:
    time.sleep(0)
  
  
