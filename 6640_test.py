#!/usr/bin/env python

# Standard Python Imports
import os
import copy
import time
import numpy as np
import scipy

# OpenRAVE
import openravepy
#openravepy.RaveInitialize(True, openravepy.DebugLevel.Debug)


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
  def __init__(self):
    self.env = openravepy.Environment()
    self.env.SetViewer('qtcoin')
    self.env.GetViewer().SetName('Tutorial Viewer')
    self.env.Load('6640_test.env.xml')
    # time.sleep(3) # wait for viewer to initialize. May be helpful to uncomment
    self.robot = self.env.GetRobots()[0]
    

  #remove all the time.sleep(0) statements! Those are just there so the code can run before you fill in the functions

  # move in a straight line, depending on which direction the robot is facing
  def move_straight(self, dist):
    t_move = np.array(([1, 0, 0, dist], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]))
    with self.env:
      self.robot.SetTransform(np.dot(self.robot.GetTransform(), t_move))

  # rotate the robot about the z-axis by the specified angle (in radians)
  def rotate_by(self, ang):
    t_rot = openravepy.matrixFromAxisAngle([0, 0, ang])
    with self.env:
      self.robot.SetTransform(np.dot(self.robot.GetTransform(), t_rot))

  # go to each of the square corners, point towards the center, and snap a photo!
  def go_around_square(self):
    with self.env:
      self.robot.SetTransform(np.array(([1, 0, 0, -1], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1])))

    # No need for with self.env, as it is embedded in all calls
    for i in xrange(0, 4):
      self.move_straight(2)
      self.rotate_by(5*np.pi/4)
      time.sleep(5)
      self.rotate_by(np.pi/4)    

    # set the robot back to the initialize position after
    with self.env:
      self.robot.SetTransform(np.identity(4)); 

  # a function to help figure out which DOF indices correspond to which part of HERB
  def figure_out_DOFS(self):
    with self.env:
      self.robot.GetJoints()  

  # put herb in self collision
  def put_in_self_collision(self):
    with self.env:
      self.robot.SetDOFValues([3], dofindices=[1], checklimits=0)

  # saves an image from above, pointed straight down
  def save_viewer_image_topdown(self, imagename):
    TopDownTrans = np.array([ [0, -1.0, 0, 0], [-1.0, 0, 0, 0], [0, 0, -1.0, 5.0], [0, 0, 0, 1.0] ])
    #seems to work without this line...but its in the tutorial, so I'll keep it here in case
    self.env.GetViewer().SendCommand('SetFiguresInCamera 1') # also shows the figures in the image
    I = self.env.GetViewer().GetCameraImage(640,480,  TopDownTrans,[640,640,320,240])
    scipy.misc.imsave(imagename + '.jpg',I)
      

if __name__ == '__main__':
  robo = RoboHandler()

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
  
  
