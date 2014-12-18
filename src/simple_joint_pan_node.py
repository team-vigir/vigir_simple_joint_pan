#!/usr/bin/env python
import roslib; roslib.load_manifest('vigir_simple_joint_pan')
import math
import rospy
import sensor_msgs.msg
import actionlib
#import brics_actuator.msg
from control_msgs.msg import *
from trajectory_msgs.msg import *
from std_msgs.msg import *
#import numpy as np

#class JointTrajectoryPanControl:
    
        #def vel_cmd_callback(data):	  
	  #self._client.send_goal(goal)	  
	
	#def __init__(self):	
	  
	  #self._param_joint_name = "joint"
	  
          #self._client = actionlib.SimpleActionClient("action_topic", FollowJointTrajectoryAction)
          #self._client.wait_for_server(rospy.Duration.from_sec(0.5))
          
          
          #self._goal_point = JointTrajectoryPoint()
          #self._goal_point.positions.append(0)
          #self._goal_point.velocities.append(0)
          #self._goal_point.accelerations.append(0)
          #self._goal_point.time_from_start = rospy.Duration(2.0)
          
          #self._goal = FollowJointTrajectoryGoal()          
          
          #self._goal.trajectory.header.stamp = rospy.Time.now()
          #self._goal.trajectory.joint_names.append(self._param_joint_name)
          #self._goal.trajectory.points.append(self._goal_point)
          
          #self._vel_sub = rospy.Subscriber("velocity_command", Float64, self.vel_cmd_callback)
          
          #self._client.send_goal(self._goal)
	

def vel_cmd_callback(data):	  
	  print "bla"


if __name__ == "__main__":
  
	rospy.init_node("simple_joint_pan_node")
	rospy.sleep(0.5)

	
	#pan_control = JointTrajectoryPanControl()
	
	
        _param_joint_name = "waist_lidar"
	  
        _client = actionlib.SimpleActionClient("/thor_mang/waist_lidar_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        _client.wait_for_server(rospy.Duration.from_sec(0.5))
          
          
        _goal_point = JointTrajectoryPoint()
        _goal_point.positions.append(0.8)
        _goal_point.velocities.append(0)
        _goal_point.accelerations.append(0)
        _goal_point.time_from_start = rospy.Duration(0.1)
         
        _goal = FollowJointTrajectoryGoal()          
          
        
        _goal.trajectory.joint_names.append(_param_joint_name)
        _goal.trajectory.points.append(_goal_point)
         
        _vel_sub = rospy.Subscriber("velocity_command", Float64, vel_cmd_callback)
         
        _client.send_goal(_goal)
        
        while not rospy.is_shutdown():
	  _goal.trajectory.header.stamp = rospy.Time.now()
	  _client.send_goal(_goal)
	  _client.wait_for_result()
	  _goal_point.positions[0] = -_goal_point.positions[0]
	
	rospy.spin()