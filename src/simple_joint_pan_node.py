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

from copy import deepcopy

class Segment:
  def __init__(self):
      self._start_position = 0
      self._start_velocity = 0
      self._acceleration = 0
      self._start_time = 0
      self._end_time = 0
      
  def sample(self, time_offset, start_position):
      bla = 0
 
   

class JointTrajectoryPanControl:
  
  
  def sample (self, time_offset, time_sample, position_begin, velocity_begin, acceleration):
        
        vel_sample = velocity_begin + acceleration * time_sample
        pos_sample = position_begin + velocity_begin * time_sample + 0.5 * acceleration * time_sample * time_sample
        
        trajectory_point = JointTrajectoryPoint()
        
        trajectory_point.time_from_start = rospy.Duration(time_offset + time_sample)
        trajectory_point.positions.append(pos_sample)
        trajectory_point.velocities.append(vel_sample)
        trajectory_point.accelerations.append(acceleration)        
        
        return trajectory_point
  
  def multi_sample (self, time_offset, time_sample_start, time_sample_end, num_samples, position_begin, velocity_begin, acceleration):
            
        time_increment = (time_sample_end - time_sample_start) / num_samples
        
        trajectory_points = []
    
        for i in range (0, num_samples+1):
	  
	    time_sample = time_sample_start + time_increment*i
            trajectory_points.append(self.sample(time_offset, time_sample , position_begin, velocity_begin, acceleration))
    
        return trajectory_points
  
  def set_motion_properties(self, time_from_start, velocity, max_acceleration, start_position, target_position):
  
        acceleration = math.copysign (max_acceleration, velocity)
        
        #When do we reach our desired velocity?
        dur_ramp_up = velocity/acceleration
        
        #from_start = 0
        
        # Path travelled till desired velocity reached
        s_ramp_up = 0 * dur_ramp_up + 0.5 * acceleration * dur_ramp_up * dur_ramp_up
        pos_ramp_up = start_position + s_ramp_up
        
        pos_ramp_down = target_position - s_ramp_up
        
        dur_mid = (pos_ramp_down - pos_ramp_up) / velocity
        
        dur_final = dur_ramp_up + dur_mid + dur_ramp_up
        
        #upper_segment = Segment()
        #mid_segment = Segment()
        #lower_segment = Segment()
        
        trajectory_points = []
        trajectory_points.extend(self.multi_sample(dur_ramp_up, 0, dur_mid, 3     , pos_ramp_up, velocity, 0))
        #trajectory_points.append(self.sample(dur_ramp_up, 0                     , pos_ramp_up, velocity, 0))
        #trajectory_points.append(self.sample(dur_ramp_up, dur_mid               , pos_ramp_up, velocity, 0))
        trajectory_points.append(self.sample(dur_ramp_up + dur_mid, dur_ramp_up , pos_ramp_down, velocity, -acceleration))
        trajectory_points.extend(self.multi_sample(dur_final + dur_ramp_up, 0, dur_mid, 3     , pos_ramp_down, -velocity, 0))        
        #trajectory_points.append(self.sample(dur_final + dur_ramp_up, 0         , pos_ramp_down, -velocity, 0))
        #trajectory_points.append(self.sample(dur_final + dur_ramp_up, dur_mid   , pos_ramp_down, -velocity, 0))
        trajectory_points.append(self.sample(dur_final + dur_ramp_up + dur_mid  , dur_ramp_up    , pos_ramp_up, -velocity, acceleration))
        
        return trajectory_points
        
    
  def vel_cmd_callback(self, data):	  
	self._velocity_command = data.data
	
  def __init__(self):
    
        self._velocity_command = 1.0
	  
        self._param_joint_name = "waist_lidar"
        self._param_acceleration = 3.14*5
        self._param_min_angle = -0.8
        self._param_max_angle = 0.8
	  
        self._client = actionlib.SimpleActionClient("/thor_mang/waist_lidar_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        self._client.wait_for_server(rospy.Duration.from_sec(0.5))
          
          
        self._goal_point = JointTrajectoryPoint()
        self._goal_point.positions.append(0.8)
        self._goal_point.velocities.append(0)
        self._goal_point.accelerations.append(0)
        self._goal_point.time_from_start = rospy.Duration(0.1)
         
        self._goal = FollowJointTrajectoryGoal()          
          
        
        self._goal.trajectory.joint_names.append(self._param_joint_name)
         
        self._vel_sub = rospy.Subscriber("velocity_command", Float64, self.vel_cmd_callback)
	
	  
  def run(self):
    
        up = True
        
        while not rospy.is_shutdown():
	  
	  if self._velocity_command <= 0.0001:
	    rospy.sleep(0.1)
	  else:
	    
	    if up == True: 
	      self._goal.trajectory.points = self.set_motion_properties(0, self._velocity_command, self._param_acceleration, self._param_min_angle, self._param_max_angle)
	      #up = False

	    #else:
	      #self._goal.trajectory.points = self.set_motion_properties(0, -self._velocity_command, self._param_acceleration, self._param_max_angle, self._param_min_angle)
	      #up = True
	    
	    self._goal.trajectory.header.stamp = rospy.Time.now()
	    self._client.send_goal(self._goal)
	    self._client.wait_for_result()	    

if __name__ == "__main__":
  
	rospy.init_node("simple_joint_pan_node")
	rospy.sleep(0.5)
	
	pan_control = JointTrajectoryPanControl()
	
	pan_control.run()