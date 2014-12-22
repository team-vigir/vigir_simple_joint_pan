#!/usr/bin/env python
import roslib; roslib.load_manifest('vigir_simple_joint_pan')
import math
import rospy
import sensor_msgs.msg
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from std_msgs.msg import *
from copy import deepcopy


class JointTrajectoryPanControl:
  
  def get_trajectory_total_time(self, trajectory):
        return trajectory[-1].time_from_start
      
  # Copies trajectory and shifts time from start to future
  def shift_and_clone_trajectory_points(self, trajectory_points):
        
        trajectory_points_clone = deepcopy(trajectory_points)
        
        for i in trajectory_points_clone:
          i.time_from_start += trajectory_points[-1].time_from_start
	  
	return trajectory_points_clone
	  
  
  # Samples trajectory at given time_sample
  def sample (self, time_offset, time_sample, position_begin, velocity_begin, acceleration):
        
        vel_sample = velocity_begin + acceleration * time_sample
        pos_sample = position_begin + velocity_begin * time_sample + 0.5 * acceleration * time_sample * time_sample
        
        trajectory_point = JointTrajectoryPoint()
        
        trajectory_point.time_from_start = rospy.Duration(time_offset + time_sample)
        trajectory_point.positions.append(pos_sample)
        trajectory_point.velocities.append(vel_sample)
        trajectory_point.accelerations.append(acceleration)        
        
        return trajectory_point
  
  # Samples trajectory at multiple time samples
  def multi_sample (self, time_offset, time_sample_start, time_sample_end, num_samples, position_begin, velocity_begin, acceleration):
            
        time_increment = (time_sample_end - time_sample_start) / num_samples
        
        trajectory_points = []
    
        for i in range (0, num_samples+1):
	  
	    time_sample = time_sample_start + time_increment*i	    
            trajectory_points.append(self.sample(time_offset, time_sample , position_begin, velocity_begin, acceleration))
    
        return trajectory_points
  
  # Computes trajectory starting at 0 at ending at 0
  def set_motion_properties(self, time_from_start, velocity, max_acceleration, start_position, target_position):
  
        acceleration = math.copysign (max_acceleration, velocity)
        
        #When do we reach our desired velocity?
        dur_ramp_up = velocity/acceleration
        
        from_start = 0
        
        # Path travelled till desired velocity reached
        s_ramp_up =  0.5 * acceleration * dur_ramp_up * dur_ramp_up
        
        pos_ramp_up = start_position + s_ramp_up        
        pos_ramp_down = target_position - s_ramp_up
        
        pos_mid = (start_position + target_position) * 0.5
        
        dur_mid = (pos_ramp_down - pos_ramp_up) / velocity       
        
        if dur_mid > 0:
        
          dur_final =  from_start + dur_mid*0.5 + dur_ramp_up
          
          first_time_from_start = (dur_mid * 0.5) / 4
        
          trajectory_points = []
          trajectory_points.extend(self.multi_sample(from_start, first_time_from_start, dur_mid*0.5, 3 , pos_mid, velocity, 0))
          trajectory_points.append(self.sample(from_start + dur_mid*0.5, dur_ramp_up , pos_ramp_down, velocity, -acceleration))
          trajectory_points.extend(self.multi_sample(dur_final + dur_ramp_up, 0, dur_mid, 3     , pos_ramp_down, -velocity, 0))        
          trajectory_points.append(self.sample(dur_final + dur_ramp_up + dur_mid  , dur_ramp_up    , pos_ramp_up, -velocity, acceleration))
          trajectory_points.extend(self.multi_sample(dur_final + dur_ramp_up*3 + dur_mid, 0, dur_mid*0.5, 3, pos_ramp_up, velocity, 0))
                
          return trajectory_points
	
	else:
	  rospy.logwarn("Infeasible velocity commanded for joint %s", self._param_joint_name)
	  trajectory_points = []
	  return trajectory_points
	  
        
  # Velocity callback. Recomputes trajectory when new velocity command comes in  
  def vel_cmd_callback(self, data):
	self._velocity_command = data.data

        if (self._velocity_command > 0):
	  self._trajectory_points = self.set_motion_properties(0, self._velocity_command, self._param_acceleration, self._param_min_angle, self._param_max_angle)
	else:
	  self._trajectory_points = []
	  
	self._client.cancel_all_goals()  
	
  def __init__(self):
    
        self._velocity_command = 2.0
	  
        self._param_joint_name = "waist_lidar"
        self._param_acceleration = 3.14*5
        self._param_min_angle = -0.8
        self._param_max_angle = 0.8
	  
        self._client = actionlib.SimpleActionClient("/thor_mang/waist_lidar_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        self._client.wait_for_server(rospy.Duration.from_sec(0.5))
          
        self._goal = FollowJointTrajectoryGoal()          
          
        
        self._goal.trajectory.joint_names.append(self._param_joint_name)
         
        self._vel_sub = rospy.Subscriber("velocity_command", Float64, self.vel_cmd_callback)
        
        self._trajectory_points = self.set_motion_properties(0, self._velocity_command, self._param_acceleration, self._param_min_angle, self._param_max_angle)
	
	  
  def run(self):        
    
        next_start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():		  
	  
	  if (len(self._trajectory_points) == 0):
              self._client.cancel_all_goals()
	  elif ( (rospy.Time.now() > (next_start_time - rospy.Duration(0.1)))): #or (self._client.get_state != 1)):
	    
	      traj_time = self.get_trajectory_total_time(self._trajectory_points)
	      self._goal.trajectory.points = deepcopy(self._trajectory_points)
	      clone = self.shift_and_clone_trajectory_points(self._goal.trajectory.points)
	      self._goal.trajectory.points.extend(clone)

	      self._goal.trajectory.header.stamp = next_start_time
	      self._client.send_goal(self._goal)
	      next_start_time = next_start_time + traj_time
	      rospy.sleep(0.1)	   

if __name__ == "__main__":
  
	rospy.init_node("simple_joint_pan_node")
	rospy.sleep(0.5)
	
	pan_control = JointTrajectoryPanControl()
	
	pan_control.run()