#!/usr/bin/env python3

"""
2D Controller Class to be used for the CARLA waypoint follower demo.
"""

import cutils
import numpy as np

class Controller2D(object):
    def __init__(self, waypoints):
        self.vars                = cutils.CUtils()
        self._current_x          = 0
        self._current_y          = 0
        self._current_yaw        = 0
        self._current_speed      = 0
        self._desired_speed      = 0
        self._current_frame      = 0
        self._current_timestamp  = 0
        self._start_control_loop = False
        self._set_throttle       = 0
        self._set_brake          = 0
        self._set_steer          = 0
        self._waypoints          = waypoints
        self._conv_rad_to_steer  = 180.0 / 70.0 / np.pi
        self._pi                 = np.pi
        self._2pi                = 2.0 * np.pi

    def update_values(self, x, y, yaw, speed, timestamp, frame):
        self._current_x         = x
        self._current_y         = y
        self._current_yaw       = yaw
        self._current_speed     = speed
        self._current_timestamp = timestamp
        self._current_frame     = frame
        if self._current_frame:
            self._start_control_loop = True

    def update_desired_speed(self):
        min_idx       = 0
        min_dist      = float("inf")
        desired_speed = 0
        for i in range(len(self._waypoints)):
            dist = np.linalg.norm(np.array([
                    self._waypoints[i][0] - self._current_x,
                    self._waypoints[i][1] - self._current_y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        if min_idx < len(self._waypoints)-1:
            desired_speed = self._waypoints[min_idx][2]
        else:
            desired_speed = self._waypoints[-1][2]
        self._desired_speed = desired_speed

    def update_waypoints(self, new_waypoints):
        self._waypoints = new_waypoints

    def get_commands(self):
        return self._set_throttle, self._set_steer, self._set_brake

    def set_throttle(self, input_throttle):
        # Clamp the throttle command to valid bounds
        throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        # Covnert radians to [-1, 1]
        input_steer = self._conv_rad_to_steer * input_steer_in_rad

        # Clamp the steering command to valid bounds
        steer           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        # Clamp the steering command to valid bounds
        brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake

    def wrap2pi(self, angle):
        if(angle > np.pi):
            angle -= 2*np.pi

        elif(angle < -np.pi):
            angle += 2*np.pi
        return angle


    def get_cross_error(self,xn,yn,xf,yf,x,y):

        pt1 = [xn-x,yn-y]
        pt2 = [xf-x,yf-y]

        prod = np.cross(pt2,pt1)

        sign = prod/abs(prod)
        
        error = sign*np.sqrt((xn-xf)**2 + (yn - yf)**2)

        return error

    def get_eucd_dist(self,x1,y1,x2,y2):

        return np.sqrt((x1-x2)**2 + (y1-y2)**2)
    
    def get_nearest_waypoint(self, xf,yf, waypoints):
        nearest_pt = []
        min_dist = np.Infinity

        for i in range (np.shape(waypoints)[0]):
            dist = self.get_eucd_dist(xf,yf,waypoints[i][0],waypoints[i][1])
            if(dist < min_dist):
                min_dist = dist
                nearest_pt = [waypoints[i][0],waypoints[i][1]]

        return nearest_pt,i

    def update_controls(self):
        ######################################################
        # RETRIEVE SIMULATOR FEEDBACK
        ######################################################
        x               = self._current_x
        y               = self._current_y
        yaw             = self._current_yaw
        v               = self._current_speed
        self.update_desired_speed()
        v_desired       = self._desired_speed
        t               = self._current_timestamp
        waypoints       = self._waypoints
        throttle_output = 0
        steer_output    = 0
        brake_output    = 0

        
       
        self.vars.create_var('v_previous', 0.0)
        self.vars.create_var('x_previous',0.0)
        self.vars.create_var('y_previous',0.0)
        self.vars.create_var('cum_error',0.0)
        self.vars.create_var('t_previous',0.0)
      
    


        

        # Skip the first frame to store previous values properly
        if self._start_control_loop:
            """
                Controller iteration code block.
                
                Controller Feedback Variables:
                    x               : Current X position (meters)
                    y               : Current Y position (meters)
                    yaw             : Current yaw pose (radians)
                    v               : Current forward speed (meters per second)
                    t               : Current time (seconds)
                    v_desired       : Current desired speed (meters per second)
                                      (Computed as the speed to track at the
                                      closest waypoint to the vehicle.)
                    waypoints       : Current waypoints to track
                                      (Includes speed to track at each x,y
                                      location.)
                                      Format: [[x0, y0, v0],
                                               [x1, y1, v1],
                                               ...
                                               [xn, yn, vn]]
                                      Example:
                                          waypoints[2][1]: 
                                          Returns the 3rd waypoint's y position

                                          waypoints[5]:
                                          Returns [x5, y5, v5] (6th waypoint)
                
                Controller Output Variables:
                    throttle_output : Throttle output (0 to 1)
                    steer_output    : Steer output (-1.22 rad to 1.22 rad)
                    brake_output    : Brake output (0 to 1)
            """
            
            self.vars.v_previous = v  # Store forward speed to be used in next step
            self.vars.x_previous = x
            self.vars.y_previous = y
         
            """
                Longitudinal Control (PID)
            """
            K_p = 1.0
            K_i = 0.5
            K_d = 0.01
            v_error = v_desired - v
            dt = t - self.vars.t_previous 
            self.vars.cum_error += v_error*dt
            acc = K_p*v_error + K_i*self.vars.cum_error + K_d*v_error/dt

            
            
            if acc >= 0:
                throttle_output = acc
                brake_output    = 0
            else:
                throttle_output = 0
                brake_output    = acc

           
            """
                Lateral Control (Stanley)
            """

            lk = 10
          

            wp_array = np.array(waypoints)



            l_f = 1.5 ##distance of center of vehicle to front axle
            x_f = x + l_f*np.cos(yaw) ##front axle position
            y_f = y + l_f*np.sin(yaw)
            
           
            nearest,idx = self.get_nearest_waypoint(x_f,y_f,waypoints) ##nearest waypoint to track
            
            track_angle = np.arctan2(waypoints[idx-100][1] -nearest[1],waypoints[idx-100][0] - nearest[0]) ##angle of trajectory to follow
            
            yaw = self.wrap2pi(yaw)
            psi = track_angle - yaw
            psi = self.wrap2pi(psi)
        
            cross_error = self.get_cross_error(nearest[0],nearest[1],x_f,y_f,x,y) ##get cross error from trajectory
           

            K_c = 8.0
            K_s = 0.00001 ##constant for numeric stability
           
            heading_correc = np.arctan2(K_c*cross_error,K_s + v) ##cross-error
          

            # Change the steer output with the lateral controller. 
            steer_output  = self.wrap2pi(psi + heading_correc)
           
            
            
            ######################################################
            # SET CONTROLS OUTPUT
            ######################################################
            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
            self.set_brake(brake_output)        # in percent (0 to 1)
     
    


        self.vars.v_previous = v  # Store forward speed to be used in next step
        self.vars.x_previous = x
        self.vars.y_previous = y
        self._start_control_loop = True
        self.vars.t_previous = t
     
        
