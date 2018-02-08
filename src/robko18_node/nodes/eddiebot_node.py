#!/usr/bin/env python
# Software License Agreement (BSD License)
# Copyright (c) 2010, Willow Garage Inc. 
# Copyright (c) 2012, Tang Tiong Yew
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id: __init__.py 11217 2010-09-23 21:08:11Z kwc $

#Stefan:obsolete used with the old rospackage style, we don't need it in catkin
# import roslib; roslib.load_manifest('eddiebot_node')

"""
ROS Robko18 node (based on the code for Eddiebot node) buit on top robko18_driver.
(ROS Eddiebot node for ROS built on top of eddiebot_driver.)
This driver is based on otl_roomba by OTL (otl-ros-pkg).

eddiebot_driver is based on Damon Kohler's pyrobot.py.
"""

import os
import sys
import select
import serial
import termios
import time
import math 

from math import sin, cos

import roslib.rosenv
import rospy
import tf

from geometry_msgs.msg import Point, Pose, Pose2D, PoseWithCovariance, \
    Quaternion, Twist, TwistWithCovariance, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from eddiebot_driver import Eddiebot, MAX_WHEEL_SPEED, DriverError
from eddiebot_node.msg import EddiebotSensorState, Drive, Eddie
from eddiebot_node.srv import SetEddiebotMode,SetEddiebotModeResponse, SetDigitalOutputs, SetDigitalOutputsResponse
from eddiebot_node.diagnostics import EddiebotDiagnostics
from eddiebot_node.gyro import EddiebotGyro
import eddiebot_node.robot_types as robot_types
from eddiebot_node.covariances import \
     ODOM_POSE_COVARIANCE, ODOM_POSE_COVARIANCE2, ODOM_TWIST_COVARIANCE, ODOM_TWIST_COVARIANCE2
from eddiebot_node.songs import bonus

#dynamic reconfigure
import dynamic_reconfigure.server
from eddiebot_node.cfg import EddieBotConfig


class EddiebotNode(object):
    UNDEFINED = -1000
    _SENSOR_READ_RETRY_COUNT = 5 
    
    ENCODER_TICKS = 144
    WHEEL_BASE = 39.0525 # cm
    WHEEL_RADIUS = 7.62 # cm
    MILLIVOLTS_PER_VOLT = 3.918457 # magic constant
    CENTIMETERS_PER_TICK = 0.332485223 # 2 * PI * WHEEL_RADIUS / ENCODER_TICKS
    METERS_PER_TICK = 0.00332485223
    DEGREES_PER_TICK = 0.975609758 # CENTIMETERS_PER_TICK / (PI * WHEEL_BASE) * 360
    RADIANS_PER_TICK = 0.017027602
    
    def __init__(self, default_port='/dev/ttyUSB0', default_update_rate=120.0):

        """
        @param default_port: default tty port to use for establishing
            connection to Eddiebot.  This will be overriden by ~port ROS
            param if available.
        """
        self.default_port = default_port
        self.default_update_rate = default_update_rate

        self.robot = Eddiebot()
        self.sensor_handler = None
        self.sensor_state = EddiebotSensorState()
        self.req_cmd_vel = None

        rospy.init_node('eddiebot')
        self._init_params()
        self._init_pubsub()
        
        self._pos2d = Pose2D() # 2D pose for odometry

        self._diagnostics = EddiebotDiagnostics()
        if self.has_gyro:
            self._gyro = EddiebotGyro()
        else:
            self._gyro = None
            
        dynamic_reconfigure.server.Server(EddieBotConfig, self.reconfigure)
        
        self.last_pan_degree = 96 # Default is facing center 
        
        self.last_angle = self.UNDEFINED
        self.last_dist = self.UNDEFINED
        self.emergency_activated = False
        
        self.linear_speed = 0
        self.angular_speed = 0

    def start(self):
        log_once = True
        while not rospy.is_shutdown():
            try:
                self.robot.start(self.port, robot_types.ROBOT_TYPES[self.robot_type].baudrate)
                break
            except serial.serialutil.SerialException as ex:
                msg = "Failed to open port %s.  Please make sure the Create cable is plugged into the computer. \n"%(self.port)
                self._diagnostics.node_status(msg,"error")
                if log_once:
                    log_once = False
                    rospy.logerr(msg)
                else:
                    sys.stderr.write(msg)
                os.system("fuser -k /dev/ttyUSB0")
#                time.sleep(3.0)

        #self.robot.sci.start_time = time.time()
        #self.robot.sci.com_log =  open('/home/eddie/logs/com_log_' + str(int(self.robot.sci.start_time)), 'w')
        #self.robot.sci.com_log.write('cur_time,command\n')

        self.sensor_handler = robot_types.ROBOT_TYPES[self.robot_type].sensor_handler(self.robot) 
        self.robot.safe = True

 #       if rospy.get_param('~bonus', False):
 #           bonus(self.robot)

        #self.robot.control()
        # Write driver state to disk
        with open(connected_file(), 'w') as f:
            f.write("1")

        # Disable all sensors Startup readings from Create can be incorrect, discard first values
        s = EddiebotSensorState()
        try:
            self.sense(s)
        except Exception:
            # packet read can get interrupted, restart loop to
            # check for exit conditions
            pass


    def _init_params(self):
        self.port = rospy.get_param('~port', self.default_port)
        self.robot_type = rospy.get_param('~robot_type', 'eddie')
        #self.baudrate = rospy.get_param('~baudrate', self.default_baudrate)
        self.update_rate = rospy.get_param('~update_rate', self.default_update_rate)
        self.drive_mode = rospy.get_param('~drive_mode', 'twist')
        self.has_gyro = rospy.get_param('~has_gyro', False)
        self.odom_angular_scale_correction = rospy.get_param('~odom_angular_scale_correction', 1.0)
        self.odom_linear_scale_correction = rospy.get_param('~odom_linear_scale_correction', 1.0)
        self.cmd_vel_timeout = rospy.Duration(rospy.get_param('~cmd_vel_timeout', 0.2))
        self.stop_motors_on_bump = rospy.get_param('~stop_motors_on_bump', True)
        self.min_abs_yaw_vel = rospy.get_param('~min_abs_yaw_vel', None)
        self.max_abs_yaw_vel = rospy.get_param('~max_abs_yaw_vel', None)
        self.publish_tf = rospy.get_param('~publish_tf', True)
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.base_frame = rospy.get_param('~base_frame', 'base_footprint')
        self.operate_mode = rospy.get_param('~operation_mode', 3)

        rospy.loginfo("serial port: %s"%(self.port))
        rospy.loginfo("update_rate: %s"%(self.update_rate))
        rospy.loginfo("drive mode: %s"%(self.drive_mode))
        rospy.loginfo("has gyro: %s"%(self.has_gyro))
        
        
        self.max_linear_speed = rospy.get_param('~max_linear_speed_m_per_sec', 1)
        self.max_slow_speed = rospy.get_param('~max_slow_speed_m_per_sec', 0.3)
        self.max_angular_speed = rospy.get_param('~max_angular_speed_deg_per_sec', 40)
        self.max_acc = int(rospy.get_param('~max_acceleration_cm_per_sec_square', 20) / self.CENTIMETERS_PER_TICK)
        self.slow_speed_distance = rospy.get_param('~slow_speed_distance_cm', 150)
        self.stop_distance = rospy.get_param('~stop_distance_cm', 30)
        self.check_distance = rospy.get_param('~check_distance', True)
            

    def _init_pubsub(self):
        self.joint_states_pub = rospy.Publisher('joint_states', JointState)
        self.odom_pub = rospy.Publisher('odom', Odometry)

        self.sensor_state_pub = rospy.Publisher('~sensor_state', EddiebotSensorState)
        self.operating_mode_srv = rospy.Service('~set_operation_mode', SetEddiebotMode, self.set_operation_mode)
        self.digital_output_srv = rospy.Service('~set_digital_outputs', SetDigitalOutputs, self.set_digital_outputs)

        if self.drive_mode == 'twist':
            #self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel)
            self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self.web_and_joy_cmd_vel)
            self.drive_cmd = self.robot.direct_drive
            self.cmd_joints_sub = rospy.Subscriber('cmd_joints', JointState, self.process_cmd_joints)
            #self.cmd_navigation_vel_sub = rospy.Subscriber('navigation_velocity_smoother/raw_cmd_vel', Twist, self.cmd_navigation_vel)
            self.cmd_navigation_vel_sub = rospy.Subscriber('navigation_velocity_smoother/raw_cmd_vel', Twist, self.nav_cmd_vel)
            
        elif self.drive_mode == 'drive':
            self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Drive, self.cmd_vel)
            self.drive_cmd = self.robot.direct_drive
        elif self.drive_mode == 'eddie':
            self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Eddie, self.cmd_vel)
            self.drive_cmd = self.robot.direct_drive
        else:
            rospy.logerr("unknown drive mode :%s"%(self.drive_mode))

        self.transform_broadcaster = None
        if self.publish_tf:
            self.transform_broadcaster = tf.TransformBroadcaster()
            
        self.emergency_stop_sub = rospy.Subscriber('emergency_stop', Bool, self.emergency_stop)
    
    def reconfigure(self, config, level):
        self.update_rate = config['update_rate']
        self.drive_mode = config['drive_mode']
        self.has_gyro = config['has_gyro']
 #       if self.has_gyro:
 #           self._gyro.reconfigure(config, level)
        self.odom_angular_scale_correction = config['odom_angular_scale_correction']
        self.odom_linear_scale_correction = config['odom_linear_scale_correction']
        self.cmd_vel_timeout = rospy.Duration(config['cmd_vel_timeout'])
        self.stop_motors_on_bump = config['stop_motors_on_bump']
        self.min_abs_yaw_vel = config['min_abs_yaw_vel']
        self.max_abs_yaw_vel = config['max_abs_yaw_vel']
        return config
    
    def process_cmd_joints(self, msg):
        if(msg.position[0] > 0):
            pan_degree = self.last_pan_degree - (msg.velocity[0] / 3) * 10
        else:
            pan_degree = self.last_pan_degree + (msg.velocity[0] / 3) * 10
        print pan_degree
#        if(msg.position[1] > 0):
#            tilt_degree = 97 - (msg.velocity[1] / 3) * 97
#        else:
#            tilt_degree = 97 + (msg.velocity[1] / 3) * 97
        tilt_degree = 0.0
#        if(abs(self.last_pan_degree - pan_degree) > 10): # Smooth the servo and cancer minor movement
        if(pan_degree > 180):
            pan_degree = 180
        if(pan_degree < 0):
            pan_degree = 0
        self.joints_cmd = self.robot.command_joints(pan_degree, tilt_degree)
        self.last_pan_degree = pan_degree
#        self.tilt_degree = msg.
#        selft.pan_degree = msg.
    
    def emergency_stop(self, msg):
        if msg.data == True:
            print ("Software emergency stop activated")
        else:
            print ("Software emergency stop deactivated") 
        self.emergency_activated = msg.data
    
    def limit_speed(self):
    #if self.check_distance:
      if self.linear_speed > 0:
        if self.sensor_state.cliff_front_right_signal < self.stop_distance:
          self.linear_speed = 0
        elif self.sensor_state.wall_signal < self.slow_speed_distance or self.sensor_state.cliff_left_signal < self.slow_speed_distance:
          self.linear_speed = min(self.max_slow_speed, self.linear_speed)
    
    def web_and_joy_cmd_vel(self, msg):
      self.linear_speed = self.max_linear_speed * msg.linear.x
      self.angular_speed = self.max_angular_speed * msg.angular.z
        
      #self.limit_speed()
      if self.angular_speed > 15:
        self.angular_speed = 15
      
      if self.angular_speed < -15:
        self.angular_speed = -15
      
        
      linear_speed_ticks = self.linear_speed / self.METERS_PER_TICK
      angular_speed_ticks = self.angular_speed / self.DEGREES_PER_TICK
         
      s1 = linear_speed_ticks + angular_speed_ticks
      s2 = linear_speed_ticks - angular_speed_ticks
      
      self.req_cmd_vel = int(s1), int(s2)
      print self.req_cmd_vel
      
      
    def nav_cmd_vel(self, msg):
    
        if msg.angular.z > 0.26:
          msg.angular.z = 0.26
        
        if msg.angular.z < -0.26:
          msg.angular.z = -0.26
        
        
        
        if msg.linear.x > 0.1:
          msg.linear.x = 0.1
        
        if msg.linear.x < -0.1:
          msg.linear.x = -0.1
            
        if msg.linear.x < 0.001:    
          if msg.angular.z > 0 and msg.angular.z < 0.3:
            msg.angular.z = 0.3
              
          if msg.angular.z < 0 and msg.angular.z > -0.3:
            msg.angular.z = -0.3
              
        linear_speed_ticks = msg.linear.x / self.METERS_PER_TICK
        angular_speed_ticks = msg.angular.z / self.RADIANS_PER_TICK
        
        #if linear_speed_ticks < 11:
        #  linear_speed_ticks = 0
        
        s1 = linear_speed_ticks + angular_speed_ticks
        s2 = linear_speed_ticks - angular_speed_ticks
        
        #while linear_speed_ticks > 11 and (s1 < 11 or s2 < 11):
        #  s1 = s1 + 3
        #  s2 = s2 + 3
        
        print s1, s2
        
        self.req_cmd_vel = int(s1), int(s2)
        
    def set_operation_mode(self,req):
        if not self.robot.sci:
            raise Exception("Robot not connected, SCI not available")

        self.operate_mode = req.mode

        if req.mode == 1: #passive
            self._robot_run_passive()
        elif req.mode == 2: #safe
            self._robot_run_safe()
        elif req.mode == 3: #full
            self._robot_run_full()
        else:
            rospy.logerr("Requested an invalid mode.")
            return SetEddiebotModeResponse(False)
        return SetEddiebotModeResponse(True)

    def _robot_run_passive(self):
        """
        Set robot into passive run mode
        """
        rospy.loginfo("Setting eddiebot to passive mode.")
        #setting all the digital outputs to 0
        self._set_digital_outputs([0, 0, 0])
        self.robot.passive()

    def _robot_reboot(self):
        """
        Perform a soft-reset of the Create
        """
        msg ="Soft-rebooting eddiebot to passive mode."
#        rospy.logdebug(msg)
#        self._diagnostics.node_status(msg,"warn")
#        self._set_digital_outputs([0, 0, 0])
#        self.robot.soft_reset()
#       time.sleep(2.0)

    def _robot_run_safe(self):
        """
        Set robot into safe run mode
        """
        rospy.loginfo("Setting eddiebot to safe mode.")
        self.robot.safe = True
        #self.robot.control()
        b1 = (self.sensor_state.user_digital_inputs & 2)/2
        b2 = (self.sensor_state.user_digital_inputs & 4)/4
        self._set_digital_outputs([1, b1, b2])



    def _robot_run_full(self):
        """
        Set robot into full run mode
        """
#        rospy.loginfo("Setting eddiebot to full mode.")
        self.robot.safe = False
#        self.robot.control()
#        b1 = (self.sensor_state.user_digital_inputs & 2)/2
#        b2 = (self.sensor_state.user_digital_inputs & 4)/4
#        self._set_digital_outputs([1, b1, b2])



    def _set_digital_outputs(self, outputs):
        assert len(outputs) == 3, 'Expecting 3 output states.'
        byte = 0
        for output, state in enumerate(outputs):
            byte += (2 ** output) * int(state)
        self.robot.set_digital_outputs(byte)
        self.sensor_state.user_digital_outputs = byte

    def set_digital_outputs(self,req):
        if not self.robot.sci:
            raise Exception("Robot not connected, SCI not available")
            
        outputs = [req.digital_out_0,req.digital_out_1, req.digital_out_2]
        self._set_digital_outputs(outputs)
        return SetDigitalOutputsResponse(True)

    def sense(self, sensor_state):
         self.sensor_handler.get_all(sensor_state)
 #       if self._gyro:
 #           self._gyro.update_calibration(sensor_state)

    def spin(self):

        # state
        s = self.sensor_state
        odom = Odometry(header=rospy.Header(frame_id=self.odom_frame), child_frame_id=self.base_frame)
        js = JointState(name = ["left_wheel_joint", "right_wheel_joint", "front_castor_joint", "back_castor_joint"],
                        position=[0,0,0,0], velocity=[0,0,0,0], effort=[0,0,0,0])

        r = rospy.Rate(self.update_rate)
        last_cmd_vel = 0, 0
        last_cmd_vel_time = rospy.get_rostime()
        last_js_time = rospy.Time(0)
        # We set the retry count to 0 initially to make sure that only 
        # if we received at least one sensor package, we are robust 
        # agains a few sensor read failures. For some strange reason, 
        # sensor read failures can occur when switching to full mode 
        # on the Roomba. 
        sensor_read_retry_count = 0 


        while not rospy.is_shutdown():
            last_time = s.header.stamp
            curr_time = rospy.get_rostime()

            # SENSE/COMPUTE STATE
            try:
                self.sense(s)
                transform = self.compute_odom(s, last_time, odom)
                # Future-date the joint states so that we don't have
                # to publish as frequently.
                js.header.stamp = curr_time + rospy.Duration(1)
            except select.error:
                # packet read can get interrupted, restart loop to
                # check for exit conditions
                continue

            except DriverError: 
                if sensor_read_retry_count > 0: 
                    rospy.logwarn('Failed to read sensor package. %d retries left.' % sensor_read_retry_count) 
                    sensor_read_retry_count -= 1 
                    continue 
                else: 
                    raise 
            sensor_read_retry_count = self._SENSOR_READ_RETRY_COUNT 

            # Reboot Create if we detect that charging is necessary.
#            if s.charging_sources_available > 0 and \
#                   s.oi_mode == 1 and \
#                   s.charging_state in [0, 5] and \
#                   s.charge < 0.93*s.capacity:
#                rospy.loginfo("going into soft-reboot and exiting driver")
#                self._robot_reboot()
#                rospy.loginfo("exiting driver")
#                break

            # Reboot Create if we detect that battery is at critical level switch to passive mode.
#            if s.charging_sources_available > 0 and \
#                   s.oi_mode == 3 and \
#                   s.charging_state in [0, 5] and \
#                   s.charge < 0.15*s.capacity:
#                rospy.loginfo("going into soft-reboot and exiting driver")
#                self._robot_reboot()
#                rospy.loginfo("exiting driver")
#                break

            # PUBLISH STATE
            if transform is not None:
                self.sensor_state_pub.publish(s)
                self.odom_pub.publish(odom)
                if self.publish_tf:
                    self.publish_odometry_transform(odom)
            # 1hz, future-dated joint state
#            if curr_time > last_js_time + rospy.Duration(1):
#                self.joint_states_pub.publish(js)
#                last_js_time = curr_time
#            self._diagnostics.publish(s, self._gyro)
#            if self._gyro:
#                self._gyro.publish(s, last_time)

            # ACT
            if self.req_cmd_vel is not None:
                # check for velocity command and set the robot into full mode if not plugged in
#                if s.oi_mode != self.operate_mode and s.charging_sources_available != 1:
#                    if self.operate_mode == 2:
#                        self._robot_run_safe()
#                    else:
#                        self._robot_run_full()

                # check for bumper contact and limit drive command
                req_cmd_vel = self.check_bumpers(s, self.req_cmd_vel)

                # Set to None so we know it's a new command
                self.req_cmd_vel = None
                # reset time for timeout
                last_cmd_vel_time = last_time

            else:
                #zero commands on timeout
#                if last_time - last_cmd_vel_time > self.cmd_vel_timeout:
#                    last_cmd_vel = 0,0
                # double check bumpers
                req_cmd_vel = self.check_bumpers(s, last_cmd_vel)

#            if(req_cmd_vel[0] != last_cmd_vel[0] or req_cmd_vel[1] != last_cmd_vel[1]): # Added if only drive command are different then issue new command
            # send command
            if self.emergency_activated:
                #self.drive_cmd(0, 0)
                self.robot.sci.getSer().write('STOP 0' + chr(13))
            else:
                self.drive_cmd(*req_cmd_vel)
            # record command
            last_cmd_vel = req_cmd_vel

            r.sleep()

    def check_bumpers(self, s, cmd_vel):
        # Safety: disallow forward motion if bumpers or wheeldrops
        # are activated.
        # TODO: check bumps_wheeldrops flags more thoroughly, and disable
        # all motion (not just forward motion) when wheeldrops are activated
        #forward = (cmd_vel[0] + cmd_vel[1]) > 0
        if self.stop_motors_on_bump and s.bumps_wheeldrops:# and forward:
            #return (0,0)
            return cmd_vel
        else:
            return cmd_vel
#        return cmd_vel

    def compute_odom(self, sensor_state, last_time, odom):
        """
        Compute current odometry.  Updates odom instance and returns tf
        transform. compute_odom() does not set frame ids or covariances in
        Odometry instance.  It will only set stamp, pose, and twist.

        @param sensor_state: Current sensor reading
        @type  sensor_state: EddiebotSensorState
        @param last_time: time of last sensor reading
        @type  last_time: rospy.Time
        @param odom: Odometry instance to update.
        @type  odom: nav_msgs.msg.Odometry

        @return: transform
        @rtype: ( (float, float, float), (float, float, float, float) )
        """
        # based on otl_roomba by OTL <t.ogura@gmail.com>

        current_time = sensor_state.header.stamp
        dt = (current_time - last_time).to_sec()

        # On startup, Create can report junk readings
        #if abs(sensor_state.distance) > 1.0 or abs(sensor_state.angle) > 1.0:
        #    raise Exception("Distance, angle displacement too big, invalid readings from robot. Distance: %.2f, Angle: %.2f" % (sensor_state.distance, sensor_state.angle))
        
        angle_rad = math.radians(-sensor_state.angle)
        
        #sensor_state.distance = self.dist_cmd
        #sensor_state.angle = self.angle_cmd
        if self.last_angle == self.UNDEFINED:
            self.last_angle = angle_rad
            self.last_dist = sensor_state.distance
            #with open('/home/robco/ros/pos2d_x', 'r') as f:
            #    self._pos2d.x = float(f.read())
            #with open('/home/robco/ros/pos2d_y', 'r') as f:
            #    self._pos2d.y = float(f.read())
            #with open('/home/robco/ros/pos2d_theta', 'r') as f:
            #    self._pos2d.theta = float(f.read())
            return None
            
        # this is really delta_distance, delta_angle
        d  = (sensor_state.distance - self.last_dist) #* self.odom_linear_scale_correction #correction factor from calibration
        angle = (angle_rad - self.last_angle) #* self.odom_angular_scale_correction #correction factor from calibration
        self.last_angle = angle_rad
        self.last_dist = sensor_state.distance
        x = cos(angle) * d
        y = -sin(angle) * d
        #print(sensor_state.distance)
        last_angle = self._pos2d.theta
        self._pos2d.x += cos(last_angle)*x - sin(last_angle)*y
        self._pos2d.y += sin(last_angle)*x + cos(last_angle)*y
        self._pos2d.theta += angle
        #with open('/home/robco/ros/pos2d_x', 'w') as f:
        #    f.write(str(self._pos2d.x))
        #with open('/home/robco/ros/pos2d_y', 'w') as f:
        #    f.write(str(self._pos2d.y))
        #with open('/home/robco/ros/pos2d_theta', 'w') as f:
        #    f.write(str(self._pos2d.theta))
        # Eddiebot quaternion from yaw. simplified version of tf.transformations.quaternion_about_axis
        odom_quat = (0., 0., sin(self._pos2d.theta/2.), cos(self._pos2d.theta/2.))

        # construct the transform
        transform = (self._pos2d.x, self._pos2d.y, 0.), odom_quat

        # update the odometry state
        odom.header.stamp = current_time
        odom.pose.pose   = Pose(Point(self._pos2d.x, self._pos2d.y, 0.), Quaternion(*odom_quat))
        odom.twist.twist = Twist(Vector3(d/dt, 0, 0), Vector3(0, 0, angle/dt))
        if sensor_state.requested_right_velocity == 0 and \
               sensor_state.requested_left_velocity == 0 and \
               sensor_state.distance == 0:
            odom.pose.covariance = ODOM_POSE_COVARIANCE2
            odom.twist.covariance = ODOM_TWIST_COVARIANCE2
        else:
            odom.pose.covariance = ODOM_POSE_COVARIANCE
            odom.twist.covariance = ODOM_TWIST_COVARIANCE

        # return the transform
        return transform

    def publish_odometry_transform(self, odometry):
        self.transform_broadcaster.sendTransform(
            (odometry.pose.pose.position.x, odometry.pose.pose.position.y, odometry.pose.pose.position.z),
            (odometry.pose.pose.orientation.x, odometry.pose.pose.orientation.y, odometry.pose.pose.orientation.z,
             odometry.pose.pose.orientation.w),
             odometry.header.stamp, odometry.child_frame_id, odometry.header.frame_id)

def connected_file():
    return os.path.join(roslib.rosenv.get_ros_home(), 'eddiebot-connected')

def eddiebot_main(argv):
    c = EddiebotNode()
    while not rospy.is_shutdown():
        #try:
            # This sleep throttles reconnecting of the driver.  It
            # appears that pyserial does not properly release the file
            # descriptor for the USB port in the event that the Create is
            # unplugged from the laptop.  This file desecriptor prevents
            # the create from reassociating with the same USB port when it
            # is plugged back in.  The solution, for now, is to quickly
            # exit the driver and let roslaunch respawn the driver until
            # reconnection occurs.  However, it order to not do bad things
            # to the Create bootloader, and also to keep relaunching at a
            # minimum, we have a 3-second sleep.
            # time.sleep(3.0)
            
            c.start()
            
            c.robot.sci.getSer().write('ACC {}'.format(c.max_acc) + chr(13))
            c.last_time = time.time()
            
            c.spin()

        #except Exception as ex:
        #    msg = "eddiebot_main error: [%s]"%(ex)
        #    c._diagnostics.node_status(msg,"error")
        #    rospy.logerr(msg)

        #finally:
        #    # Driver no longer connected, delete flag from disk
        #    try:
        #        os.remove(connected_file())
        #    except Exception: pass


if __name__ == '__main__':
    eddiebot_main(sys.argv)
