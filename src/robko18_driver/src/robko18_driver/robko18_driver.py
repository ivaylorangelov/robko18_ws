#!/usr/bin/python

# The BSD License
# Copyright (c) 2010, Willow Garage Inc.
# Copyright (c) 2012 Tang Tiong Yew 
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
# Modified for use in ROS by Tang Tiong Yew. API names have been modified
# for consistency ROS Python coding guidelines.

from __future__ import with_statement

"""iRobot Roomba Serial control Interface (SCI) and Turtlebot Open Interface (OI).

turtlebot.py is a fork of PyRobot.

PyRobot was originally based on openinterface.py, developed by iRobot
Corporation. Many of the docstrings from openinterface.py, particularly those
which describe the specification, are also used here. Also, docstrings may
contain specification text copied directly from the Roomba SCI Spec Manual and
the Turtlebot Open Interface specification.

Since SCI is a subset of OI, PyRobot first defines the Roomba's functionality
in the Roomba class and then extends that with the Turtlebot's additional
functionality in the Turtlebot class. In addition, since OI is built on SCI the
SerialCommandInterface class is also used for OI.

"""
__author__ = "tang.tiong.yew@gmail.com (Tang Tiong Yew)"

import logging
import math
import serial
import struct
import time
import threading
import traceback
import rospy

import binascii

import sys
import os

ROBKO18_OPCODES = dict(
    start =  '', #128,
    baud =  '', #129,
    control =  '', #130,
    safe =  '', #131,
    full =  '', #132,
    power =  '', #133,
    spot =  '', #134,
    clean =  '', #135,
    max =  '', #136,
    drive =  'GOSPD', #137,
#    drive =  'GO', #137,
    motors =  'SV', #138,
    leds =  '', #139,
    song =  '', #140,
    play =  '', #141,
    sensors =  '', #142,
    force_seeking_dock =  '', #143,

    
    soft_reset =  '', #7,  # Where is this documented?
    low_side_drivers =  '', #138,
    play_song =  '', #141,
    pwm_low_side_drivers =  '', #144,
    acc = 'ACC',
    direct_drive =  'GOSPD', #145,
  #  direct_drive =  'GO', #145,
    digital_outputs =  '', #147,
    stream =  '', #148,
    query_list =  '', #149,
    pause_resume_stream =  '', #150,
    send_ir =  '', #151,
    script =  '', #152,
    play_script =  '', #153,
    show_script =  '', #154,
    wait_time =  '', #155,
    wait_distance =  '', #156,
    wait_angle =  '', #157,
    wait_event =  '', #158,
    )

REMOTE_OPCODES = {
    # Remote control.
    129: 'left',
    130: 'forward',
    131: 'right',
    132: 'spot',
    133: 'max',
    134: 'small',
    135: 'medium',
    136: 'large',
    136: 'clean',
    137: 'pause',
    138: 'power',
    139: 'arc-left',
    140: 'arc-right',
    141: 'drive-stop',
    # Scheduling remote.
    142: 'send-all',
    143: 'seek-dock',
    # Home base.
    240: 'reserved',
    242: 'force-field',
    244: 'green-buoy',
    246: 'green-buoy-and-force-field',
    248: 'red-buoy',
    250: 'red-buoy-and-force-field',
    252: 'red-buoy-and-green-buoy',
    254: 'red-buoy-and-green-buoy-and-force-field',
    255: 'none',
    }

BAUD_RATES = (  # In bits per second.
    300,
    600,
    1200,
    2400,
    4800,
    9600,
    14400,
    19200,
    28800,
    38400,
    57600,  # Default.
    115200)

CHARGING_STATES = (
    'not-charging',
    'charging-recovery',
    'charging',
    'trickle-charging',
    'waiting',
    'charging-error')

OI_MODES = (
    'off',
    'passive',
    'safe',
    'full')

# Various state flag masks
WHEEL_DROP_CASTER = 0x10
WHEEL_DROP_LEFT = 0x08
WHEEL_DROP_RIGHT = 0x04
BUMP_LEFT = 0x02
BUMP_RIGHT = 0x01

OVERCURRENTS_DRIVE_LEFT = 0x10
OVERCURRENTS_DRIVE_RIGHT = 0x08
OVERCURRENTS_MAIN_BRUSH = 0x04
OVERCURRENTS_VACUUM = 0x02
OVERCURRENTS_SIDE_BRUSH = 0x01

BUTTON_POWER = 0x08
BUTTON_SPOT = 0x04
BUTTON_CLEAN = 0x02
BUTTON_MAX = 0x01

SENSOR_GROUP_PACKET_LENGTHS = {
    0: 26,
    1: 10,
    2: 6,
    3: 10,
    4: 14,
    5: 12,
    6: 52,
    100: 80 }

# drive constants.
RADIUS_TURN_IN_PLACE_CW = -2000 # TODO: Radius is unknown without create robot.
RADIUS_TURN_IN_PLACE_CCW = 2000
RADIUS_STRAIGHT = 32768
RADIUS_MAX = 2000

VELOCITY_MAX = 127  # positions per second
VELOCITY_SLOW = int(VELOCITY_MAX * 0.10)
VELOCITY_FAST = int(VELOCITY_MAX * 0.66)

PWM_RATIO_FORWARD_MAX = 127
PWM_RATIO_BACKWARD_MAX = -127

MAX_WHEEL_SPEED = 50
WHEEL_SEPARATION = 390  # mm

SERIAL_TIMEOUT = 2  # Number of seconds to wait for reads. 2 is generous.
START_DELAY = 5  # Time it takes the Roomba/Turtlebot to boot.

BAUDRATE = 115200

assert struct.calcsize('H') == 2, 'Expecting 2-byte shorts.'

class DriverError(Exception):
  pass

class SharpIR(object):
  def __init__(self):
    self.last_cm = 99
    self.ticks = 0
    
  def get_cm(self, adc):
    if adc == 0:
      return 99
    cm = 27.728 * ((adc * 0.001221001221) ** -1.2045)  
    if cm > 10 and cm < 80:
      self.ticks = 0
      self.last_cm = cm
    else:
      self.ticks = self.ticks + 1
    if self.ticks > 10:
        self.last_cm = 99
    return self.last_cm

class SerialCommandInterface(object):

  """A higher-level wrapper around PySerial specifically designed for use with
  Parallax Eddie Propeller Board.

  """

  def __init__(self, tty, baudrate):
    self.ser = serial.Serial(
        port='/dev/ttyUSB0',
        baudrate=BAUDRATE,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=SERIAL_TIMEOUT
    )
      
    #self.ser.open()
#    self.wake()
    self.opcodes = {}

    #TODO: kwc all locking code should be outside of the driver. Instead,
    #could place a lock object in Roomba and let people "with" it
    self.lock = threading.RLock()
    
  def wake(self):
    """wake up robot."""
    print "wake"
#    self.ser.setRTS(0)
#    time.sleep(0.1)
#    self.ser.setRTS(1)
#    time.sleep(0.75)  # Technically it should wake after 500ms.
#    for i in range(3):
#        self.ser.setRTS(0)
#        time.sleep(0.25)
#        self.ser.setRTS(1)
#        time.sleep(0.25) 

  def add_opcodes(self, opcodes):
    """Add available opcodes to the SCI."""
    self.opcodes.update(opcodes)

  def send(self, bytes):
    """send a string of bytes to the robot."""
    print bytes
    with self.lock:
      self.ser.write(bytes)

  #TODO: kwc the locking should be done at a higher level
  def read(self, num_bytes):
    """Read a string of 'num_bytes' bytes from the robot."""
#    logging.debug('Attempting to read %d bytes from SCI port.' % num_bytes)
#    with self.lock:
    data = self.ser.read(num_bytes)
#    logging.debug('Read %d bytes from SCI port.' % len(data))
#    if not data:
#      raise DriverError('Error reading from SCI port. No data.')
#    if len(data) != num_bytes:
#      raise DriverError('Error reading from SCI port. Wrong data length.')
    return data

  def flush_input(self):
    """Flush input buffer, discarding all its contents."""
    logging.debug('Flushing serial input buffer.')
    self.ser.flushInput()
    
  def inWaiting(self):
    """ InWaiting Called """
    logging.debug('Called inWaiting')
    self.ser.inWaiting()

  def __getattr__(self, name):
    """Robko18 methods for opcodes on the fly.

    Each opcode method sends the opcode optionally followed by a string of
    parameter.

    """
    #TODO: kwc do static initialization instead
    if name in self.opcodes:
      def send_opcode(input_string):
        logging.debug('sending opcode %s.' % name)
        self.send(self.opcodes[name] + ' ' + input_string)
      return send_opcode
    raise AttributeError

  def getSer(self):
    return self.ser
    
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

class Robko18():
  METERS_PER_TICK = 0.00332485223
  MILLIVOLTS_PER_VOLT = 3.918457 # magic constant

  """Represents a Robko18 robot."""

  def __init__(self):
    """
    @param sensor_class: Sensor class to use for fetching and decoding sensor data.
    """
    logging.basicConfig(filename='robko18_driver.log', level=logging.INFO)
    self.tty = None
    self.sci = None
    self.safe = True
    
    self.infrared_sensor_1 = SharpIR()
    self.infrared_sensor_2 = SharpIR()
    self.infrared_sensor_3 = SharpIR()
    self.infrared1 = 0
    self.infrared2 = 0
    self.infrared3 = 0
    self.sonar1 = 0
    self.sonar2 = 0
    self.heading = 0
    self.voltage = 0
    self.dist_l = 0
    self.dist_r = 0
    
    self.start()
    
  def start(self, tty='/dev/ttyUSB0', baudrate=115200):
    self.tty = tty
    self.sci = SerialCommandInterface(tty, baudrate)
    self.sci.add_opcodes(ROBKO18_OPCODES)
    
    #------------------- Roomba methods -----------------------------
  def change_baud_rate(self, baud_rate):
    """Sets the baud rate in bits per second (bps) at which SCI commands and
    data are sent according to the baud code sent in the data byte.

    The default baud rate at power up is 57600 bps. (See Serial Port Settings,
    above.) Once the baud rate is changed, it will persist until Roomba is
    power cycled by removing the battery (or until the battery voltage falls
    below the minimum requir''' ed for processor operation). You must wait 100ms
    after sending this command before sending additional commands at the new
    baud rate. The SCI must be in passive, safe, or full mode to accept this
    command. This command puts the SCI in passive mode.

    """
    if baud_rate not in BAUD_RATES:
      raise DriverError('Invalid baud rate specified.')
    self.sci.baud(baud_rate)
    self.sci = SerialCommandInterface(self.tty, baud_rate)
  
  def direct_drive(self, velocity_left, velocity_right):
#    print("direct_drive(self, velocity_left, velocity_right)")
#    print("velocity_left: " + str(int(velocity_left)))
#    print("velocity_right: " + str(int(velocity_right)))
    # Mask integers to 2 bytes.
    vl = int(velocity_left) & 0xffff
    vr = int(velocity_right) & 0xffff
    
    parameters = binascii.hexlify(struct.pack('>2H', vr, vl))
    
    parameters_output = ''
    
    for i in range(len(parameters)):
             parameters_output += parameters[i]
             if(i == 3):
                 parameters_output += ' '
        
    self.sci.direct_drive(parameters_output + chr(13))
    
  def command_joints(self, pan_degree, tilt_degree):
    pan_degree = int(pan_degree)
    tilt_degree = int(tilt_degree)
    parameters = binascii.hexlify(struct.pack('>H', pan_degree))
    self.sci.motors(' 4 ' + parameters + chr(13)) # Pan Servo
    
    #parameters = binascii.hexlify(struct.pack('>H', tilt_degree))
    #self.sci.motors(' 5 ' + parameters + chr(13)) # Pan Servo

  def stop(self):
    """Set velocity and radius to 0 to stop movement."""
    self.direct_drive(0, 0)
        
  def sxtn(self, x, bits):
     h = 1<<(bits-1)
     m = (1<<bits)-1
     return ((x+h) & m) - h

  def get_ints_from_command(self, command):
      self.sci.getSer().flushInput()
      self.sci.getSer().flushOutput()
      self.sci.getSer().write(command + chr(13))
      time.sleep(0.07)
    
    
    #ret = self.sci.command(command)
    #try:
      ret = ""
      while self.sci.getSer().inWaiting() > 0:
        c = self.sci.getSer().read(1)
        if(c == '\r'):
          continue
          self.sci.getSer().flushInput()
          self.sci.getSer().flushOutput()
          break
        else:
          ret += c
      
      if ret and ret != "":
        data = [int(x, 16) for x in ret.split()]
        return data
    #except:
    #  return ""
      return ""
 
  def sensors(self):
    """This is a sensor command to emulate SCI sensor sending"""
    self.start()
    
    adc = self.get_ints_from_command('ADC')
    if adc != "":
      try:
        self.voltage = int(adc[7] * self.MILLIVOLTS_PER_VOLT)
        self.infrared1 = int(self.infrared_sensor_1.get_cm(adc[0]))
        self.infrared2 = int(self.infrared_sensor_2.get_cm(adc[1]))
        self.infrared3 = int(self.infrared_sensor_3.get_cm(adc[2]))
      except:
        pass
      
    ping = self.get_ints_from_command('PING')
    if ping != "":
      try:
        if ping[0] > 0:
          self.sonar1 = int(ping[0]/10)
        if ping[1] > 0:
          self.sonar2 = int(ping[1]/10)
      except:
        pass
      
    head = self.get_ints_from_command('HEAD')
    if head != "":
      try:
        head = self.sxtn(head[0], 12)
        head = (7200 + head) % 360
        self.heading = head
      except:
        pass
    
    dist = self.get_ints_from_command('DIST')
    if dist != "":
      try:
        self.dist_l = self.sxtn(dist[0], 32)*self.METERS_PER_TICK
        self.dist_r = self.sxtn(dist[1], 32)*self.METERS_PER_TICK
      except:
        pass
            
    d = (self.dist_l + self.dist_r)/2.0
    
    return (0,#1
            0,#2 wall
            0,#3 cliff_left
            0,#4 cliff_front_left
            0,#5 cliff_front_right
            0,#6 cliff_right
            0,#7
            0,#8
            0,#9
            0,#10
            0,#11
            0,#12
            d,#13
            self.heading,#14
            0,#15
            self.voltage,#16
            0,#17
            0,#18
            0,#19
            0,#20 adc_battery,
            self.sonar1,#21 wall_signal
            self.sonar2,#22 cliff_left_signal
            self.infrared1,#23 cliff_front_left_signal
            self.infrared2,#24 cliff_front_right_signal
            self.infrared3,#25 cliff_right_signal
            0,#26
            0,#27
            0,#28
            0,#29
            0,#30
            0,#31
            0,#32
            0,#33
            0,#34
            0,#35
            0,#36
            0,#37
            0,#38
            0,#39 light_bumper
            0,#40 light_bump_left
            0,#41 light_bump_front_left
            0,#42 light_bump_center_left
            0,#43 light_bump_center_right
            0,#44 light_bump_front_right
            0,#45 light_bump_right
            0,#46
            0,#47
            0,#48
            0,#49
            0,#50
            0,#51
            0)#52
#   (ping_left, ping_right, adc_left, adc_right, adc_center, adc_battery)

