#!/usr/bin/env python2
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import SensorState
import requests
import socket
import json
import argparse
import threading
import time
import subprocess

kTimeout = 0.1
socket.setdefaulttimeout(kTimeout)
parser = argparse.ArgumentParser()
parser.add_argument("robot_name", help="Robot name [robot0|robot1|robot2]")
parser.add_argument("server_url", help="Robot url without protocol")
parser.add_argument("--http_port", type=int, default=80, help="Robot server HTTP port")
opt = parser.parse_args()

url = opt.server_url
port = 9001

def get_laptop_state():
  out = subprocess.Popen('upower -i /org/freedesktop/UPower/devices/battery_BAT0 | grep -E "percentage|state"',
                         shell=True,
                         stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT)
  stdout, stderr = out.communicate()
  lines = stdout.splitlines()
  state, percentage = [e.strip().split(':')[1].strip() for e in lines]
  return state.capitalize(), percentage

def sensor_state_callback(sensor_state):
  # Runs at 20 Hz
  if sensor_state.header.seq % 20 != 0:
    return
  is_charging = (sensor_state.charger != 0)
  battery = sensor_state.battery
  laptop_state, laptop_percentage = get_laptop_state()
  payload = {'robot' : opt.robot_name, 'is_charging' : is_charging, 'battery' : battery, 'laptop_state': laptop_state, 'laptop_percentage': laptop_percentage}
  status_url = "http://" + url + ":" + str(opt.http_port) + '/update_status'
  try:
    requests.post(url=status_url, json=payload)
  except:
    print("Failed to update robot server at " + status_url)

rospy.init_node('teleop')
teleop_pub = rospy.Publisher('/teleop_topic', Twist, queue_size=10)
use_safety_pub = rospy.Publisher('/use_safety', Bool, queue_size=10)
status_sub = rospy.Subscriber('/mobile_base/sensors/core', SensorState, sensor_state_callback)

def init_socket():
  socket.setdefaulttimeout(kTimeout)
  # create a socket object
  s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  # connection to hostname on the port.
  is_connected = False
  while not is_connected and not rospy.is_shutdown():
    try:
      s.connect((url, port))
      is_connected = True
      print("Connected socket to command server at {}".format(url))
    except socket.error:
      print("Failed to connect socket to command server at {}".format(url))
      time.sleep(0.2)

  s.send(opt.robot_name.encode('ascii'))
  return s

s = init_socket()

def make_twist(msg_json):
  delta_x = 0
  delta_theta = 0

  kForwardDelta = 0.2
  kThetaDelta = 1

  if msg_json[u'forward']:
    delta_x += kForwardDelta
  if msg_json[u'backward']:
    delta_x -= kForwardDelta
  if msg_json[u'left']:
    delta_theta += kThetaDelta
  if msg_json[u'right']:
    delta_theta -= kThetaDelta
  speed = float(msg_json[u'speed'])

  twist = Twist()
  twist.linear.x = delta_x * speed
  twist.angular.z = delta_theta * speed
  return twist

def use_safety_system(msg_json):
  return msg_json.get('use_safety_system', True)

def socket_reading_thread():
  global s
  while not rospy.is_shutdown():
    try:
      # Receive no more than 1024 bytes
      msg = s.recv(1024)
    except:
      continue
    msg_str = msg.decode('ascii').strip()
    if len(msg_str) == 0:
      # Socket connection ended.
      print("Socket connect to command server terminated!")
      s = init_socket()
      continue
    try:
      msg_json = json.loads(msg_str)
    except:
      print("Failed to parse >>{}<<".format(msg_str))
      continue
    print(msg_json)
    teleop_pub.publish(make_twist(msg_json))
    use_safety_pub.publish(use_safety_system(msg_json))


x = threading.Thread(target=socket_reading_thread)
x.start()

rospy.spin()
x.join()
s.close()
