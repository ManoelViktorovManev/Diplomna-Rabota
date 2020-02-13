#!/usr/bin/env python
from __future__ import print_function
import glob
import os
import sys

# try:
#     sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
#         sys.version_info.major,
#         sys.version_info.minor,
#         'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
# except IndexError:
#     pass

import carla

import rospy
from msg_folder.msg import MyRosMsg
import pygame

import spawning_actors
import time

class Pushing_Buton:
    def __init__(self,list_of_pushed=[],state_of_pushed_button=False):
        self.date_of_pushed_button=list_of_pushed
        self.state_of_pushed_button=state_of_pushed_button
        self.date_of_released_button=[]
    def set_state_of_pushed_button(self,bool_set):
        self.state_of_pushed_button=bool_set
    def set_list_of_pushed(self,list):
        self.date_of_pushed_button=list
        self.date_of_released_button=list
    def set_list_of_released(self,list):
        self.date_of_released_button=list
    def get_button(self):
        return self.state_of_pushed_button
    def get_list(self):
        return self.date_of_pushed_button
    def get_pushed_time(self):
        return self.date_of_released_button
    def add_new_date_now(self):
        self.date_of_pushed_button.append(time.strftime("%H"))
        self.date_of_pushed_button.append(time.strftime("%M"))
        self.date_of_pushed_button.append(time.strftime("%S"))
    def add_released_time(self):
        self.date_of_released_button.append(time.strftime("%H"))
        self.date_of_released_button.append(time.strftime("%M"))

client = carla.Client('localhost', 2000)
client.set_timeout(10.0) 
world=client.get_world()
button_autopilot=Pushing_Buton()
actor_list=world.get_actors()
autopilot=False
control_car=carla.VehicleControl()
def callback(data):
    global autopilot
    global button_autopilot
    vehicle=actor_list.find(data.vehicle_id)
    if data.is_exit==True:
        print("The console stoped :)")
        rospy.signal_shutdown("exit")

    if data.is_autopilot==False:
        if len(button_autopilot.get_pushed_time()) == 0 and len(button_autopilot.get_list()) != 0:
            button_autopilot.add_released_time()
            
    if data.is_autopilot==True:
        if len(button_autopilot.get_list())!=0 and len(button_autopilot.get_pushed_time())!=0:
                button_autopilot.set_list_of_pushed([])
                button_autopilot.set_list_of_released([])
        if len(button_autopilot.get_list())==0:       
            button_autopilot.add_new_date_now()
            button_autopilot.set_state_of_pushed_button(True)

        if button_autopilot.get_button() is True:
            if autopilot is False:
                vehicle.set_autopilot(True)
                autopilot=True
                print("AUTOPILOT ON")
            elif autopilot is True:
                autopilot=False
                vehicle.set_autopilot(False)
                print("AUTOPILOT OFF")
            button_autopilot.set_state_of_pushed_button(False)
    if autopilot == False:
        control_car.reverse=data.is_reverse
        control_car.brake=data.breaks
        control_car.throttle=data.throttle
        control_car.steer=data.steer
        vehicle.apply_control(control_car)
def listener():
 
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("controler_info", MyRosMsg, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
