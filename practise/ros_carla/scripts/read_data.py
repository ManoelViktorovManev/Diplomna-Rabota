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
    def __init__(self,list_of_pushed,state_of_pushed_button=False):
        self.list_of_pushed=list_of_pushed
        self.state_of_pushed_button=state_of_pushed_button
    def set_state_of_pushed_button(self,bool_set):
        self.state_of_pushed_button=bool_set
    def set_list_of_pushed(slef,list):
        self.list_of_pushed=list
    def get_button(self):
        return self.state_of_pushed_button
    def get_list(self):
        return self.list_of_pushed
    def add_new_date_now(self):
        self.list_of_pushed.append(time.strftime("%H"))
        self.list_of_pushed.append(time.strftime("%M"))
        self.list_of_pushed.append(time.strftime("%S"))
    def is_diferent_date(self):
        if (int(time.strftime("%S"))>=(int(self.list_of_pushed[2])+2) or 
        (time.strftime("%M")!=self.list_of_pushed[1] and int(time.strftime("%H"))!=(int(self.list_of_pushed[0])+2))):
            return True
        return False

client = carla.Client('localhost', 2000)
client.set_timeout(10.0) 
world=client.get_world()
button_autopilot=Pushing_Buton([])
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
    if data.is_autopilot==True:
        if len(button_autopilot.get_list())!=0:
            if (button_autopilot.is_diferent_date()):
                spawning_actors.deleteAllelements(button_autopilot.get_list())
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
