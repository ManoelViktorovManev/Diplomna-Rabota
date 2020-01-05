#!/usr/bin/env python

from __future__ import print_function
import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import pygame
import time
import math
import random


list_of_actors=[]

def destroyingActors(list):
    for i in list:
        i.destroy()

def chose_element_from_list(list):
    chosen=random.randrange(0,len(list))
    return list[chosen]
def is_existing_point(list,point):
    for i in list:
        if i is point:
            return True
    return False
def deleteAllelements(mylist):
    del mylist[0:len(mylist)]
def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0) # seconds
    world=client.get_world()
    blueprint_lib=world.get_blueprint_library()

    list_ofCars=blueprint_lib.filter('vehicle.*')
    list_of_pedestrians=blueprint_lib.filter('walker.pedestrian.*')
    list_of_used_points=[]
    list_of_spawn_points=world.get_map().get_spawn_points()

    for _ in range(5):
        for _ in range(5):
            car=chose_element_from_list(list_ofCars)
            transform=chose_element_from_list(list_of_spawn_points)
            while (is_existing_point(list_of_used_points,transform)==True):
                transform=chose_element_from_list(list_of_spawn_points)
            list_of_used_points.append(transform)
            vehicle=world.spawn_actor(car,transform)
            list_of_actors.append(vehicle)
            vehicle.set_autopilot(True)
        # deleteAllelements(list_of_used_points)
        # time.sleep(0.5)
    print ("Number of spawned actors: "+str(len(list_of_actors)))

def listofActors():
    return list_of_actors
