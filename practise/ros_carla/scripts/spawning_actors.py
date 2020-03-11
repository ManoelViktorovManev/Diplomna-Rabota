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
import random

list_of_actors=[]

# Function that destroys the elements in the list => that destroys the actors in the list. 
def destroyingActors(list):
    for i in list:
        i.destroy()
        
# Function that gives a random element from the list.
def chose_element_from_list(list):
    chosen=random.randrange(0,len(list))
    return list[chosen]

# Function that checks if the point is in the list.
def is_existing_point(list,point):
    for i in list:
        if i is point:
            return True
    return False

def deleteAllelements(mylist):
    del mylist[0:len(mylist)]
    # ???
# Function that returns list of all actors created from me in the simulator. 
def listofActors():
    return list_of_actors

# Main function. Here are created 25 new vehicle and are set in autopilot mode.
def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0) 
    world=client.get_world()
    blueprint_lib=world.get_blueprint_library()

    list_ofCars=blueprint_lib.filter('vehicle.*')
    list_of_used_points=[]
    list_of_spawn_points=world.get_map().get_spawn_points()

    for _ in range(25):
        car=chose_element_from_list(list_ofCars)
        transform=chose_element_from_list(list_of_spawn_points)
        while (is_existing_point(list_of_used_points,transform)==True):
            transform=chose_element_from_list(list_of_spawn_points)
        list_of_used_points.append(transform)
        vehicle=world.spawn_actor(car,transform)
        list_of_actors.append(vehicle)
        vehicle.set_autopilot(True)
            
    print ("Number of spawned actors: "+str(len(list_of_actors)))
