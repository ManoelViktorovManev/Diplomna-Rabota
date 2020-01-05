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
import spawning_actors

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')



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
    # funkciyata otdolu moje da se naprawi s nyakakwo opredeleno wreme
    def is_diferent_date(self):
        if (int(time.strftime("%S"))>=(int(self.list_of_pushed[2])+2) or 
        (time.strftime("%M")!=self.list_of_pushed[1] and int(time.strftime("%H"))!=(int(self.list_of_pushed[0])+2))):
            return True
        return False

class World:
    def __init__(self,client,world,blueprint_library,list_of_actors):
        self.client=client
        self.client.set_timeout(10)
        self.world=world
        self.blueprint_library=blueprint_library
        self.list_of_actors=list_of_actors


def parse(surface,image):
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    image_surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    surface.blit(image_surface, (0, 0))

def safing_photos(camattach):
    camattach.listen(lambda image: image.save_to_disk('output/%06d.png' % image.frame))

def print_info():
    print()
    print("Welcome to my CARLA controler")
    print()
    print("Use one of the controlers to move:")
    print("     Left analog - up and down       :       move forward and back")
    print("     Right analog - left and right   :       move left and right")
    print()
    print("From the right buttons:")
    print("     Down button     :       exit")
    print("     Left button     :       turn on/off autopilot")
    print()

spawning_actors.main()
list_of_actors=spawning_actors.listofActors()

client=carla.Client('localhost',2000)
client.set_timeout(10.0)
world=client.get_world()
blueprint_library = world.get_blueprint_library()
car=blueprint_library.find("vehicle.tesla.model3")
transform = carla.Transform(carla.Location(x=30, y=0, z=3), carla.Rotation(yaw=180))
vehicle = world.spawn_actor(car, transform)
# x=30
# y =0
# z=3


cam=blueprint_library.find("sensor.camera.rgb")
transform_cam=carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15))
camattach=world.spawn_actor(cam,transform_cam,attach_to=vehicle)
camattach2=world.spawn_actor(cam,transform_cam,attach_to=vehicle)

pygame.init()
display = pygame.display.set_mode(
    (800, 600),
    pygame.HWSURFACE | pygame.DOUBLEBUF)
pygame.display.set_caption("Kontrol na kolata ot PS4")
weather=carla.WeatherParameters()
weather.cloudiness=90
world.set_weather(weather)
pygame.joystick.init()
listofall=[pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
a=listofall[0]
a.init()
camattach.listen(lambda image: parse(display,image))
# safing_photos(camattach2)
control_car=carla.VehicleControl()
autopilot=False


list_of_actors.append(vehicle)
list_of_actors.append(camattach)
list_of_actors.append(camattach2)

button_autopilot=Pushing_Buton([])
print_info()
while True:
    
    pygame.event.get()
    if a.get_button(0)==True:
        spawning_actors.destroyingActors(list_of_actors)
        pygame.joystick.quit()
        pygame.display.quit()
        break
    if a.get_button(1)==True:
        if len(button_autopilot.get_list())!=0:
            if (button_autopilot.is_diferent_date()):
                spawning_actors.deleteAllelements(button_autopilot.get_list())
        if len(button_autopilot.get_list())==0:       
            button_autopilot.add_new_date_now()
            button_autopilot.set_state_of_pushed_button(True)

        if button_autopilot.get_button() is True:
            if autopilot is False:
                vehicle.apply_control(carla.VehicleControl())
                vehicle.set_autopilot(True)
                autopilot=True
                print("AUTOPILOT ON")
            elif autopilot is True:
                autopilot=False
                vehicle.set_autopilot(False)
                vehicle.apply_control(control_car)
                print("AUTOPILOT OFF")
            button_autopilot.set_state_of_pushed_button(False)
    if autopilot is False:
        if a.get_axis(1)>0:
            control_car.reverse=True
        elif a.get_axis(1)<0:
            control_car.reverse=False
        if a.get_axis(2)>0 or a.get_axis(5)>0:
            if a.get_axis(2)>0:
                control_car.brake=a.get_axis(2)
            else:
                control_car.brake=a.get_axis(5)
        elif a.get_axis(2)<=0 or a.get_axis(5)<=0:
            control_car.brake=0.0
        control_car.throttle=math.fabs(a.get_axis(1))
        control_car.steer=a.get_axis(3)
        vehicle.apply_control(control_car)
    pygame.display.flip()

    # speed print 
    # speed=vehicle.get_velocity()
    # print (3.6 * inmath.sqrt(speed.x**2 + speed.y**2 + speed.z**2))
    # time.sleep(0.1)