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

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')


def parse(surface,image):
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    image_surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    surface.blit(image_surface, (0, 0))
    

client=carla.Client('localhost',2000)
client.set_timeout(10.0)
world=client.get_world()
blueprint_library = world.get_blueprint_library()
car=blueprint_library.find("vehicle.audi.a2")
transform = carla.Transform(carla.Location(x=30, y=0, z=3), carla.Rotation(yaw=180))
vehicle = world.spawn_actor(car, transform)
# x=30
# y =0
# z=3


cam=blueprint_library.find("sensor.camera.rgb")
transform_cam=carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15))
camattach=world.spawn_actor(cam,transform_cam,attach_to=vehicle)

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
# camattach.listen(lambda image: image.save_to_disk('output/%06d.png' % image.frame))
control_car=carla.VehicleControl()
autopilot=False
while True:
    
    pygame.event.get()
    if a.get_button(0)==True:
        vehicle.destroy()
        camattach.destroy()
        pygame.joystick.quit()
        pygame.display.quit()
        break
    if a.get_button(1)==True:
        if autopilot is False:
            vehicle.set_autopilot(True)
            autopilot=True
        else:
            autopilot=False
            vehicle.set_autopilot(False)
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
    time.sleep(0.01)