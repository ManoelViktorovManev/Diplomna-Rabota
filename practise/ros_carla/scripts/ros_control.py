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
import rospy
import pygame
import time
import math
import random
import spawning_actors

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')
from sensor_msgs.msg import Image,CameraInfo
from msg_folder.msg import MyRosMsg

list_of_actors=[]

class World:
    def __init__(self,client,world,blueprint_library,list_of_actors):
        self.client=client
        self.client.set_timeout(10)
        self.world=world
        self.blueprint_library=blueprint_library
        self.list_of_actors=list_of_actors


def parse(surface,image,type_of_camera):
    if type_of_camera is 1:
        image.convert(carla.ColorConverter.Depth)
    elif type_of_camera is 2:
        image.convert(carla.ColorConverter.CityScapesPalette)
    # print(type(image))
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    image_surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    surface.blit(image_surface,(0, 0))

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


def checking_joystick(controler):
    while True:
        try:
            pygame.quit()
            pygame.init()
            pygame.joystick.init()
            listofall=[pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
            controler=listofall[0]
            controler.init()
        except IndexError:
            print()
            print("WARNING!!!")
            print("There is no joysticks. Please insert one.")
            time.sleep(1)
        else:
            return controler


def is_joystick_pluged(discon):
    pygame.joystick.quit()
    pygame.joystick.init()
    joystick_count = pygame.joystick.get_count()
    for i in range(joystick_count):
        joystick = pygame.joystick.Joystick(i)
        joystick.init()
    if not joystick_count: 
        if not discon:
            print ("reconnect you meat bag")
            discon = True
            checking_joystick()
    else:
        discon = False    

def publishing(pub_image,pub_camera,image,rate):
    # https://github.com/LCAS/spqrel_tools/blob/master/ros_ws/src/naoqi_bridge/naoqi_sensors_py/src/naoqi_sensors/naoqi_camera.py
    #  towa gore e decision
    # https://answers.ros.org/question/61821/rosserializationexception-while-publishing-pointcloud2-message/

    img=Image()
    infomsg=CameraInfo()
    img.header.stamp=rospy.Time.now()
    img.header.frame_id='base'
    img.height=infomsg.height=image.height
    img.width=infomsg.width=image.width
    img.encoding="rgb8"
    img.step=img.width*3*1
    img.data=image.raw_data

    # cx=infomsg.width/2.0
    # cy=infomsg.height/2.0
    # fx=fy=infomsg.width/(2.0 * math.tan(image.fov* math.pi / 360.0))
    # infomsg.K=[ fx, 0,cx,
    #                           0, fy, cy,
    #                           0, 0, 1 ]
    # infomsg.P=[ fx, 0, cx, 0,
    #                           0, fy, cy, 0,
    #                           0, 0, 1, 0 ]
    # infomsg.R=[1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
    # infomsg.D = [0,0,0,0,0]
    # infomsg.binning_x = 0
    # infomsg.binning_y = 0
    # infomsg.distortion_model = "plumb_bob"
    # infomsg.header=img.header
    pub_image.publish(img)
    # pub_camera.publish(infomsg)

rospy.init_node('ros_carla_control',anonymous=True)
pub=rospy.Publisher('/camera/rgb/image',Image,queue_size=10)
pub_camera=rospy.Publisher('/camera/rgb/camera_info',CameraInfo,queue_size=10)
pub_msg=rospy.Publisher("controler_info",MyRosMsg,queue_size=1)

controler=None
controler=checking_joystick(controler)

rate = rospy.Rate(30)
client=carla.Client('localhost',2000)
client.set_timeout(10.0)
world=client.get_world()

list_of_actors=world.get_actors().filter("vehicle*")
print (len(list_of_actors))
if len(list_of_actors)>0:
    spawning_actors.destroyingActors(list_of_actors)
list_of_actors=world.get_actors().filter("sensor*")
print (len(list_of_actors))
if len(list_of_actors)>0:
    spawning_actors.destroyingActors(list_of_actors)


spawning_actors.main()

list_of_actors=spawning_actors.listofActors()


blueprint_library = world.get_blueprint_library()
car=spawning_actors.chose_element_from_list(blueprint_library.filter("vehicle*"))

# blueprint_library.find("vehicle.tesla.model3")
# gore e za specialen tip kola
transform = carla.Transform(carla.Location(x=30, y=0, z=3), carla.Rotation(yaw=180))
vehicle = world.spawn_actor(car, transform)

# cam=spawning_actors.chose_element_from_list(blueprint_library.filter("sensor.camera*"))
cam=blueprint_library.find("sensor.camera.rgb")
transform_cam=carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15))
camattach=world.spawn_actor(cam,transform_cam,attach_to=vehicle)
type_of_camera=0
if "depth" in camattach.type_id:
    type_of_camera=1
elif "semantic_segmentation" in camattach.type_id:
    type_of_camera=2

# transform_cam=carla.Transform(carla.Location(x=3, z=1), carla.Rotation(pitch=-15))
camattach_car=world.spawn_actor(cam,transform_cam,attach_to=vehicle)

# camattach2=world.spawn_actor(cam,transform_cam,attach_to=vehicle)

display = pygame.display.set_mode(
    (800, 600),
    pygame.HWSURFACE | pygame.DOUBLEBUF)
pygame.display.set_caption("Kontrol na kolata ot PS4")
weather=carla.WeatherParameters()
weather.cloudiness=90
world.set_weather(weather)

camattach.listen(lambda image: parse(display,image,type_of_camera))
# camattach_car.listen(lambda image: publishing(pub,pub_camera,image,rate))
# safing_photos(camattach)

control_car=carla.VehicleControl()
discon=False


list_of_actors.append(vehicle)
list_of_actors.append(camattach)
list_of_actors.append(camattach_car)

file1=open("list_elements.txt",'w')
for item in list_of_actors:
    file1.write("%s\n"% item)
file1.write("%s\n"% vehicle)
file1.flush()

print_info()

comands=MyRosMsg()
while True:
    # is_joystick_pluged(discon)
    pygame.event.get()
    comands.axis_0=0
    comands.axis_1=0
    comands.break_=0
    comands.reverse=False
    comands.is_autopilot=False
    comands.exit=False
    if controler.get_button(0)==True:
        comands.exit=True
    if controler.get_button(1)==True:
        comands.is_autopilot=True
    if controler.get_axis(1)>0:
        comands.reverse=True
    if controler.get_axis(2)>0 or controler.get_axis(5)>0:
        if controler.get_axis(2)>0:
            comands.break_=controler.get_axis(2)
        else:
            comands.break_=controler.get_axis(5)
    comands.axis_0=math.fabs(controler.get_axis(1))
    comands.axis_1=controler.get_axis(3)
    pub_msg.publish(comands)
    if controler.get_button(0)==True:
        spawning_actors.destroyingActors(list_of_actors)
        pygame.joystick.quit()
        pygame.display.quit()
        break
    pygame.display.flip()