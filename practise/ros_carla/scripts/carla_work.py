#!/usr/bin/env python

import carla
import pygame
import rospy
import time

import spawning_actors
import displaying_users_camera
import publishing_image

from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import Int16

# Function that checks if there are actors in simulation when the script is started and afterwards destroying them. 
# This fundction is used when there is a program crash.  
def checking_for_actors(world):
    list_of_actors=world.get_actors().filter("vehicle*")
    if len(list_of_actors)>0:
        spawning_actors.destroyingActors(list_of_actors)
    list_of_actors=world.get_actors().filter("sensor*")
    if len(list_of_actors)>0:
        spawning_actors.destroyingActors(list_of_actors)

# Function that returns what type of a camera is used for visualization. Types are : 0 - sensor.camera.rgb, 1 - sensor.camera.depth and 
#  2 - sensor.camera.semantic_segmentation
def getting_camera_sensor(camera):
    type_of_camera=0
    if "depth" in camera.type_id:
        type_of_camera=1
    elif "semantic_segmentation" in camera.type_id:
        type_of_camera=2
    return type_of_camera

# Function that checks if there is a controller connected with the PC. If there is not, then it will wait until it is connected to a one.
def checking_joystick(controller=None):
    while True:
        try:
            pygame.quit()
            pygame.init()
            pygame.joystick.init()
            listofall=[pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
            controller=listofall[0]
            controller.init()
        except IndexError:
            print("WARNING!!!")
            print("There is no joysticks. Please insert one.")
            time.sleep(1)
        else:
            return controller



client=carla.Client('localhost',2000)
client.set_timeout(1.0)
world=client.get_world()

rospy.init_node('ros_subscriber',anonymous=True)
pub_image=rospy.Publisher('/camera/rgb/image',Image,queue_size=10)
pub_camera=rospy.Publisher('/camera/rgb/camera_info',CameraInfo,queue_size=10)
pub_id_car=rospy.Publisher('/vehicle_id',Int16,queue_size=1)


checking_for_actors(world)
controler=checking_joystick()
spawning_actors.main()

list_of_actors=spawning_actors.listofActors()

blueprint_library = world.get_blueprint_library()

# car=spawning_actors.chose_element_from_list(blueprint_library.filter("vehicle*"))
car=blueprint_library.find("vehicle.mercedes-benz.coupe")
transform = carla.Transform(carla.Location(x=30, y=0, z=3), carla.Rotation(yaw=180))
vehicle = world.spawn_actor(car, transform)

# setting spawned car not to move
control_car=carla.VehicleControl()
control_car.brake=1.0
vehicle.apply_control(control_car)

# cam=spawning_actors.chose_element_from_list(blueprint_library.filter("sensor.camera*"))
camera_for_user=blueprint_library.find("sensor.camera.rgb")
camera_visualization=blueprint_library.find("sensor.camera.rgb")

transform_cam=carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15))
user_camera=world.spawn_actor(camera_for_user,transform_cam,attach_to=vehicle)

transform_cam=carla.Transform(carla.Location(x=3, z=1), carla.Rotation(pitch=-15))
rViz_camera=world.spawn_actor(camera_visualization,transform_cam,attach_to=vehicle)

type_of_camera=getting_camera_sensor(rViz_camera)

display=displaying_users_camera.make_display()

rViz_camera.listen(lambda image: publishing_image.publishing(pub_image,pub_camera,image,type_of_camera))
user_camera.listen(lambda image: displaying_users_camera.parse(display,image,0))

list_of_actors.append(vehicle)
list_of_actors.append(user_camera)
list_of_actors.append(rViz_camera)
while True:
    pub_id_car.publish(vehicle.id)
    pygame.event.get()
    if controler.get_button(0)==True:
        rViz_camera.destroy()
        user_camera.destroy()
        break
    pygame.display.flip()