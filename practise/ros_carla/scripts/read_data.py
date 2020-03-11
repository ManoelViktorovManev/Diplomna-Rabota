#!/usr/bin/env python
import carla
from msg_folder.msg import MyRosMsg
from std_msgs.msg import Int16
import pygame
import controller_button
import rospy
import spawning_actors
import publishing_all_information


client = carla.Client('localhost', 2000)
client.set_timeout(10.0) 
world=client.get_world()
actors_list=world.get_actors()
button_autopilot=controller_button.Button()
autopilot=False
control_car=carla.VehicleControl()
vehicle_id=None
draw=world.debug

def get_car_cordinates(vehicle):
    return vehicle.get_location()


def drawing(cordinates,drawing):
    drawing.draw_point(cordinates,color=carla.Color(0,255,0))
    
# Function that destroys every actor that I have made.
def destroying_actors(world):
    list_of_actors=world.get_actors().filter("vehicle*")
    if len(list_of_actors)>0:
        spawning_actors.destroyingActors(list_of_actors)
    list_of_actors=world.get_actors().filter("sensor*")
    if len(list_of_actors)>0:
        spawning_actors.destroyingActors(list_of_actors)

# Function that is called with the message as the first argument, when the subscriber receives new messages.
def callback(data):
    global autopilot
    global button_autopilot
    global vehicle_id
    global world
    global draw

    vehicle=world.get_actors().find(vehicle_id)
    if data.is_exit==True:
        print("The console stoped :)")
        destroying_actors(world)
        rospy.signal_shutdown("exit")

    if data.is_autopilot==False:
        if len(button_autopilot.get_when_the_button_is_released()) == 0 and len(button_autopilot.get_when_the_button_is_pushed()) != 0:
            button_autopilot.add_released_time()
            
    if data.is_autopilot==True:
        if len(button_autopilot.get_when_the_button_is_pushed())!=0 and len(button_autopilot.get_when_the_button_is_released())!=0:
                button_autopilot.set_date_of_pushed_and_released([])
        if len(button_autopilot.get_when_the_button_is_pushed())==0:       
            button_autopilot.add_pushed_time()
            button_autopilot.set_state_of_button(True)

        if button_autopilot.get_state_of_button() is True:
            if autopilot is False:
                vehicle.set_autopilot(True)
                autopilot=True
                print("AUTOPILOT ON")
            elif autopilot is True:
                autopilot=False
                vehicle.set_autopilot(False)
                print("AUTOPILOT OFF")
            button_autopilot.set_state_of_button(False)
    if autopilot == False:
        control_car.reverse=data.is_reverse
        control_car.brake=data.breaks
        control_car.throttle=data.throttle
        control_car.steer=data.steer
        vehicle.apply_control(control_car)
    print(vehicle.get_transform().rotation)
    # drawing(get_car_cordinates(vehicle),draw)

# Function that gets the id on the echo vehicle - the car that the user's controlling.
def getInt(data):
    global vehicle_id
    vehicle_id=data.data

# Function that initializes that this script is a subscriber.
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("controler_info", MyRosMsg, callback)
    rospy.Subscriber('/vehicle_id',Int16,getInt)
    rospy.spin()

if __name__ == '__main__':
    listener()
