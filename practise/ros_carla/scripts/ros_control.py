#!/usr/bin/env python
import rospy
import pygame
import math
from msg_folder.msg import MyRosMsg


# Function that shows how to use the controller to control the car in the simulator.
def print_info():
    print(" ")
    print("Welcome to my CARLA controller")
    print(" ")
    print("Use one of the controlers to move:")
    print("     Left analog - up and down       :       move forward and back")
    print("     Right analog - left and right   :       move left and right")
    print(" ")
    print("From the right buttons:")
    print("     Down button     :       exit")
    print("     Right button     :       turn on/off autopilot")

# Function that returns if any controller is connected with the PC.
def is_joystick_pluged():
    joystick_count = pygame.joystick.get_count()
    if joystick_count > 0:
        return True
    return False  

# Function that initializes the controller. 
def initialization_of_controller():
    try:
        pygame.init()
        listofall=[pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
        controller=listofall[0]
        controller.init()
        return controller
    except IndexError:
            return
# Main function. It discribes that this script is Publisher, that it will publish what the user is typing in the controller.
def main():
    rospy.init_node('ros_carla_control',anonymous=True)
    pub_msg=rospy.Publisher("controler_info",MyRosMsg,queue_size=1)
    controler=initialization_of_controller()
    print_info()
    msg=MyRosMsg()
    while True:
        pygame.event.get()
        msg.throttle=0
        msg.steer=0
        msg.breaks=0
        msg.is_reverse=False
        msg.is_autopilot=False
        msg.is_exit=False
        msg.vehicle_id=0
        msg.vehicle_name=""
        if controler.get_button(0)==True:
            msg.is_exit=True
        if controler.get_button(1)==True:
            msg.is_autopilot=True
        if controler.get_axis(1)>0:
            msg.is_reverse=True
        if controler.get_axis(2)>0 or controler.get_axis(5)>0:
            if controler.get_axis(2)>0:
                msg.breaks=controler.get_axis(2)
            else:
                msg.breaks=controler.get_axis(5)
        msg.throttle=math.fabs(controler.get_axis(1))
        msg.steer=controler.get_axis(3)
        pub_msg.publish(msg)
        if controler.get_button(0)==True:
            break

if __name__ == '__main__':
    main()
