#!/usr/bin/env python

import rospy
import carla
import waypoints
import autopilot
import PID_Controller
from msg_folder.msg import MyRosMsg

class Autopilot:

    def __init__(self,vehicle,Pid_controller=None,my_radar=None,creating_road=None):
        self.vehicle=vehicle
        self.Pid_controller=Pid_controller
        self.my_radar=my_radar
        self.creating_road=creating_road
        self.world=self.vehicle.get_world()
        self.draw=self.world.debug
        self.publisher=rospy.Publisher('/autopilot/commands',MyRosMsg,queue_size=1)
        rospy.init_node('talker', anonymous=True)
        # towa gore mai nema da mi trybwa :)

    def get_main_MyRosMsg(self):
        msg=MyRosMsg()
        msg.throttle=0
        msg.steer=0
        msg.breaks=0
        msg.is_reverse=False
        msg.is_autopilot=False
        msg.is_exit=False
        msg.vehicle_id=self.vehicle.id
        msg.vehicle_name=self.vehicle.type_id
        return msg

    def start(self):
        # print("ok")
        self.creating_road.create_road()
        self.creating_road.printing_path()
        while True:
            try:
                if self.creating_road.current_waypoint is None or self.creating_road.compare_waypoint_and_vehicle(self.creating_road.current_waypoint) < 1:

                    self.creating_road.current_waypoint=self.creating_road.getting_waypoint(self.creating_road.waypoints_from_the_path)
                    self.creating_road.waypoints_from_the_path.remove(self.creating_road.waypoints_from_the_path[0])
                    # w edna funckiya twa gornoto 

                if len(self.creating_road.waypoints_from_the_path)<5:
                    self.creating_road.create_road(self.creating_road.waypoints_from_the_path)
                    self.creating_road.printing_path()

                msg=self.get_main_MyRosMsg()

                # self.my_radar.printing()

                vehicle_control=carla.VehicleControl()
                self.Pid_controller.set_speed_on_car_and_steer(self.creating_road.current_waypoint)
                vehicle_control.throttle=self.Pid_controller.throttle
                vehicle_control.steer=self.Pid_controller.steer
                self.vehicle.apply_control(vehicle_control)
            except IndexError:
                break
            except KeyboardInterrupt:
                break
        # <3 <3 <3 <3 <3 <3 <3 <3 <3 <3 <3 <3 <3 <3 <3 <3 <3 <3 <3 <3 <3 <3 <3 <3 <3 <3 <3 <3 <3 <3 <3 <3 <3 <3 <3 <3 <3 <3 <3 <3 <3 <3 <3 <3 <3 <3 <3 <3

        # self.creating_road.showing_graph()