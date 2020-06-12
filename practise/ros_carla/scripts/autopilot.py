#!/usr/bin/env python
import rospy
import carla
import waypoints
import PID_Controller
import detecting_actors
from msg_folder.msg import MyRosMsg
from std_msgs.msg import Int16
import threading
import controller_button

class Autopilot:
    def __init__(self,vehicle=None,pid_controller=None,my_radar=None,creating_road=None):
        try:
            self.vehicle_id=0
            self.vehicle=vehicle
            if vehicle == None:
                self.sub1=rospy.Subscriber('/vehicle_id',Int16,self.getInt)
                client = carla.Client('localhost', 2000)
                client.set_timeout(10.0) 
                world=client.get_world()
                while self.vehicle_id is 0 and self.vehicle is None:
                    if self.vehicle_id !=0:
                        self.vehicle=world.get_actors().find(self.vehicle_id)
                if self.vehicle is None:
                    self.vehicle=world.get_actors().find(self.vehicle_id)
            else:
                self.vehicle=vehicle
            if pid_controller == None:
                control_throttle=PID_Controller.PID_Controller_Throttle(self.vehicle,25)
                control_steer=PID_Controller.PID_Controller_Steer(self.vehicle)
                self.pid_controller=PID_Controller.Controller(self.vehicle,control_throttle,control_steer)
            else:
                self.pid_controller=pid_controller
            if my_radar == None:
                self.my_radar=detecting_actors.MyRadar(self.vehicle,20)
            else:
                self.my_radar=my_radar
            if creating_road == None:
                self.creating_road=waypoints.Creating_Path(self.vehicle)
            else:
                self.creating_road=creating_road     
            self.world=self.vehicle.get_world()   
            self.publisher=rospy.Publisher("/controler_info/full_info",MyRosMsg,queue_size=1)
            self.button=controller_button.Button()
            self.map=self.world.get_map()
            self.autopilot_flag=False
            self.emergency_flag=False

            rospy.Subscriber("/controler_info", MyRosMsg, self.callback)
        except AttributeError:
            return 

    def getInt(self,data):
        if self.vehicle_id != 0:
            self.sub1.unregister()
        self.vehicle_id=data.data



    def callback(self,data):

        if data.is_exit==True:
            print("The console stoped :)")
            rospy.signal_shutdown("exit")
        if data.is_autopilot==False:
            if len(self.button.get_when_the_button_is_released()) == 0 and len(self.button.get_when_the_button_is_pushed()) != 0:
                self.button.add_released_time()
        if data.is_autopilot==True:
            if len(self.button.get_when_the_button_is_pushed())!=0 and len(self.button.get_when_the_button_is_released())!=0:
                self.button.set_date_of_pushed_and_released([])
                self.button.reset_released_data([])
            if len(self.button.get_when_the_button_is_pushed())==0:       
                self.button.add_pushed_time()
                self.button.set_state_of_button(True)
            if self.button.get_state_of_button() is True:
                if self.autopilot_flag is False:
                    self.autopilot_flag=True
                    print("AUTOPILOT ON")
                    thread=threading.Thread(name='asdf', target=self.start)
                    thread.start()
                elif self.autopilot_flag is True:
                    self.autopilot_flag=False
                    print("AUTOPILOT OFF")
                self.button.set_state_of_button(False)

    def publish_MyRosMsg(self,vehicle_control):
        msg=MyRosMsg()
        msg.throttle=vehicle_control.throttle
        msg.steer=vehicle_control.steer
        msg.breaks=vehicle_control.brake
        msg.is_reverse=False
        msg.is_autopilot=False
        msg.is_exit=False
        msg.vehicle_id=self.vehicle.id
        msg.vehicle_name=self.vehicle.type_id
        self.publisher.publish(msg)

# Number of nodes: 69
# Number of edges: 154

    def get_under_control(self):
        
        if self.pid_controller.pid_throttle.get_speed()-self.pid_controller.pid_throttle.wanted_speed > 20 or self.pid_controller.pid_throttle.wanted_speed>20:
            self.emergency_flag=True        
                
                
        else:
            if self.emergency_flag is True:
                if len(self.creating_road.emergency_waypoints) != 0:
                    self.emergency_flag=False
                    self.creating_road.remember_wp_to_follow=None
                    self.creating_road.refresh_waypoints()
    
    def start(self):
        print("Here?")
        self.creating_road.refresh_call()
        self.creating_road.create_path_to_closest_crossroad()
        self.creating_road.create_path()
        self.pid_controller.pid_throttle.set_wanted_speed(20)
        
        # self.creating_road.printing_path()

        while True:     
            if self.autopilot_flag is False:
                self.emergency_flag=False
                return
            try:
                self.get_under_control()

                if self.creating_road.current_waypoint is None or self.creating_road.compare_waypoint_and_vehicle(self.creating_road.current_waypoint) < 2:
                    self.creating_road.current_waypoint=self.creating_road.getting_waypoint(self.creating_road.waypoints_from_the_path)
                    self.creating_road.waypoints_from_the_path.remove(self.creating_road.waypoints_from_the_path[0])
                    if self.emergency_flag is True:
                        self.creating_road.emergency_wp(self.pid_controller.pid_throttle.get_speed())

                if len(self.creating_road.waypoints_from_the_path)<10:
                    self.creating_road.create_path()
                    # self.creating_road.printing_path()

                actors=self.my_radar.get_only_front_actors(self.creating_road.waypoints_from_the_path[0:10])
                vehicle_control=carla.VehicleControl()

                if self.emergency_flag is True and self.creating_road.remember_wp_to_follow is not None:
                    self.pid_controller.set_speed_on_car_and_steer(self.creating_road.remember_wp_to_follow,actors)
                else:
                    self.pid_controller.set_speed_on_car_and_steer(self.creating_road.current_waypoint,actors)
                vehicle_control.throttle=self.pid_controller.throttle
                vehicle_control.steer=self.pid_controller.steer
                vehicle_control.brake=self.pid_controller.breaks
                self.vehicle.apply_control(vehicle_control)
                self.publish_MyRosMsg(vehicle_control)
            
            except (IndexError,KeyboardInterrupt,rospy.ROSException) as exception:
                print("Ther was Exception")
                break

if __name__ == '__main__':
    rospy.init_node('autopilot_node', anonymous=True)
    autopilot=Autopilot()
    rospy.spin()
