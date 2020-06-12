import math
import carla
import numpy as np
from collections import deque
import time

class Controller:

    def __init__(self,vehicle,pid_throttle=None,pid_steer=None):
        self.vehicle=vehicle
        self.pid_throttle=pid_throttle
        self.pid_steer=pid_steer
        self.throttle=0
        self.steer=0
        self.breaks=0
        
    def set_speed_on_car_and_steer(self,wp,list_of_actors):
        self.pid_throttle.run(list_of_actors)
        self.pid_steer.run_step(wp)
        self.throttle=self.pid_throttle.throttle
        self.breaks=self.pid_throttle.brakes
        self.steer=self.pid_steer.steer

    def refresh(self):
        self.pid_throttle.refresh()

class PID_Controller_Steer:

    def __init__(self, vehicle, K_P=1.0, K_D=0.0, K_I=0.0, dt=0.03):

        self.vehicle = vehicle
        self.steer=0
        self.K_P = K_P
        self.K_D = K_D
        self.K_I = K_I
        self.dt = dt
        self.e_buffer = deque(maxlen=10)


    # This function is used from ~/<carla_folder>/PythonAPI/carla/agents/navigation/controller.py
    def pid_control(self, waypoint, vehicle_transform):
        v_begin = vehicle_transform.location
        v_end = v_begin + carla.Location(x=math.cos(math.radians(vehicle_transform.rotation.yaw)),
                                         y=math.sin(math.radians(vehicle_transform.rotation.yaw)))
        v_vec = np.array([v_end.x - v_begin.x, v_end.y - v_begin.y, 0.0])
        w_vec = np.array([waypoint.transform.location.x -
                          v_begin.x, waypoint.transform.location.y -
                          v_begin.y, 0.0])
        dot = math.acos(np.clip(np.dot(w_vec, v_vec) /
                                 (np.linalg.norm(w_vec) * np.linalg.norm(v_vec)), -1.0, 1.0))

        cross = np.cross(v_vec, w_vec)
        if cross[2] < 0:
            dot *= -1.0

        self.e_buffer.append(dot)
        de=0
        ie=0
        if len(self.e_buffer) >= 2:
            de = (self.e_buffer[-1] - self.e_buffer[-2]) / self.dt
            ie = sum(self.e_buffer) * self.dt
        else:
            de = 0.0
        
        return np.clip((self.K_P * dot) + (self.K_D * de /
                                             self.dt) + (self.K_I * ie * self.dt), -1.0, 1.0)

    def run_step(self, waypoint):
        self.steer=self.pid_control(waypoint, self.vehicle.get_transform())


class PID_Controller_Throttle:
    def __init__(self,vehicle,wanted_speed=30):
        self.vehicle=vehicle
        self.world=self.vehicle.get_world()
        self.map=self.world.get_map()
        self.wanted_speed=wanted_speed
        self.throttle=0
        self.brakes=0
        self.percent_for_slow=5
        self.traffic_light=None
        self.speed_limit_sign=None
        self.stop_sign=None
        self.timer=None
        self.on_junction=False
        self.real_junction=False

    def run(self,list_of_actors): 
        self.moving()
        self.braking(list_of_actors)

    def refresh(self):
        self.throttle=0
        self.brakes=0
        self.percent_for_slow=5
        self.traffic_light=None
        self.speed_limit_sign=None
        self.stop_sign=None
        self.wanted_speed=25
        self.on_junction=False
        self.real_junction=False
        self.timer=None

    def set_wanted_speed(self,wanted):
        self.wanted_speed=wanted
    
    def get_speed(self,vehicle=None):
        if vehicle==None:    
            vel = self.vehicle.get_velocity()
            return 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)
        else:
            vel=vehicle.get_velocity()
            return 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

    
    def controll_speed(self):
        if self.vehicle.get_transform().rotation.pitch<-5:
            return 0.5

        difference=self.get_speed()-self.wanted_speed
        if difference<-10:
            return 1
        if difference<0:
            return 0.75
        if difference>0 and difference<5:
            return 0.5
        return 0

    def moving(self):
        self.throttle=float(self.controll_speed())

    def imergency_breaks(self):
        self.throttle=0
        self.brakes=1      
    
    def compare_location(self,location1,location2):
        return math.fabs(location1.x-location2.x) + math.fabs(location1.y - location2.y) +math.fabs(location1.z - location2.z)

    def get_speed_coefficient(self):
        return int(self.get_speed())/10

    def is_different_time(self):
        time_int = 0
        if self.timer>=58:
            time_int=self.timer-58
        else:
            time_int = self.timer+2
        if time_int <= int(time.strftime("%S")):
            return True
        return False

    def braking(self,list_of_actors):
        self.brakes=0
        if list_of_actors[1] is not None:
            state= "Green" == str(list_of_actors[1].state)
            if self.traffic_light is None:
                self.traffic_light=list_of_actors[1]
            if state is False and list_of_actors[1].id == self.traffic_light.id:
                
                trigger=list_of_actors[1].trigger_volume
                list_of_actors[1].get_transform().transform(trigger.location)
                self.throttle=0.5
                if self.compare_location(trigger.location,self.vehicle.get_location()) < 6+self.get_speed_coefficient():
                    self.brakes=1
                    self.throttle=0       
        else:
            self.traffic_light=None
            
        if list_of_actors[0] is not None:
            self.throttle=0.5
            if self.get_speed(list_of_actors[0])<1:
                self.throttle=0.25
            front_vehicle_location=list_of_actors[0].get_location()
            if front_vehicle_location.distance(self.vehicle.get_location()) < 8 and self.get_speed(list_of_actors[0])<1:
                self.brakes=1
                self.throttle=0

        if list_of_actors[2] is not None:
            state=str(list_of_actors[2])
            state=int(state[state.find("mit")+4:state.find(")")])
            self.speed_limit_sign=list_of_actors[2]
            self.wanted_speed=state*2/3

        if list_of_actors[3] is not None:
            if self.stop_sign is None: 
                self.stop_sign=list_of_actors[3]

            if self.timer is None:
                self.timer=int(time.strftime("%S"))
            if self.is_different_time() is False or list_of_actors[4] is not None:
                self.brakes=1
                self.throttle=0
        else:
            if self.timer is not None and (self.is_different_time() is True or list_of_actors[4] is None):
                self.stop_sign=None
                self.timer= None
                self.on_junction=True
        
        if self.on_junction is True:
            if list_of_actors[4] is not None:
                self.breaks=1
                self.throttle=0
            wp_junction=self.map.get_waypoint(self.vehicle.get_location()).is_junction
            if wp_junction is True:
                self.real_junction=True
            if wp_junction is False and self.real_junction is True:
                self.on_junction = False
                self.real_junction = False

        # print(self.map.get_waypoint(self.vehicle.get_location()).is_junction)

    def get_type_ot_the_turn(self,waypoints):
        count=waypoints[0].transform.rotation.pitch+waypoints[0].transform.rotation.yaw+waypoints[0].transform.rotation.roll
        diff=0
        for wp in waypoints[1:]:
            new_val=wp.transform.rotation.pitch+wp.transform.rotation.yaw+wp.transform.rotation.roll
            if count != new_val:
                diff=diff+count-new_val

        if diff is 0:
            return None
        elif diff < 10:
            return "Low"
        elif diff < 20:
            return "Big"
        return "Mega"
    
    def is_there_turn(self,waypoints):
        val=self.get_type_ot_the_turn(waypoints)
        if val == None:
            self.percent_for_slow=5
        elif "Low" in val:
            self.percent_for_slow=10
        elif "Big" in val:
            self.percent_for_slow=20
        else:
            self.percent_for_slow=30
