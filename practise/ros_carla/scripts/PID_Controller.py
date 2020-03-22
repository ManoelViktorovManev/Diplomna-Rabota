import math
import carla
import numpy as np
from collections import deque


class Controller:

    def __init__(self,vehicle,pid_throttle=None,pid_steer=None):
        self.vehicle=vehicle
        self.world=self.vehicle.get_world()
        self.pid_throttle=pid_throttle
        self.pid_steer=pid_steer
        self.throttle=0
        self.steer=0
        self.breaks=0
        
    def set_speed_on_car_and_steer(self,wp):
        self.pid_throttle.set_currnet_speed()
        self.throttle=self.pid_throttle.moving(wp)
        self.steer=self.pid_steer.run_step(wp)
        self.breaks=self.pid_throttle.breaks
   
class PID_Controller_Steer:

    def __init__(self, vehicle, K_P=1.0, K_D=0.0, K_I=0.0, dt=0.03):

        self.vehicle = vehicle
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
        return self.pid_control(waypoint, self.vehicle.get_transform())


class PID_Controller_Throttle:
    def __init__(self,vehicle,wanted_speed=20):
        self.vehicle=vehicle
        self.wanted_speed=wanted_speed
        self.current_speed=0
        self.breaks=0
    

    def set_wanted_speed(self,wanted):
        self.wanted_speed=wanted
    
    def get_speed(self):
        vel = self.vehicle.get_velocity()
        return 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

    def set_currnet_speed(self):
        self.current_speed=self.get_speed()
        
    def get_speed_val(self,val):
        if val > 1:
            return 0
        elif val > -1 and val < 1:
            return 0.5
        else:
            return 1
        
    def controll_speed(self):
        if self.wanted_speed is 0:
           self.breaks=1
        difference=self.get_speed()-self.wanted_speed
        val=self.get_speed_val(difference)
        return val

    def moving(self,waypoint):
        return self.controll_speed()        
