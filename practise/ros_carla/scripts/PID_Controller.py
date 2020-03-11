import math
import carla


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
        self.pid_steer.set_waypoint(wp)
        self.pid_throttle.set_currnet_speed()
        # if self.pid_steer.get_diference_between_vehicle_and_wp() > 10:
        #     self.pid_throttle.set_wanted_speed(10)
        # else:
            # self.pid_throttle.set_wanted_speed(20)
        # self.pid_throttle.set_wanted_speed(20)
        self.throttle=self.pid_throttle.moving(wp)
        self.steer=self.pid_steer.set_steer()
        self.breaks=self.pid_throttle.breaks
   
# 1 zawoi kum posokata na wp
# 2 zawoi kum wp
class PID_Controller_Steer:
    def __init__(self,vehicle):
        self.vehicle=vehicle
        self.waypoint=None
        self.steer=0

    def set_waypoint(self,wp):
        self.waypoint=wp

    def get_diference_between_vehicle_and_wp(self):
        if self.waypoint is not None:
            wp_angle=self.waypoint.transform.rotation.yaw
            if wp_angle < 0:
                wp_angle=360-math.fabs(wp_angle)
            
            vehicle_angle=self.vehicle.get_transform().rotation.yaw
            if vehicle_angle < 0:
                vehicle_angle=360-math.fabs(vehicle_angle)

            right_angle=left_angle=0
            if min(vehicle_angle,wp_angle) is wp_angle:
                right_angle=vehicle_angle-wp_angle
                left_angle=-(360-right_angle)
            else:
                left_angle=-(wp_angle-vehicle_angle)
                right_angle=360-(-left_angle)
            if math.fabs(right_angle)<math.fabs(left_angle):
                return right_angle
            else:
                return left_angle
        return None

    def distance_vehicle(self,waypoint):
        loc = self.vehicle.get_location()
        dx = waypoint.transform.location.x - loc.x
        dy = waypoint.transform.location.y - loc.y
        return math.sqrt(dx * dx + dy * dy)
    
    def get_angle_to_the_wp(self):
        loc=self.vehicle.get_location()
        dx=math.fabs(self.waypoint.transform.location.x - loc.x)
        dy=math.fabs(self.waypoint.transform.location.y - loc.y)
        if loc.y>self.waypoint.transform.location.y:
            if loc.x > self.waypoint.transform.location.x:
                return dy/dx
            return -dy/dx
        else:
            if loc.x > self.waypoint.transform.location.x:
                return -dx/dy  
            return  dx/dy
    
    def set_steer(self):
        # angle=self.get_diference_between_vehicle_and_wp()
        # print(angle)
        angle=self.get_angle_to_the_wp()
        # if angle is not None:
        angle=angle*180/3.14
        # + ili - ?   
        if math.fabs(angle)<=2:
            return 0
        else:
            return 1.5*angle/180

# steer - zawoi naso4wane
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
        # print(self.get_speed())
        val=self.get_speed_val(difference)
        return val



    def moving(self,waypoint):
        return self.controll_speed()
        