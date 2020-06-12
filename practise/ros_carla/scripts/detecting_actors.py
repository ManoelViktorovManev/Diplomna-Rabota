import carla
import math

class Point:
    def __init__(self,x=0,y=0,hipotenuza=0):
        self.x=x
        self.y=y
        self.katet_x=0
        self.katet_y=0
        self.hipotenuza=hipotenuza

    def finding_katet_y(self,angle=None,y=None):
        if angle!=None:
            self.katet_y=math.sin(math.radians(angle))*self.hipotenuza
        else:
            self.katet_y=self.y-y
    def finding_katet_x(self,angle=None,x=None):
        if angle!=None:
            self.katet_x=math.cos(math.radians(angle))*self.hipotenuza
        else:
            self.katet_x=self.x-x

    def set_x_and_y(self,x,y):
        self.x= x+self.katet_x
        self.y=y+self.katet_y

    def hipotenuza_find(self):
        self.hipotenuza=math.sqrt(self.katet_x**2 + self.katet_y**2)


class MyRadar:
    def __init__(self,vehicle,radius=15):
        self.vehicle=vehicle
        self.radius=radius
        self.world=self.vehicle.get_world()
        self.map=self.world.get_map()
        self.actors=self.list_of_actors()
        self.all_waypoints=self.world.get_map().generate_waypoints(2)
        self.is_car_in_junction=None
        self.traffic_light=self.world.get_actors().filter('traffic.traffic_light')
        self.speed_limit=self.world.get_actors().filter('traffic.speed*')
        self.stop_sign=self.world.get_actors().filter('traffic.stop')
        
        self.print_all_traffic_signs()

    def print_all_traffic_signs(self):
        debug=self.world.debug
        for sign in self.speed_limit:
            trigger=sign.trigger_volume
            sign.get_transform().transform(trigger.location)
            debug.draw_box(trigger,sign.get_transform().rotation)

        for sign in self.traffic_light:
            trigger=sign.trigger_volume
            sign.get_transform().transform(trigger.location)
            debug.draw_box(trigger,sign.get_transform().rotation,color=carla.Color(0,255,0))
        
        for sign in self.stop_sign:
            trigger=sign.trigger_volume
            sign.get_transform().transform(trigger.location)
            debug.draw_box(trigger,sign.get_transform().rotation,color=carla.Color(0,0,255))
    
    def is_car_in_radius(self,looking_actor):
        my_vehicle_x = self.vehicle.get_location().x
        my_vehicle_y = self.vehicle.get_location().y
        actor_x = looking_actor.get_location().x
        actor_y = looking_actor.get_location().y
        radius_square = self.radius ** 2
        num = (( actor_x - my_vehicle_x ) ** 2 ) + (( actor_y - my_vehicle_y) ** 2)
        if radius_square>=num:
            return True
        return False


    def list_of_actors(self):
        list=[]
        for i in self.world.get_actors().filter("vehicle*"):
            if i.id is not self.vehicle.id:
                list.append(i)
        return list


    def get_closest_actors(self):
        list=[]
        for i in self.actors:
            if self.is_car_in_radius(i) is True:
                list.append(i)
        return list
    
    def are_two_cars_in_one_road(self,actor,waypoints_list):
        near_waypoint_to_echo_car=self.map.get_waypoint(self.vehicle.get_location())
        near_waypoint_to_actor_car=self.map.get_waypoint(actor.get_location())
        if near_waypoint_to_actor_car.lane_id == near_waypoint_to_echo_car.lane_id:
            return True
        for wp in waypoints_list:
            if wp.road_id == near_waypoint_to_actor_car.road_id and wp.lane_id == near_waypoint_to_actor_car.lane_id:
                return True 
        return False    


    def getting_actors_in_the_road(self,wp_list):
        list=[]
        for i in self.get_closest_actors():
            if i.type_id.find("vehicle") >= 0:
                if self.are_two_cars_in_one_road(i,wp_list) is True:
                    list.append(i)   
        return list

    def get_yaw_degrees(self,yaw):
        if yaw<0:
            return 360-math.fabs(yaw) 
        return yaw

    def is_point_in_the_half_circle(self,main_point,looking_point):
        kateti_count=(main_point.katet_x*looking_point.katet_x) + (main_point.katet_y * looking_point.katet_y)
        hipotenuza_count=main_point.hipotenuza*looking_point.hipotenuza
        if hipotenuza_count/kateti_count>=0:
            return True
        return False

    def get_only_front_cars_math(self):
        list=[]
        angle=self.get_yaw_degrees(self.vehicle.get_transform().rotation.yaw)
        p1= Point(hipotenuza=self.radius)
        p1.finding_katet_x(angle=angle)
        p1.finding_katet_y(angle=angle)
        p1.set_x_and_y(self.vehicle.get_location().x,self.vehicle.get_location().y)
        for i in self.get_closest_actors():
            p2 = Point(x=i.get_location().x,y=i.get_location().y)
            p2.finding_katet_x(x=self.vehicle.get_location().x)
            p2.finding_katet_y(y=self.vehicle.get_location().y)
            p2.hipotenuza_find()
            if self.is_point_in_the_half_circle(p1,p2) is True:
                list.append(i)
        return list

    def get_only_moving_vehicles_on_junction(self):
        list=[]
        for vehicle in self.get_only_front_cars_math():
            waypoint=self.map.get_waypoint(vehicle.get_location())
            vel = vehicle.get_velocity()
            speed=3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)
            if waypoint.is_junction is True and speed>5:
                list.append(vehicle)
        return list

    def is_point_contains_in_3d_cube(self,trigger_box,wp):
        trigger=trigger_box.trigger_volume
        trigger_box.get_transform().transform(trigger.location)

        x_low=trigger.location.x-trigger.extent.x
        x_high=trigger.location.x+trigger.extent.x
        y_low=trigger.location.y-trigger.extent.y
        y_high=trigger.location.y + trigger.extent.y

        vehicle_bounding_box= self.vehicle.bounding_box
        self.vehicle.get_transform().transform(vehicle_bounding_box.location)

        wp_x_high = vehicle_bounding_box.location.x + vehicle_bounding_box.extent.x
        wp_x_low = vehicle_bounding_box.location.x-vehicle_bounding_box.extent.x
        wp_y_high = vehicle_bounding_box.location.y + vehicle_bounding_box.extent.y
        wp_y_low = vehicle_bounding_box.location.y-vehicle_bounding_box.extent.y


        wp_x=wp.transform.location.x
        wp_y=wp.transform.location.y
        wp_z=wp.transform.location.z

        if x_low <= wp_x <= x_high and y_low <= wp_y <= y_high:
            return True
        wp_car=self.map.get_waypoint(self.vehicle.get_location())
        wp_sign=self.map.get_waypoint(trigger_box.get_location())
        if wp_car.lane_id == wp_sign.lane_id and wp_car.transform.rotation.yaw-5 <= wp_sign.transform.rotation <= wp_car.transform.rotation.yaw+5:
            return (x_low<= wp_x_high <= x_high or  x_low<= wp_x_low <= x_high ) and ( y_low<= wp_y_high <= y_high or y_low<= wp_y_low <= y_high )
        return False

    def get_stop_sign_is_front(self,wp):
        list=[]
        for i in self.stop_sign:
            if self.is_car_in_radius(i):
                list.append(i)
        for i in list:
            trigger=i.trigger_volume
            i.get_transform().transform(trigger.location)
            if self.is_point_contains_in_3d_cube(i,wp) is True and self.detect_trigger_box_in_other_trigger_box(trigger) is False:
                return i
        return None
        
    def get_only_front_actors(self,list_of_wp=[]):
        list=[]
        dict_for_cars={}
        wp_echo_veh = self.map.get_waypoint(self.vehicle.get_location())
        self.is_car_in_junction=wp_echo_veh.is_junction
        for i in self.getting_actors_in_the_road(list_of_wp):
            
            wp_actor = self.map.get_waypoint(i.get_location())

            third_wp=wp_echo_veh.next(1)[0]
            
            distance=self.compare_waypoints_values(wp_echo_veh,wp_actor)
            distance_actor_and_point=self.compare_waypoints_values(wp_actor,third_wp)

            if distance > distance_actor_and_point:
                if i.type_id.find("vehicle")>=0:
                    echo_yaw=self.get_yaw_degrees(self.vehicle.get_transform().rotation.yaw)
                    actor_yaw=self.get_yaw_degrees(i.get_transform().rotation.yaw)
                    if math.fabs(echo_yaw-actor_yaw)<=40:
                        # dict_for_cars[i]=distance_actor_and_point
                        # it was with only that up
                        
                        trigger=i.bounding_box
                        i.get_transform().transform(trigger.location)

                        x_low=trigger.location.x-trigger.extent.x
                        x_high=trigger.location.x+trigger.extent.x
                        y_low=trigger.location.y-trigger.extent.y
                        y_high=trigger.location.y + trigger.extent.y

                        vehicle_bounding_box= self.vehicle.bounding_box
                        self.vehicle.get_transform().transform(vehicle_bounding_box.location)

                        wp_x_high = vehicle_bounding_box.location.x + vehicle_bounding_box.extent.x
                        wp_x_low = vehicle_bounding_box.location.x-vehicle_bounding_box.extent.x
                        wp_y_high = vehicle_bounding_box.location.y + vehicle_bounding_box.extent.y
                        wp_y_low = vehicle_bounding_box.location.y-vehicle_bounding_box.extent.y

                        par1 = min(x_low - wp_x_low, x_low - wp_x_high)
                        par2 = min(x_high - wp_x_low, x_high - wp_x_high)
                        par3 = min(y_low - wp_y_high, y_low - wp_y_low)
                        par4 = min(y_high - wp_y_high, y_high - wp_y_low)

                        final_par=min(par1,par2,par3,par4)
                        dict_for_cars[i]=final_par

        list.append(self.finding_closest_actor(dict_for_cars))
        list.append(self.detecting_trafic_signs(detect_traffic_light=True))
        list.append(self.detecting_trafic_signs(detect_speed_limit=True))
        list.append(self.get_stop_sign_is_front(list_of_wp[0]))
        junc_veh=self.get_only_moving_vehicles_on_junction()
        if len(junc_veh)>0:
            list.append(junc_veh)
        else:
            list.append(None)

        return list

    def compare_waypoints_values(self,wp1,wp2):
        return math.fabs(wp1.transform.location.x-wp2.transform.location.x) + math.fabs(wp1.transform.location.y-wp2.transform.location.y) 
        + math.fabs(wp1.transform.location.z-wp2.transform.location.z) + math.fabs(wp1.transform.rotation.pitch-wp2.transform.rotation.pitch)
        + math.fabs(wp1.transform.rotation.yaw-wp2.transform.rotation.yaw) + math.fabs(wp1.transform.rotation.roll-wp2.transform.rotation.roll)
    
    def counting(self,item):
        return math.sqrt(item.x**2 + item.y**2 + item.z**2)

    def finding_closest_actor(self,dict):
        if len(dict) is 0:
            return None
        list=[]
        for i in dict.values():
            list.append(i)
        min_val=min(list)
        for key,value in dict.items():
            if min_val== value:
                return key
    
    def same_rotations(self,wp):
        rotation_vehicle=self.map.get_waypoint(self.vehicle.get_location()).transform.rotation.yaw
        rotation_wp=wp.transform.rotation.yaw
        if rotation_vehicle<0:
            rotation_vehicle=360+rotation_vehicle
        if rotation_wp<0:
            rotation_wp=360+rotation_wp
        if math.fabs(rotation_wp-rotation_vehicle)<20:
            return True
        return False

    def detect_trigger_box_in_other_trigger_box(self,trigger):
        for sign in self.traffic_light:
            trig=sign.trigger_volume
            sign.get_transform().transform(trig.location)
            distance_to_car=trig.location.distance(trigger.location)
            s= self.counting(trig.extent) + self.counting(trigger.extent)
            if s>=distance_to_car:
                return True
        return False


    def detecting_trafic_signs(self,detect_traffic_light=False,detect_speed_limit=False,detect_stop_sign=False):
        list=[]
        dic_for_stop_sign={}
        if detect_traffic_light is True:
            list=self.traffic_light
        elif detect_speed_limit is True:
            list=self.speed_limit
        else:
            list= self.stop_sign
        for sign in list:
            trigger=sign.trigger_volume
            sign.get_transform().transform(trigger.location)
            distance_to_car=trigger.location.distance(self.vehicle.get_location())
            s= self.counting(trigger.extent) + self.counting(self.vehicle.bounding_box.extent)

            if s>=distance_to_car:
                if self.same_rotations(self.map.get_waypoint(trigger.location)):
                    return sign
        return None
