import carla
import math
import os
import time
import waypoints
class Sign:
    def __init__(self,actor):
        self.actor=actor
        self.x=self.actor.get_location().x
        self.y=self.actor.get_location().y
        self.waypoint=self.actor.get_world().get_map().get_waypoint(self.actor.get_location())

    def get_closest_waypoint(self,list):
        min_value=float("inf")
        waypoint=self.waypoint
        for i in list:
            value = math.fabs(i.transform.location.x-self.waypoint.transform.location.x) + math.fabs(i.transform.location.y-self.waypoint.transform.location.y)     
            if min_value > value:
                min_value=value
                waypoint=i
        return waypoint


class MyRadar:
    def __init__(self,vehicle,radius=10):
        self.vehicle=vehicle
        self.radius=radius
        self.world=self.vehicle.get_world()
        self.list_of_actors=self.list_of_actors()
        self.all_waypoints=self.world.get_map().generate_waypoints(2)
        self.dict={}

        # Functions:
        self.getting_near_waypoint_to_traffic_speed_sign()
    
    def getting_near_waypoint_to_traffic_speed_sign(self):
        for i in self.world.get_actors().filter("traffic.speed*"):
            sign = Sign(i)
            wp_road = sign.get_closest_waypoint(self.all_waypoints)
            self.world.debug.draw_point(wp_road.transform.location)
            self.dict[i]=wp_road

    def is_car_in_radius(self,echo_vehicle_cordinates,looking_actor,radius):
        my_vehicle_x = echo_vehicle_cordinates[0]
        my_vehicle_y = echo_vehicle_cordinates[1]
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
        for i in self.world.get_actors().filter("traffic.stop"):
            list.append(i)
        for i in self.world.get_actors().filter("traffic.speed*"):
            list.append(i)

        return list

    def get_road_on_vehicle_car(self,actor):
        waypoint=self.world.get_map().get_waypoint(actor.get_location())
        yaw=self.get_yaw_degrees(waypoint.transform.rotation.yaw)
        yaw=self.get_yaw_degrees(yaw-180)
        waypoint.transform.rotation.yaw=yaw
        return waypoint.next(1)[0]
        # return self.world.get_map().get_waypoint(self.vehicle.get_location()).lane_id

    def get_closest_actors(self):
        list=[]
        echo_vehicle_cordinates=[]
        echo_vehicle_cordinates.append(self.vehicle.get_location().x)
        echo_vehicle_cordinates.append(self.vehicle.get_location().y)
        for i in self.list_of_actors:
            if self.is_car_in_radius(echo_vehicle_cordinates,i,self.radius) is True:
                list.append(i)
        return list
    
    def get_distance(self,actor):
        x=self.vehicle.get_location().x
        y=self.vehicle.get_location().y
        actor_x=actor.get_location().x
        actor_y=actor.get_location().y
        return math.sqrt((x-actor_x) * (x-actor_x) + (y-actor_y) * (y-actor_y))
    
    def are_two_cars_in_one_road(self,actor):
        map=self.world.get_map()
        near_waypoint_to_echo_car=map.get_waypoint(self.vehicle.get_location())
        near_waypoint_to_actor_car=map.get_waypoint(actor.get_location())
        return near_waypoint_to_actor_car.lane_id == near_waypoint_to_echo_car.lane_id
    
    def get_waypoint_to_actor(self,actor):
        wp=self.world.get_map().get_waypoint(actor.get_location())
        near_waypoint_to_echo_car=self.world.get_map().get_waypoint(self.vehicle.get_location())

    def getting_actors_in_the_road(self):
        list=[]
        for i in self.get_closest_actors():
            if i.type_id.find("vehicle") >= 0:
                if self.are_two_cars_in_one_road(i) is True:
                    list.append(i)
            if i.type_id.find("speed")>=0:
                list.append(i)    
        return list

    def get_yaw_degrees(self,yaw):
        if yaw<0:
            return 360-math.fabs(yaw) 
        return yaw
    
    def get_closest_stop_signs(self,list_of_stops):
        list=[]
        for i in list_of_stops:
            if i.type_id.find("stop")>=0:
                list.append(i)
        if len(list) is 0:
            return None
        min_value=float("inf")
        echo_location=self.vehicle.get_location()
        actor=list[0]
        for i in list:
            actor_location=i.get_location()
            x=math.fabs(echo_location.x-actor_location.x)
            y=math.fabs(echo_location.y-actor_location.y)
            value=x/y
            if min_value>value:
                min_value=value
                actor=i
        return actor

    def get_end_range_of_circle(self,begin,degree):
        if yaw+degree > 360:
            new_value = 360 - yaw
            return degree - new_value
        else: 
            return begin+degree

    def get_range_of_circle(self,yaw,begin,degree):
        list_of_range=[]
        list_of_range.append(begin)
        if yaw is begin:
            result=self.get_end_range_of_circle(yaw,degree)
            list_of_range.append(result)
        else:
            result = self.get_end_range_of_circle(begin,degree)
            list_of_range.append(result)

        return list_of_range 

    def get_if_one_speed_sign_is_in_road(self,actor):
        # global dict
        for i in self.dict:
            if i.id is actor.id:
                waypoint=self.dict.get(i)
                break
        vehicle_wp=self.world.get_map().get_waypoint(self.vehicle.get_location())
        lane_change=str(vehicle_wp.lane_change)
        id_road=[]
        while True:
            if lane_change == "Both" or lane_change == "Right":
                id_road.append(vehicle_wp.lane_id)
                vehicle_wp=vehicle_wp.get_right_lane()
                lane_change=str(vehicle_wp.lane_change)
           
            elif lane_change == "Left" or lane_change == "NONE":
                id_road.append(vehicle_wp.lane_id)
                break
        print(len(id_road))
        # trgybwa da naprawa list ot wp okolo 10-15 elementa za da se preceni po koi put se dwijim
        if waypoint.lane_id in id_road:
            return True
        else:
            return False

    def get_closest_speed_limit_sign(self,actor_list):
        for i in actor_list:
            if self.get_distance(i) < 2:
                return i
        return None
        
    def start_looking_for_new_speed(self,actor):
        if actor is not None:
            return self.vehicle.get_speed_limit()
        return None   


    # prawim edin radius okolo to4kata i izbirame nai-blizkata to4ka i taka 6e otkriem wp.
    def get_only_front_actors(self):
        list=[]
        map=self.world.get_map()
        for i in self.getting_actors_in_the_road():
            wp_echo_veh = map.get_waypoint(self.vehicle.get_location())
            wp_actor = map.get_waypoint(i.get_location())

            third_wp=wp_echo_veh.next(1)[0]
            
            distance=waypoints.compare_waypoints_values(wp_echo_veh,wp_actor)
            distance_actor_and_point=waypoints.compare_waypoints_values(wp_actor,third_wp)
            if distance > distance_actor_and_point:

                if i.type_id.find("speed_limit")>=0:
                    if self.get_if_one_speed_sign_is_in_road(i) is True:
                        list.append(i)

                if i.type_id.find("vehicle")>=0:
                    echo_yaw=self.get_yaw_degrees(self.vehicle.get_transform().rotation.yaw)
                    actor_yaw=self.get_yaw_degrees(i.get_transform().rotation.yaw)
                    if math.fabs(echo_yaw-actor_yaw)<=40:
                        list.append(i)
        return list

    def get_is_car_in_traffic_light(self):
        if self.vehicle.get_traffic_light() is not None:
            return 1
        else:
            return 0
    
    def printing(self):
        for i in self.get_only_front_actors():
            print(str(i) + " distance = "+ str(self.get_distance(i)))

# speed_limit
def main():
    client=carla.Client('localhost',2000)
    client.set_timeout(1.0)
    world=client.get_world()
    id_car=input()
    vehicle=world.get_actors().find(id_car)
    # do tuk e kensura
    radar = MyRadar(vehicle,20)
    map=world.get_map() 
    waypoints=map.generate_waypoints(2)
    while True:
        try:
            radar.printing()
        except KeyboardInterrupt:
            break
# main()
"""
Things to do:
1. da detektwa swetofar i znak za skorost -bounding box da raboti
2. togawa da wikame funcciite za tazi boza

"""