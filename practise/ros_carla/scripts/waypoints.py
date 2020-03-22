import time
import carla 
import sys
import networkx as nx
import matplotlib.pyplot as plt
import math
import random
import PID_Controller


class Createing_Road:
    
    def __init__(self,vehicle):
        self.vehicle=vehicle
        self.world=self.vehicle.get_world()
        self.draw=self.world.debug
        self.map=self.world.get_map()
        self.all_waypoints=self.map.generate_waypoints(2.0)
        self.length=0
        self.graph=nx.DiGraph()
        self.path=None
        self.waypoints_from_the_path=[]
        self.current_waypoint=None

        # Function to call to make graph :)
        self.making_nodes_only_from_crossroad()
        self.making_graph_from_crossroad()

    # Function that makes nodes which are crossroads. 
    def making_nodes_only_from_crossroad(self):
        for i in self.all_waypoints:
            if len(i.next(2.0))>1:
                self.graph.add_node(i)

    # Function that returns the number of an element in the graph that is with the same value as wp. 
    def get_element_from_graph(self,wp):
        for i in list(self.graph.nodes):
            if self.compare_waypoints_instance(i,wp) is True:
                return i
        return None
        
    # Function that draws a waypoint.
    def drawing_waypoint(self,waypoint):
        self.draw.draw_point(waypoint.transform.location)

    # Function that compares two waypoints.
    def compare_waypoints_instance(self,wp1,wp2):
        return (wp1.transform.location == wp2.transform.location) and (wp1.transform.rotation == wp2.transform.rotation)

    # Function that compares the value between wp1 and wp2
    def compare_waypoints_values(self,wp1,wp2):
        return math.fabs(wp1.transform.location.x-wp2.transform.location.x) + math.fabs(wp1.transform.location.y-wp2.transform.location.y) 
        + math.fabs(wp1.transform.location.z-wp2.transform.location.z) + math.fabs(wp1.transform.rotation.pitch-wp2.transform.rotation.pitch)
        + math.fabs(wp1.transform.rotation.yaw-wp2.transform.rotation.yaw) + math.fabs(wp1.transform.rotation.roll-wp2.transform.rotation.roll)
        # FUNTIATA DA YA OPRAWA

    # ?????????
    def compare_waypoint_and_vehicle(self,wp):
        vehicle=self.vehicle.get_transform()
        return math.fabs(wp.transform.location.x-vehicle.location.x) + math.fabs(wp.transform.location.y-vehicle.location.y) 
        + math.fabs(wp.transform.location.z-vehicle.location.z) + math.fabs(wp.transform.rotation.yaw-vehicle.rotation.yaw)

    # Function that gets the next waypoint
    def getting_waypoint(self,list):
        return list[0]

    # Function that returns closest one waypoint to wp.
    def getting_closest_one_wp(self,wp):
        min_value=float("inf")
        waypoint=wp
        for i in list(self.graph.nodes):
            value = math.fabs(i.transform.location.x-wp.transform.location.x) + math.fabs(i.transform.location.y-wp.transform.location.y) 
            + math.fabs(i.transform.location.z-wp.transform.location.z) + math.fabs(i.transform.rotation.pitch-wp.transform.rotation.pitch)
            + math.fabs(i.transform.rotation.yaw-wp.transform.rotation.yaw) + math.fabs(i.transform.rotation.roll-wp.transform.rotation.roll)
            
            if min_value > value:
                min_value=value
                waypoint=i
        return waypoint
        # POWTARYA SE SUS DOLNATA FUNKCIYA

    # Function that returns closest waypoint to the car 
    def getting_closest_one_wp_from_car(self,cordinated):
        min_value=float("inf")
        waypoint=cordinated
        for i in list(self.graph.nodes):
            value = math.fabs(i.transform.location.x-cordinated.location.x) + math.fabs(i.transform.location.y-cordinated.location.y) 
            + math.fabs(i.transform.location.z-cordinated.location.z) + math.fabs(i.transform.rotation.pitch-cordinated.rotation.pitch)
            + math.fabs(i.transform.rotation.yaw-cordinated.rotation.yaw) + math.fabs(i.transform.rotation.roll-cordinated.rotation.roll)
            if min_value > value:
                min_value=value
                waypoint=i
        return waypoint
        # POWTARYA SE SUS GORNATA FUNKCYTA


    # Function that returns list of waypoints whcih conect the value waypoint
    def connected_nodes_from_crossroad(self,waypoint):
        list_of_connected_crossroads=[]
        self.length=0
        for i in waypoint.next(2.0):
            while True:
                next_wp=i.next(2.0)
                if len(next_wp)>1:
                    list_of_connected_crossroads.append(i)
                    break
                i=self.getting_waypoint(i.next(2.0))
                self.length+=2
        return list_of_connected_crossroads

    # Function that creates list between wp1 and wp2. 
    def get_wp_that_is_for_crossroad(self,wp1,wp2):
        list=[]
        while True:
            next_wp=wp1.next(2.0)
            if len(next_wp)>1:
                if self.compare_waypoints_values(wp1,wp2)<10:
                    list.append(wp2)
                    return list
                else:
                    return None
            list.append(wp1)
            wp1=self.getting_waypoint(wp1.next(2.0))
                
    # Function that returns list of all waypoints between wp1 and wp2.
    def get_all_waypoints_from_waypoint_to_waypoint(self,wp1,wp2):
        for i in wp1.next(2.0):
            looking_list = self.get_wp_that_is_for_crossroad(i,wp2)
            if looking_list is not None:
                return looking_list

    # Function that connects nodes from the graph
    def making_graph_from_crossroad(self):
        for i in list(self.graph.nodes()):
            crossroad=self.connected_nodes_from_crossroad(i)
            for k in crossroad:
                node=self.getting_closest_one_wp(k)
                node=self.get_element_from_graph(node)
                list_of_wp=self.get_all_waypoints_from_waypoint_to_waypoint(i,node)
                self.graph.add_edge(i,node,length=self.length,data=list_of_wp)

    # Function that displays the graph with connections between nodes.
    def showing_graph(self):
        print(nx.info(self.graph))
        nx.draw(self.graph)
        plt.show()
    
    # Function that chooses ranndom node from graph 
    def choose_random(self):
        return random.choice(list(self.graph.nodes()))

    # Funtcion that prints the path 
    def printing_path(self):
        for i in self.waypoints_from_the_path:
            self.drawing_waypoint(i)
            time.sleep(0.0001)

    def getting_all_waypoints_from_node_to_node(self):
        list=[]
        for i in range(len(self.path[:-1])):
            arr1=self.path[i]
            arr2=self.path[i+1]
            asdf=self.graph.get_edge_data(self.get_element_from_graph(arr1),self.get_element_from_graph(arr2))['data']
            list.append(asdf)
        return list

    def converting_the_path_to_list_of_waypoints(self,list):
        listofwp=[]
        for i in list:
            for k in i:
                listofwp.append(k)
        return listofwp

    def get_closest_wp_to_the_car(self):
        return self.map.get_waypoint(self.vehicle.get_location())


    def get_wp_to_crossroad(self,waypoint,list=[]):
        next_wp=waypoint.next(2.0)
        if len(next_wp)>1:
            list.append(waypoint)
            return list
        list.append(waypoint)
        waypoint=self.getting_waypoint(next_wp)
        return self.get_wp_to_crossroad(waypoint,list)

    def create_road(self,remain_waypoints=[]):
        if len(remain_waypoints) == 0:
            close_one_wp_to_car=self.get_closest_wp_to_the_car()
            waypoints_to_crossroad=self.get_wp_to_crossroad(close_one_wp_to_car)
            remain_waypoints=waypoints_to_crossroad    
        
        self.path=nx.shortest_path(self.graph,self.getting_closest_one_wp_from_car(remain_waypoints[-1].transform),self.choose_random())
        waypoints= self.converting_the_path_to_list_of_waypoints(self.getting_all_waypoints_from_node_to_node())
        self.waypoints_from_the_path=remain_waypoints+waypoints
