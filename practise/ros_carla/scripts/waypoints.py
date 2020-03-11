import time
import carla 
import sys
import networkx as nx
import matplotlib.pyplot as plt
import math
import random
import PID_Controller


length=0
# Function that makes nodes which are crossroads. 
def making_nodes_only_from_crossroad(list,graph):
    for i in list:
        if len(i.next(2.0))>1:
            graph.add_node(i)

# Function that returns the number of an element in the graph that is with the same value as wp. 
def get_element_from_graph(graph,wp):
    for i in list(graph.nodes):
        if compare_waypoints_instance(i,wp) is True:
            return i
    return None
    
# Function that draws a waypoint.
def drawing_waypoint(waypoint,drawing):
    drawing.draw_point(waypoint.transform.location)

# Function that compares two waypoints.
def compare_waypoints_instance(wp1,wp2):
    return (wp1.transform.location == wp2.transform.location) and (wp1.transform.rotation == wp2.transform.rotation)

# Function that compares the value between wp1 and wp2
def compare_waypoints_values(wp1,wp2):
    return math.fabs(wp1.transform.location.x-wp2.transform.location.x) + math.fabs(wp1.transform.location.y-wp2.transform.location.y) 
    + math.fabs(wp1.transform.location.z-wp2.transform.location.z) + math.fabs(wp1.transform.rotation.pitch-wp2.transform.rotation.pitch)
    + math.fabs(wp1.transform.rotation.yaw-wp2.transform.rotation.yaw) + math.fabs(wp1.transform.rotation.roll-wp2.transform.rotation.roll)


def compare_waypoint_and_vehicle(wp,vehicle):
    return math.fabs(wp.transform.location.x-vehicle.location.x) + math.fabs(wp.transform.location.y-vehicle.location.y) 
    + math.fabs(wp.transform.location.z-vehicle.location.z) + math.fabs(wp.transform.rotation.yaw-vehicle.rotation.yaw)


# Function that draws all waypoints in the map.
def drawing_all_waypoints(waypoints,drawing):
    for w in waypoints:
        drawing_waypoint(w,drawing)
        time.sleep(0.00001)

# Function that gets the next waypoint
def getting_waypoint(list):
    return list[0]

# Function that returns closest one waypoint to wp.
def getting_closest_one_wp(graph,wp):
    # moga da prowerya ot kakuw tip e wp i da go sukratya tazi boza
    min_value=float("inf")
    waypoint=wp
    for i in list(graph.nodes):
        value = math.fabs(i.transform.location.x-wp.transform.location.x) + math.fabs(i.transform.location.y-wp.transform.location.y) 
        + math.fabs(i.transform.location.z-wp.transform.location.z) + math.fabs(i.transform.rotation.pitch-wp.transform.rotation.pitch)
        + math.fabs(i.transform.rotation.yaw-wp.transform.rotation.yaw) + math.fabs(i.transform.rotation.roll-wp.transform.rotation.roll)
        
        if min_value > value:
            min_value=value
            waypoint=i
    return waypoint

# Function that returns closest waypoint to the car 
def getting_closest_one_wp_from_car(graph,cordinated):
    min_value=float("inf")
    waypoint=cordinated
    for i in list(graph.nodes):
        value = math.fabs(i.transform.location.x-cordinated.location.x) + math.fabs(i.transform.location.y-cordinated.location.y) 
        + math.fabs(i.transform.location.z-cordinated.location.z) + math.fabs(i.transform.rotation.pitch-cordinated.rotation.pitch)
        + math.fabs(i.transform.rotation.yaw-cordinated.rotation.yaw) + math.fabs(i.transform.rotation.roll-cordinated.rotation.roll)
        if min_value > value:
            min_value=value
            waypoint=i
    return waypoint


# Function that returns list of waypoints whcih conect the value waypoint
def connected_nodes_from_crossroad(waypoint):
    global length
    list_of_connected_crossroads=[]
    length=0
    for i in waypoint.next(2.0):
        while True:
            next_wp=i.next(2.0)
            if len(next_wp)>1:
                list_of_connected_crossroads.append(i)
                break
            i=getting_waypoint(i.next(2.0))
            length+=2
    return list_of_connected_crossroads

# Function that creates list between wp1 and wp2. 
def get_wp_that_is_for_crossroad(wp1,wp2):
    list=[]
    while True:
        next_wp=wp1.next(2.0)
        if len(next_wp)>1:
            if compare_waypoints_values(wp1,wp2)<10:
                list.append(wp2)
                return list
            else:
                return None
        list.append(wp1)
        wp1=getting_waypoint(wp1.next(2.0))
            
# Function that returns list of all waypoints between wp1 and wp2.
def get_all_waypoints_from_waypoint_to_waypoint(wp1,wp2):
    for i in wp1.next(2.0):
        looking_list = get_wp_that_is_for_crossroad(i,wp2)
        if looking_list is not None:
            return looking_list

# Function that connects nodes from the graph
def making_graph_from_crossroad(graph):
    global length
    for i in list(graph.nodes):
        crossroad=connected_nodes_from_crossroad(i)
        for k in crossroad:
            node=getting_closest_one_wp(graph,k)
            node=get_element_from_graph(graph,node)
            list_of_wp=get_all_waypoints_from_waypoint_to_waypoint(i,node)
            graph.add_edge(i,node,length=length,data=list_of_wp)

# Function that displays the graph with connections between nodes.
def showing_graph(graph):
    print(nx.info(graph))
    nx.draw(graph)
    plt.show()
 
# Function that chooses ranndom node from graph 
def choose_random(graph):
    return random.choice(list(graph.nodes()))

# Funtcion that prints the path 
def printing_path(list,drawing):
    for i in list:
        drawing_waypoint(i,drawing)
        # drawing_all_waypoints(i,drawing)

def getting_all_waypoints(path,graph):
    list=[]
    for i in range(len(path[:-1])):
        arr1=path[i]
        arr2=path[i+1]
        asdf=graph.get_edge_data(get_element_from_graph(graph,arr1),get_element_from_graph(graph,arr2))['data']
        list.append(asdf)
    return list

def converting_to_list_waypoints(list):
    listofwp=[]
    for i in list:
        for k in i:
            listofwp.append(k)
    return listofwp

def get_closest_wp_to_the_car(map,transform):
    return map.get_waypoint(transform.location)


def get_wp_to_crossroad(waypoint,list=[]):
    next_wp=waypoint.next(2.0)
    if len(next_wp)>1:
        list.append(waypoint)
        return list
    list.append(waypoint)
    waypoint=getting_waypoint(next_wp)
    return get_wp_to_crossroad(waypoint,list)

#  Main function that calls other functions.
def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(1.0) 
    world=client.get_world()
    blueprint_library = world.get_blueprint_library()

    drawing=world.debug
    map = world.get_map()
    waypoints=map.generate_waypoints(2.0)
    # 6831 elements in list
    graph=nx.DiGraph()

    making_nodes_only_from_crossroad(waypoints,graph)
    # 69
    making_graph_from_crossroad(graph)

    car=blueprint_library.find("vehicle.mercedes-benz.coupe")
    # transform = carla.Transform(carla.Location(x=20, y=0, z=3), carla.Rotation(yaw=-90))
    transform = carla.Transform(carla.Location(x=100, y=-7, z=3), carla.Rotation(yaw=180))
    vehicle = world.spawn_actor(car, transform)
    

    close_one=get_closest_wp_to_the_car(map,transform)
    wp_to_crossroad=get_wp_to_crossroad(close_one)

    path=nx.shortest_path(graph,getting_closest_one_wp_from_car(graph,wp_to_crossroad[-1].transform),choose_random(graph))
   
    waypoints= (converting_to_list_waypoints( getting_all_waypoints(path,graph) ))
    all_waypoints=wp_to_crossroad+waypoints

    printing_path(all_waypoints,drawing)

    
    control_throttle=PID_Controller.PID_Controller_Throttle(vehicle,15)
    control_steer=PID_Controller.PID_Controller_Steer(vehicle)
    pid_control=PID_Controller.Controller(vehicle,control_throttle,control_steer)
    waypoint=None
    while True:
        try:
            if waypoint is None:
                waypoint=getting_waypoint(all_waypoints)
                all_waypoints.remove(all_waypoints[0])
                # print(len(all_waypoints))
            # print(compare_waypoint_and_vehicle(waypoint,vehicle.get_transform()))
            if compare_waypoint_and_vehicle(waypoint,vehicle.get_transform()) < 2:
                waypoint=getting_waypoint(all_waypoints)
                all_waypoints.remove(all_waypoints[0])
                # print(len(all_waypoints))
            vehicle_control=carla.VehicleControl()
            pid_control.set_speed_on_car_and_steer(waypoint)
            vehicle_control.throttle=pid_control.throttle
            vehicle_control.steer=pid_control.steer
            print(vehicle_control.steer)
            # print(vehicle_control.throttle)
            vehicle.apply_control(vehicle_control)
        except IndexError:
            break
        except KeyboardInterrupt:
            break

    vehicle.destroy()

    # showing_graph(graph)


if __name__ == '__main__':
    main()

# 954 reda kod
# https://en.wikipedia.org/wiki/Yaw_(rotation)

