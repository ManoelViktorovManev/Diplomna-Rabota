import random

def get_smallest_element_from_list(list,set):
    min_value=list[0].distance
    element=list[0]
    for i in list:
        if i.distance<min_value and i.node in set:
            min_value=i.distance
            element=i
    return element

def get_element_from_list(list,element):
    for i in list:
        if i.node==element:
            return i 

class Node_for_Dijikstra:

    def __init__(self,node,distance=float("inf")):
        self.node=node
        self.distance=distance
    def __str__(self):
        return str(self.node.__str__()+str(self.distance))

class Graph:

    def __init__(self):
        self.nodes=[]
        self.edges={}

    def add_node(self,node):
        self.nodes.append(node)

    def get_node(self,node):
        for i in self.nodes:
            if self.compare_waypoints_instance(i,node) is True:
                return i
        return None

    def compare_waypoints_instance(self,wp1,wp2):
        return (wp1.transform.location == wp2.transform.location) and (wp1.transform.rotation == wp2.transform.rotation)

    def get_edge_data(self,node1,node2):
        for key in self.edges:
            if key == node1:
                for i in self.edges[key]:
                    if i[0]==node2:
                        return i[2]
    
    def get_edge_length(self,node1,node2):
        for key in self.edges:
            if key == node1:
                for i in self.edges[key]:
                    if i[0]==node2:
                        return i[1]

    def add_edge(self,node1,node2,length=0,data=[]):
        if node1 in self.edges:
            self.edges[node1].append([node2,length,data])
        else: 
            self.edges[node1]=([[node2,length,data]])
    
    def directed_neighbors(self,node):
        return self.edges[node]
    
    def random(self):
        return random.choice(list(self.nodes))

    def dijkistra(self,start_node,end_node):
        list_of_range_of_nodes=[]
        list_of_paths=[]
        unvisibled_set={x for x in self.nodes}
        for i in self.nodes:
            if i == start_node:
                list_of_range_of_nodes.append(Node_for_Dijikstra(i,0))
            else:
                list_of_range_of_nodes.append(Node_for_Dijikstra(i))
        current_node=get_smallest_element_from_list(list_of_range_of_nodes,unvisibled_set)
        while True:
            for i in self.directed_neighbors(current_node.node):
                neighbor_node=i[0]
                if neighbor_node in unvisibled_set:
                    
                    neighbor=get_element_from_list(list_of_range_of_nodes,neighbor_node)
                    if neighbor.distance>current_node.distance+i[1]:
                        neighbor.distance=current_node.distance+i[1]

            if end_node not in unvisibled_set:
                for i in list_of_paths:
                    if i[-1] == end_node:
                        return i
                return None
            else:
                if len(list_of_paths) == 0:
                    list_of_paths.append([current_node.node])
                else:
                    for i in list_of_paths:
                        for last_element in self.directed_neighbors(i[-1]):
                            neighbor=get_element_from_list(list_of_range_of_nodes,last_element[0])
                            if self.compare_waypoints_instance(neighbor.node,current_node.node):
                                new_list=[]
                                for k in i:
                                    new_list.append(k)
                                new_list.append(current_node.node)
                                list_of_paths.append(new_list)
                                break
            try:
                unvisibled_set.remove(current_node.node)
            except KeyError:
                return None
            current_node=get_smallest_element_from_list(list_of_range_of_nodes,unvisibled_set)
