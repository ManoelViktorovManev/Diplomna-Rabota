#!/usr/bin/env python
import carla
import read_data
import rospy
import rosbag
import sys
from msg_folder.msg import MyRosMsg

client = carla.Client('localhost', 2000)
client.set_timeout(10.0) 
world=client.get_world()
blueprint_library = world.get_blueprint_library()
vehicle = None

autopilot=False
button_autopilot=read_data.Pushing_Buton([])
control_car=carla.VehicleControl()
vehicle_type_set=False


rosbag_file=sys.argv[1]
# print (rosbag_file)
bag=rosbag.Bag(rosbag_file)
text=''
for msg in bag.read_messages():
    text=str(msg)
    break
text=text[text.find("name"):]
name=text[text.find('"')+1:text.find('",')]

car=blueprint_library.find(name)
transform = carla.Transform(carla.Location(x=30, y=0, z=3), carla.Rotation(yaw=180))
vehicle=world.spawn_actor(car,transform)
control_car.brake=1.0
vehicle.apply_control(control_car)

def callback(data):
    global autopilot
    global button_autopilot
    global vehicle
    # global vehicle_type_set
    # if vehicle_type_set is False:
    #     vehicle_type_set = True
    #     # path = os.path.join(os.path.expanduser('~'), 'catkin_ws','src','ros_carla','scripts', 'vehicle_type.txt')
    #     # f=open(path,'w')
    #     car=blueprint_library.find(f.)
    #     car=blueprint_library.find(data.vehicle_name)
    #     transform = carla.Transform(carla.Location(x=30, y=0, z=3), carla.Rotation(yaw=180))
    #     vehicle=world.spawn_actor(car,transform)
    #     control_car.brake=1.0
    #     vehicle.apply_control(control_car)


    if data.is_exit==True:
        vehicle.destroy()
        print("The console stoped :)")
        rospy.signal_shutdown("exit")
    if data.is_autopilot==True:
        if len(button_autopilot.get_list())!=0:
            if (button_autopilot.is_diferent_date()):
                spawning_actors.deleteAllelements(button_autopilot.get_list())
        if len(button_autopilot.get_list())==0:       
            button_autopilot.add_new_date_now()
            button_autopilot.set_state_of_pushed_button(True)

        if button_autopilot.get_button() is True:
            if autopilot is False:
                vehicle.set_autopilot(True)
                autopilot=True
            elif autopilot is True:
                autopilot=False
                vehicle.set_autopilot(False)
            button_autopilot.set_state_of_pushed_button(False)
    if autopilot == False:
        control_car.reverse=data.is_reverse
        control_car.brake=data.breaks
        control_car.throttle=data.throttle
        control_car.steer=data.steer
        vehicle.apply_control(control_car)

def main():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("controler_info", MyRosMsg, callback)
    rospy.spin()
if __name__ == '__main__':
    main()
