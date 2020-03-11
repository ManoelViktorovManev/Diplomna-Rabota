import rospy
import math
try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')
from sensor_msgs.msg import Image,CameraInfo

# Function that publishes images in the topic '/camera/rgb/image'. Using rViz, it can visualize the images that are published.
def publishing(pub_image,pub_camera,image,type_of_camera):
    if type_of_camera is 1: 
        image.convert(carla.ColorConverter.Depth)
    elif type_of_camera is 2:
        image.convert(carla.ColorConverter.CityScapesPalette)
    array = np.frombuffer(image.raw_data, dtype=np.uint8)
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    img=Image()
    infomsg=CameraInfo()

    img.header.stamp=rospy.Time.now()
    img.header.frame_id='base'
    img.height=infomsg.height=image.height
    img.width=infomsg.width=image.width
    img.encoding="rgb8"
    img.step=img.width*3*1
    st1=array.tostring()
    img.data=st1

    cx=infomsg.width/2.0
    cy=infomsg.height/2.0
    fx=fy=infomsg.width/(2.0 * math.tan(image.fov* math.pi / 360.0))
    infomsg.K=[ fx, 0,cx,
                            0, fy, cy,
                            0, 0, 1 ]
    infomsg.P=[ fx, 0, cx, 0,
                            0, fy, cy, 0,
                            0, 0, 1, 0 ]
    infomsg.R=[1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
    infomsg.D = [0,0,0,0,0]
    infomsg.binning_x = 0
    infomsg.binning_y = 0
    infomsg.distortion_model = "plumb_bob"
    infomsg.header=img.header

    pub_image.publish(img)
    pub_camera.publish(infomsg)