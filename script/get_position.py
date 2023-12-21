#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from potaestark.srv import GetDepthCommand,GetDepthCommandResponse
from cv_bridge import CvBridge
import pickle, os, sys, pickle
import tf


######################### MANUAL ################################
# Check image size in : self.img_size is (720,1280) by default.
# Set roi size in : self.roi_size is 4 by default.
#################################################################


class GetDepth(object):
    def __init__(self):
        
        ########## (BASIC SETTING) ##########
        self.roi_size = 4
        self.img_size = (480,640)
        #####################################

        self.bridge = CvBridge()
        rospy.init_node("get_depth", anonymous=True)
        self.active = False
        self.shutdown = False
        self.lower_depth_bound = 40 #cm REF from Min-Z in document link: https://www.intelrealsense.com/depth-camera-d415/
        self.min_moving_avg_count = 3
        self.img_sub = rospy.Subscriber("/camera/depth_registered/image",Image,self.callback,queue_size=10)
        self.get_depth_ser = rospy.Service("/vision/get_depth",GetDepthCommand,self.get_depth)
        #self.pub = rospy.Publisher('/px_py_depth', px_py_depth, queue_size=10)
        self.position = None
        self.br = tf.TransformBroadcaster()
        self.ans = []

        # Camera Information
        directory = os.path.dirname(os.path.abspath(sys.argv[0]))
        with open(directory + '/cam_calibration/cameraMatrix.pkl', 'rb') as file:
            camera_info = pickle.load(file)
        
        self.fx = camera_info[0][0]
        self.fy = camera_info[1][1]
        self.cx = camera_info[0][2]
        self.cy = camera_info[1][2]
        self.theta = np.radians(29.1)  # camera angle in radian

        rospy.loginfo("GetDepth node is started!")

    def callback(self,data):
        if not self.active or (self.position is None):
            return 1
        data = self.bridge.imgmsg_to_cv2(data,desired_encoding='passthrough')
        depth_array = np.array(data, dtype=np.float32)
        if depth_array is None or self.position is None:
            return 1
        react = depth_array[self.position[0]-self.roi_size:self.position[0]+self.roi_size , self.position[1]-self.roi_size:self.position[1]+self.roi_size]
        print(react.mean()*100)
        self.ans.append(react.mean()*100)

    def get_depth(self,data):
        rospy.loginfo('Getting Depth')
        if len(data.position) != 2:
            rospy.logerr("Position error!")
            return -10 # Request error
        data.position = [data.position[1],data.position[0]]
        if data.position[0] > self.img_size[0]-self.roi_size or data.position[0] < self.roi_size or data.position[1] > self.img_size[1]-self.roi_size or data.position[1] < self.roi_size:
            rospy.logerr("Position out of bound!")
            return -11
        self.position = data.position
        self.active = True
        ans = self.ans
        while len(self.ans) < self.min_moving_avg_count or len(ans) < self.min_moving_avg_count:
            ans = self.ans
        
        ans = np.array(ans)
        ans = ans.mean()
        if ans < self.lower_depth_bound:
            self.ans = []
            self.position = None
            self.active = False
            rospy.logerr("Depth out of bound!")
            return -12
        self.ans = []
        self.position = None
        self.active = False
        print(f"for position px = {data.position[0]} py = {data.position[1]}] Depth : {ans}")
        px = data.position[0]
        py = data.position[1]
        """
        x = ans
        z = (px+self.img_size[1]/2-self.cx) * x / self.fx
        y = (py+self.img_size[0]/2-self.cy) * x / self.fy
        
        rotation_matrix = np.array([[np.cos(self.theta),  0, np.sin(self.theta)],
                                    [          0,             1,          0    ],
                                    [-np.cos(self.theta), 0, np.sin(self.theta)]])
        
        camera_coordination = np.array([[x_c],[y_c],[z_c]])
        
        world_coordination = np.dot(rotation_matrix, camera_coordination)
        x = world_coordination[0]
        y = world_coordination[1]
        z = world_coordination[2]
        """
        
        if data.type == 'chicken':
            rospy.loginfo(f'Getting Chicken')
            # x = (ans * np.cos(np.deg2rad(29.1)) - 50)
            x = ans
            y = (-0.0019075*data.position[1]*ans) + (0.610403*ans) 
            # z = ((-0.00190765*data.position[1]*ans + 0.458892*ans) + 68) * np.cos(np.deg2rad(29.1)) * 0.01  # unit(m)
            z = 10

            """
            x = ans
            y = (-0.00190751 * data.position[0] * ans) + (0.610403 * ans)  
            z = (-0.00190765 * data.position[1] * ans) + (0.457836 * ans)
            """
        elif data.type == 'hook':
            rospy.loginfo(f'Getting Chicken')
            x = 52.47
            y = (-0.0019075*data.position[1]*ans) + (0.610403*ans) 
            z = 51.5
        
        
        self.br.sendTransform((x*0.01, y*0.01, z*0.01),
                              tf.transformations.quaternion_from_euler(0.0, 0.0, 1.0),
                              rospy.Time.now(),
                              'chicken',
                              'camera_link')
        rospy.loginfo(f' x = {x} y = {y} z = {z}')
        response = GetDepthCommandResponse()
        response.real_position.x = x
        response.real_position.y = y
        response.real_position.z = z


        return response


if __name__ == "__main__":
    d = GetDepth()
    
    while not rospy.is_shutdown() and not d.shutdown:
        pass

    cv2.destroyAllWindows()