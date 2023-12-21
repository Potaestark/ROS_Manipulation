import rospy
import cv2, math, sys, os
from ultralytics import YOLO
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from potaestark.srv import GetDepthCommand,GetDepthCommandRequest
from potaestark.srv import GoToPosition,GoToPositionResponse
directory = os.path.dirname(os.path.abspath(sys.argv[0]))
model = YOLO(directory + "/runs/detect/train0/weights/last.pt")

class ObjectDetection(object):

    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node("Chicken_detect", anonymous=True)
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.update_frame_callback)
        rospy.wait_for_message("/camera/rgb/image_raw", Image)
        #self.get_depth_command= rospy.ServiceProxy('/vision/get_depth',GetDepthComand)
        self.get_position = rospy.ServiceProxy('/vision/get_depth', GetDepthCommand)
        rospy.Service('/position/chicken', GoToPosition, self.main)
        rospy.loginfo('Chicken Position Ready...')

    def update_frame_callback(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8") 

    def main(self, data): 

        frame = self.image
        results = model(frame)
        result = results[0]
        bboxes = np.array(result.boxes.xyxy.cpu(), dtype = "int")
        classes = np.array(result.boxes.cls.cpu(), dtype = "int")
        keypoints = np.array(result.keypoints.xy.cpu(), dtype = "int")
        chicken_positions = []
        keypoints_chicken = []

        # Chicken Detection
        for keypoint, cls, bbox in zip(keypoints, classes, bboxes):

            (x, y, x2, y2) = bbox
            (k1,k2,k3) = keypoint

            
            print(f'Keypoint: {keypoint}') # np [[x1 y1][x2 y2] ... [xn yn]]
            xk1,yk1 = k1
            xk2,yk2 = k2
            xk3,yk3 = k3
            k4 = np.array([(xk3+xk2)/2, (yk2+yk3)/2], dtype=np.int64) # [x, y]
            print("x = ",x,"y = ",y,"x = 2",x2,"y2 = ",y2)
            print("k1 =", k1,"k2 =", k2,"k3 =", k3)
            print(cls)
            keypoints_chicken.append(k4)
            cv2.putText(frame, str(cls), (x,y-5), cv2.FONT_HERSHEY_PLAIN, 1, (0,0,255), 2)

            cv2.rectangle(frame, (x,y), (x2,y2), (0,0,255), 2)

            cv2.circle(frame, k1, 0, (255,0,0), 5)
            cv2.circle(frame, k2, 0, (255,50,50), 5)
            cv2.circle(frame, k3, 0, (255,100,100), 5) 
            cv2.circle(frame, k4, 0, (255,255,255), 5)

            for keypoint_chicken in keypoints_chicken:
                request = GetDepthCommandRequest()
                request.position = keypoint_chicken
                request.type = 'chicken'
                print(f'Request from chicken: {request}')

                position = self.get_position(request)
                chicken_positions.append(position)

                rospy.sleep(0.5)
            break

        response = GoToPositionResponse()
        rospy.sleep(3)
        
        if chicken_positions == []:
            rospy.loginfo('Into the Fail state')
            response.success = False
            response.positions = []
            return response
        
        if not math.isnan(chicken_positions[0].real_position.x):
            rospy.loginfo(f'Into the Success state with chicken position: {chicken_positions}')

            response.success = True
            response.positions = chicken_positions[0]
            # response.hook_position = self.hook_position_list
            print('FINISH!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
            return response
        response.success = False
        response.positions = []
        return response
            
if __name__ == "__main__":
    obj = ObjectDetection()
    rospy.spin()