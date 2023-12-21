#!/usr/bin/env python3

import sys, math

import rospy
import numpy as np
import moveit_commander
import tf
from tf.transformations import quaternion_from_euler

# import msg
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject
from potaestark.srv import *
# from ice.msg import px_py_depth


SCENE = moveit_commander.PlanningSceneInterface()
OBJECT_POSITIONS = {'target_1': [0.05, 0.35, 0.3]}
FRAME_ID = 'base_link'
PICK_ORIENTATION_EULER = [-math.pi/2, 0, 0]
PLACE_ORIENTATION_EULER = [-math.pi/2, 0, -math.pi/2]

class Robot:
    def __init__(self):
        rospy.init_node('pick_place_ar3')

        rospy.Service('/moveit/send_command', manipulation, self.callback)
        # self.get_pos = rospy.ServiceProxy('/moveit/get_position', GetPosition)

        # init moveit commander
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.loginfo('Initialize moveit node')

        # self.listener = tf.TransformListener()
        
        self.robot = moveit_commander.RobotCommander('robot_description')
        self.arm = moveit_commander.MoveGroupCommander('arm_group', ns=rospy.get_namespace())
        # self.gripper = robot.get_joint('gripper')
        self.gripper = None

        self.arm.set_num_planning_attempts(45)

    # callback
    def callback(self, request):
        #  Plan for robot arm
        if request.command == 1:
            response = manipulationResponse()
            rospy.loginfo('Getting callback from moveit with command 1')
            self.move_arm(request)
            self.move_joint_6(request.theta)
            rospy.loginfo(f'Planning success with x = {request.x*0.01} y = {request.y*0.01} z = {request.z*0.01} theta = {request.theta}')

            response.success = True
            return response
            
        # Execute robot arm
        elif request.command == 2:
            pass
        elif request.command == 3:
            pass
    
    # open gripper
    def open_gripper(self):
        return self.gripper.move(self.gripper.max_bound() * 0.9, True)

    # close gripper
    def close_gripper(self):
        return self.gripper.move(self.gripper.max_bound() * 0.15, True)

    def move_arm(self, request, tolerance=0.001):
        response = manipulationResponse()
        pose = Pose()
        pose.position.x = request.x*0.01
        pose.position.y = request.y*0.01
        pose.position.z = request.z*0.01
        pose.orientation.w = 1.0
        self.arm.set_pose_target(pose)
        self.arm.set_goal_position_tolerance(tolerance)

        success, plan, planning_time, error_code = self.arm.plan()
        
        if not success:
            rospy.logerr('Fail to plan')
            response.success = False
            return response
        
        rospy.loginfo('Success to plan')
        rospy.loginfo(f'Plan: {plan}')
        rospy.loginfo(f'Planning Time: {planning_time}')
        rospy.loginfo(f'Error: {error_code}')
        for point in plan.joint_trajectory.points:
            rospy.loginfo(point.positions)

        self.arm.execute(plan)
        print('Arm moved!!!')
    
    def move_joint_6(self, theta):
        joint_goal = self.arm.get_current_joint_values()

        joint_goal[5] = theta
        
        print('Gripper moved!!!')
        self.arm.go(joint_goal, wait=True)
        self.arm.stop()


    # pick object
    def pick(self, target):
        pose = Pose()
        pose.position.x = OBJECT_POSITIONS[target][0]
        pose.position.y = OBJECT_POSITIONS[target][1] - 0.1
        pose.position.z = OBJECT_POSITIONS[target][2]
        orientation = quaternion_from_euler(*PICK_ORIENTATION_EULER)
        pose.orientation.x = orientation[0]
        pose.orientation.y = orientation[1]
        pose.orientation.z = orientation[2]
        pose.orientation.w = orientation[3]
        self.reach_pose(pose)
        # self.open_gripper()
        pose.position.y += 0.1
        self.reach_pose(pose)
        # self.close_gripper()
        self.arm.attach_object(target)

    # place object
    def place(self, target):
        pose = Pose()
        pose.position.x = OBJECT_POSITIONS[target][1]
        pose.position.y = OBJECT_POSITIONS[target][0]
        pose.position.z = OBJECT_POSITIONS[target][2]
        orientation = quaternion_from_euler(*PLACE_ORIENTATION_EULER)
        pose.orientation.x = orientation[0]
        pose.orientation.y = orientation[1]
        pose.orientation.z = orientation[2]
        pose.orientation.w = orientation[3]

        self.reach_pose(pose)
        # self.open_gripper()
        self.reach_pose(pose)
        self.arm.detach_object(target)

    
        

# create collision object
def create_collision_object(id, dimensions, pose):
    object = CollisionObject()
    object.id = id
    object.header.frame_id = FRAME_ID

    solid = SolidPrimitive()
    solid.type = solid.BOX
    solid.dimensions = dimensions
    object.primitives = [solid]

    object_pose = Pose()
    object_pose.position.x = pose[0]
    object_pose.position.y = pose[1]
    object_pose.position.z = pose[2]

    object.primitive_poses = [object_pose]
    object.operation = object.ADD
    return object

# add collision objects to the scene
def add_collision_objects():
    floor_limit = create_collision_object(id='floor_limit',
                                        dimensions=[10, 10, 0.2],
                                        pose=[0, 0, -0.1])
    """
    table_1 = create_collision_object(id='table_1',
                                    dimensions=[0.3, 0.6, 0.2],
                                    pose=[0.45, 0.3, 0.1])
    table_2 = create_collision_object(id='table_2',
                                    dimensions=[0.3, 0.3, 0.2],
                                    pose=[0.15, 0.45, 0.1])
    """
                    
    target_1 = create_collision_object(id='target_1',
                                    dimensions=[0.02, 0.02, 0.2],
                                    pose=[0.05, 0.35, 0.3])


    # SCENE.add_object(floor_limit)
    # SCENE.add_object(table_1)
    # SCENE.add_object(table_2)
    SCENE.add_object(target_1)

if __name__ == '__main__':
    my_robot = Robot()

    add_collision_objects()

    """
    rospy.sleep(1.0)

    # pick object
    my_robot.pick('target_1')
    rospy.loginfo('Pick up object')
    rospy.sleep(1.0)

    # place object
    my_robot.place('target_1')
    rospy.loginfo('Place object')
    rospy.sleep(1.0)
    rospy.loginfo('End of programme')
    """
    rospy.spin()
