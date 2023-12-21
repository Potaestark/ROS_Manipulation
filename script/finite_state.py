import rospy
import smach
import smach_ros
import math
import numpy as np
from std_srvs.srv import *
from potaestark.srv import GoToPosition, GoToPositionRequest
from potaestark.srv import manipulation,manipulationRequest

class Find_hook(smach.State):
    def __init__(self, outcomes=['success', 'fail']):
        super().__init__(outcomes)
        self.get_hook_position = rospy.ServiceProxy('/position/hook', GoToPosition)

    def execute(self, ud):
        rospy.loginfo("Executing state Find Chicken")
        request = GoToPositionRequest()
        request.position_name = 'one'
        response = self.get_hook_position(request)
        rospy.loginfo(f"Request from Find_hook: {request}")

        if response.success:
            rospy.loginfo(f"Hook Detect x = {response.positions[0].x} y = {response.positions[0].y} z = {response.positions[0].z}")
            
            global hook_x, hook_y, hook_z
            hook_x = response.positions[0].x
            hook_y = response.positions[0].y
            hook_z = response.positions[0].z
            return 'success'
        else:
            rospy.logerr('fail')
            return 'fail'
        
class Navigation(smach.State):
    def __init__(self, area, outcomes=['success', 'fail']):
        super().__init__(outcomes)
        self.area = area
        rospy.loginfo(f"Nav to area = {self.area}")

    def execute(self, ud):
        command_success =True
        if command_success:
            rospy.loginfo(f"Area = {self.area}")
            return 'success'
        else:
            rospy.logerr('Fail to plan')
            return 'fail'

class Find_chicken(smach.State):
    def __init__(self, outcomes=['success', 'fail']):
        super().__init__(outcomes)
        self.get_chicken_position = rospy.ServiceProxy('/position/chicken', GoToPosition)

    def execute(self, ud):
        rospy.loginfo("Executing state Find Chicken")
        request = GoToPositionRequest()
        request.position_name = 'one'
        response = self.get_chicken_position(request)
        rospy.loginfo(f" {request}")
        if response.success:
            
            rospy.loginfo(f"Chicken Detect [k1] x = {response.positions[0].x} y = {response.positions[0].y} z = {response.positions[0].z}")
            rospy.loginfo(f"Chicken Detect [k2] x = {response.positions[1].x} y = {response.positions[1].y} z = {response.positions[1].z}")
            rospy.loginfo(f"Chicken Detect [k3] x = {response.positions[2].x} y = {response.positions[2].y} z = {response.positions[2].z}")
            rospy.loginfo(f"Chicken Detect [k4] x = {response.positions[3].x} y = {response.positions[3].y} z = {response.positions[3].z}") # middle chick
            
            global x,z,y, theta
            x = response.positions[3].x
            y = response.positions[3].y
            z = response.positions[3].z
            theta = np.arctan2(response.positions[2].y - response.positions[1].y, response.positions[2].x - response.positions[1].x)
            rospy.loginfo(f'(arctan {response.positions[2].y} - {response.positions[1].y}) / ({response.positions[2].x } - {response.positions[1].x})') 
            rospy.loginfo(f'theta: {theta}')
            # hooks = response.hook_position
            # rospy.loginfo(f'Hook position: {hooks}')
            return 'success'
        else:
            rospy.logerr('fail')
            return 'fail'
        

class MoveitCommand(smach.State):
    def __init__(self, command, outcomes=['success', 'fail']):
        super().__init__(outcomes)
        self.send_command = rospy.ServiceProxy('/moveit/send_command', manipulation)
        rospy.loginfo('Executing state Moveit plan"')
        self.command = command
        rospy.loginfo(f'command: {self.command}')
        
        '''
        command 
        -------------------
        (1) plan
        (2) pick object
        (3) place object
        '''

    def execute(self, ud):
        
        if self.command == 1:
            rospy.loginfo('Enter moveit with command 1 execute:')
            rospy.loginfo('Pick chicken')
            request = manipulationRequest()
            request.command = self.command
            request.x = x
            request.y = y
            request.z = z
            request.theta = theta
            rospy.loginfo(f'theta: {theta}')
            command_success = self.send_command(request)

            if command_success.success:
                rospy.loginfo(f"Plan for Chicken at x = {x} y = {y} z = {z}")
                return 'success'
            else:
                rospy.logerr('Fail to plan')
                return 'fail'
        elif self.command == 2:
            rospy.loginfo('Enter moveit with command 2 execute:')
            rospy.loginfo('Place chicken')
            request = manipulationRequest()
            request.command = self.command
            
            request.x = hook_x
            request.y = hook_y
            request.z = hook_z
            request.theta = math.pi/2
            rospy.loginfo(f'theta: {theta}')
            command_success = self.send_command(request)

            if command_success.success:
                rospy.loginfo(f"Plan for Chicken at x = {x} y = {y} z = {z}")
                return 'success'
            else:
                rospy.logerr('Fail to plan')
                return 'fail'


class Loop(smach.State):
    def __init__(self, outcomes=['success', 'fail']):
        super().__init__(outcomes)
        self.count = 10

    def execute(self, ud):
        self.count -= 1
        if self.count == 0:
            return 'success'
        return 'fail'

class state_list(object):
    def __init__(self) -> None:
        rospy.init_node('robot_state', anonymous=True)
        sm = smach.StateMachine(outcomes=['---finish---'])

        with sm:
            smach.StateMachine.add('navigation_to_station', Navigation(area=1), 
                                transitions={'success':'find_chicken', 'fail':'navigation_to_station'}) # ไป station 1
            
            smach.StateMachine.add('find_chicken', Find_chicken(), 
                                transitions={'success':'navigation_forward_50_cm', 'fail':'find_chicken'}) # หาไก่
            
            smach.StateMachine.add('navigation_forward_50_cm', Navigation(area=2), 
                                transitions={'success':'moveit_pick', 'fail':'navigation_forward_50_cm'}) # ไปข้างหน้า 50 cm
            
            smach.StateMachine.add('moveit_pick', MoveitCommand(command=1), 
                                transitions={'success':'navigation_backward_50_cm', 'fail':'moveit_pick'}) # จับไก่
            
            smach.StateMachine.add('navigation_backward_50_cm', Navigation(area=1), 
                                transitions={'success':'find_hook', 'fail':'navigation_backward_50_cm'}) # ถอยกลับ 50 cm
            
            smach.StateMachine.add('find_hook', Find_hook(), 
                                transitions={'success':'moveit_place', 'fail':'find_hook'}) # หา hook
            
            smach.StateMachine.add('moveit_place', MoveitCommand(command=2), 
                                transitions={'success':'check_loop', 'fail':'moveit_place'}) # แขวนไก่
            
            smach.StateMachine.add('check_loop', Loop(), 
                                transitions={'success':'---finish---', 'fail':'find_chicken'}) # แขวนไก่
    
        outcome = sm.execute()

if __name__ == "__main__":
    state_list()