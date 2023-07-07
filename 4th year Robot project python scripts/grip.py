# Purpose: To instruct the braccio arms to grip the bones.

#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import time

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes

class BraccioGrip(object):
    def __init__(self):
        global arm_number, arm_group, arm_group2, initiate_grip, length_scale, theta, braccio1_pose_pub, braccio2_pose_pub, get_objects, arm1_bone_number, arm2_bone_number
        super(BraccioGrip, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('grip', anonymous=True)

       # Publishers for showing trajectory and poses of arms.
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        braccio1_pose_pub = rospy.Publisher('/braccio1_pose', PoseStamped, queue_size=10)       
        braccio2_pose_pub = rospy.Publisher('/braccio2_pose', PoseStamped, queue_size=10)                               
        
        # Path planned subscriber's role is to tell braccio_gui.py script if a path has or has not been planned.
        rospy.Subscriber('/path_planned', MoveItErrorCodes, self.plan_callback)

        # Arm number parameter decides which arm will approach the bone first.
        arm_number = rospy.get_param('/arm_number')
        
        # Creates instances of MoveIt classes for controlling the braccio arms.
        self.robot = moveit_commander.RobotCommander()    
        self.arm_group = moveit_commander.MoveGroupCommander("braccio_arm")
        self.arm_group2 = moveit_commander.MoveGroupCommander("braccio_arm2")
        self.eef_link = arm_group.get_end_effector_link()
        self.eef_link2 = arm_group2.get_end_effector_link()
        self.gripper_group = moveit_commander.MoveGroupCommander("braccio_gripper")
        self.gripper_group2 = moveit_commander.MoveGroupCommander("braccio_gripper2") 
        self.scene = moveit_commander.PlanningSceneInterface()

        self.arm_group.set_max_velocity_scaling_factor(1)
        self.arm_group2.set_max_velocity_scaling_factor(1)
        self.gripper_group.set_max_velocity_scaling_factor(1)
        self.gripper_group2.set_max_velocity_scaling_factor(1)

        # Obtains all objects in the planning scene.
        get_objects = self.scene.get_objects()

    def plan_callback(self, data):
        error = data.val
        if error != 1:
            rospy.set_param('path_planned', False)
        if error == 1:
            rospy.set_param('path_planned', True)

    def gripper_close(self, gripper_number, joint_val=1.2):
        if gripper_number == 1:
            self.go_gripper(1, joint_val)
        elif gripper_number == 2:
            self.go_gripper(2, joint_val)
        else:
            print('Invalid gripper number, only 1 or 2 accepted!')

    def gripper_open(self, gripper_number, joint_val=0.2):
        if gripper_number == 1:
            self.go_gripper(1, joint_val)
        elif gripper_number == 2:
            self.go_gripper(2, joint_val)
        else:
            print('Invalid gripper number, only 1 or 2 accepted!')

    def go_gripper(self, gripper_number, joint_val):

    # Depending on joint values, the function is responsible for opening or closing the gripper.
        if gripper_number == 1:
            joint_goal = self.gripper_group.get_current_joint_values()
            joint_goal[0] = joint_val
            joint_goal[1] = joint_val
            print(joint_goal)
            self.gripper_group.go(joint_goal, wait=True)
            self.gripper_group.stop()
            
        elif gripper_number == 2:
            joint_goal = self.gripper_group2.get_current_joint_values()
            joint_goal[0] = joint_val
            joint_goal[1] = joint_val
            print(joint_goal)
            self.gripper_group2.go(joint_goal, wait=True)
            self.gripper_group2.stop()
        else:
            print('Invalid gripper number, only 1 or 2 accepted!')

    def braccio1(self, bone_number):
        global braccio1_pose

        # Open gripper.
        self.gripper_open(1)

        # Retrieve bone pose and length.
        bone_pose = get_objects[bone_number].pose
        bone_length = get_objects[bone_number].primitives[0].dimensions[0]

        # Lower the gripper by 2cm.
        braccio1_pose = PoseStamped()
        braccio1_pose.header.frame_id = "odom"
        braccio1_pose.pose = self.arm_group.get_current_pose().pose
        braccio1_pose.pose.position.z = bone_pose.position.z + 0.08
        
        # Publish and plan route to grip position.
        braccio1_pose_pub.publish(braccio1_pose)
        braccio1_pose = self.arm_group.set_pose_target(braccio1_pose,"wrist_roll_link")   
        self.arm_group.plan(braccio1_pose) 

    def braccio2(self, bone_number):
        global braccio2_pose

        # Open gripper.
        self.gripper_open(2)

        # Retrieve bone pose and length.
        bone_pose2 = get_objects[bone_number].pose
        bone_length2 = get_objects[bone_number].primitives[0].dimensions[0]

        # Lower the gripper by 2cm.
        braccio2_pose = PoseStamped()
        braccio2_pose.header.frame_id = "odom"
        braccio2_pose.pose = self.arm_group2.get_current_pose().pose
        braccio2_pose.pose.position.z = bone_pose2.position.z + 0.08

        # Publish and plan route to grip position.
        braccio2_pose_pub.publish(braccio2_pose)
        braccio2_pose = self.arm_group2.set_pose_target(braccio2_pose,"wrist_roll_link2")
        self.arm_group2.plan(braccio2_pose)

def main():
    movement = BraccioGrip()

    if arm_number == 1:

        # Plan route to the grip pose for the Braccio arm 1.
        movement.braccio1(arm1_bone_number)

        while True:
        # If the user approves, Braccio arm 1 grips it's assigned bone.
            time.sleep(0.5)
            initiate_grip = rospy.get_param('/initiate_grip')
            print('Polling initiate grip')
            if initiate_grip == True:
                break
          
        arm_group.go(braccio1_pose) 
        movement.gripper_close(1)
        movement.scene.attach_cylinder('wrist_roll_link', arm1_bone_number)

        time.sleep(5)
        rospy.set_param('/grip_complete', True)
        print('grip completed')
      
    if arm_number == 2:

        # Plan route to the grip pose for the Braccio arm 2.
        movement.braccio2(arm2_bone_number)

        while True:
        # If the user approves, Braccio arm 2 grips it's assigned bone.
            time.sleep(0.5)
            initiate_grip = rospy.get_param('/initiate_grip')
            print('Polling initiate grip')
        
            if initiate_grip == True:
                break
          
        arm_group2.go(braccio2_pose) 
        movement.gripper_close(2)
        movement.scene.attach_cylinder('wrist_roll_link2', arm2_bone_number)
        
        time.sleep(5)
        rospy.set_param('/grip_complete', True)
        print('grip completed')

if __name__ == '__main__':
  main()