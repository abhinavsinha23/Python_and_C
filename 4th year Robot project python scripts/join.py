# Purpose: To reduce the clearance between the bones to 5mm.

#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped
import time
import numpy

from tf.transformations import quaternion_matrix, euler_from_matrix
from math import pi, cos, sin
from moveit_msgs.msg import MoveItErrorCodes

class BraccioJoin(object):
    def __init__(self):
        global arm_number, arm_group, arm_group2, braccio1_pose_pub, get_objects, arm1_bone_number, arm2_bone_number
        super(BraccioJoin, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('join', anonymous=True)

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
        
        # Obtains all attached objects in the planning scene.
        get_objects = self.scene.get_attached_objects()

    def plan_callback(self, data):
        error = data.val
        if error != 1:
            rospy.set_param('path_planned', False)
        if error == 1:
            rospy.set_param('path_planned', True)

    def braccio1(self, arm1_bone_number, arm2_bone_number):
        global braccio1_pose
        
        # Retrieve position and length of bones.
        bone_pose = get_objects[arm1_bone_number].object.pose
        bone_pose2 = get_objects[arm2_bone_number].object.pose
        bone_height = get_objects[arm1_bone_number].object.primitives[0].dimensions[0]
        bone_height2 = get_objects[arm2_bone_number].object.primitives[0].dimensions[0]

        # Retrieve current pose of Braccio arm 1.   
        braccio1_pose = PoseStamped()
        braccio1_pose.header.frame_id = "odom"
        braccio1_pose.pose = self.arm_group.get_current_pose().pose

        # Retrieve current pose of Braccio arm 2.   
        braccio2_pose = PoseStamped()
        braccio2_pose.header.frame_id = "odom"
        braccio2_pose.pose = self.arm_group2.get_current_pose().pose

        # Convert bone 1 position into a global position.
        bone_pose.position.x = braccio1_pose.pose.position.x - bone_pose.position.y
        bone_pose.position.y = braccio1_pose.pose.position.y + bone_pose.position.z
        bone_pose.position.z = braccio1_pose.pose.position.z + bone_pose.position.x

        # Convert bone 2 pose into a global pose.
        bone_pose2.position.x = braccio2_pose.pose.position.x - bone_pose2.position.y
        bone_pose2.position.y = braccio2_pose.pose.position.y + bone_pose2.position.z
        bone_pose2.position.z = braccio2_pose.pose.position.z - bone_pose2.position.x
        q = braccio2_pose.pose.orientation
        braccio2_mat_4 = quaternion_matrix([q.x, q.y, q.z, q.w])
        rel_bone2_mat_4 = quaternion_matrix([bone_pose2.orientation.x, bone_pose2.orientation.y, bone_pose2.orientation.z, bone_pose2.orientation.w])
        act_bone2_mat_4 = numpy.dot(braccio2_mat_4, rel_bone2_mat_4)
        
        # Extract yaw angle of bone 2.
        e = euler_from_matrix(act_bone2_mat_4)
        bone2_theta = e[2]
        
        # Determine euclidean distance between grip position and midpoint of bone.
        length1 = ((braccio1_pose.pose.position.x-bone_pose.position.x) ** 2 + (bone_pose.position.y-braccio1_pose.pose.position.y) ** 2) ** 0.5
        length2 = ((bone_pose2.position.x-braccio2_pose.pose.position.x) ** 2 + (braccio2_pose.pose.position.y - bone_pose2.position.y) ** 2) ** 0.5
         
        # Calculate joined position.
        braccio1_pose.pose.position.x = braccio2_pose.pose.position.x - ((bone_height2 * 0.5 - length2) + 0.005 + (bone_height * 0.5 - length1)) * cos(bone2_theta)
        braccio1_pose.pose.position.y = braccio2_pose.pose.position.y + ((bone_height2 * 0.5 - length2) + 0.005 + (bone_height * 0.5 - length1)) * sin(-bone2_theta)
        
        # Publish and plan the route to the joined pose.
        braccio1_pose_pub.publish(braccio1_pose)
        braccio1_pose = self.arm_group.set_pose_target(braccio1_pose, "wrist_roll_link")
        self.arm_group.plan(braccio1_pose)

def main():
    movement = BraccioJoin()

    # Plan route to joined position.
    movement.braccio1(arm1_bone_number, arm2_bone_number)

    while True:
        time.sleep(0.5)
        # If the user approves, Braccio arm 1 joins bone 1 with bone 2.
        initiate_join = rospy.get_param('/initiate_join')
        print('Polling initiate join')
        
        if initiate_join == True:
            break
        
    arm_group.go(braccio1_pose)
    rospy.set_param('/join_complete', True)
    print('join completed')

if __name__ == '__main__':
    main()