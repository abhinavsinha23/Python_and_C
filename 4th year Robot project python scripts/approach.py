# Purpose: To assign each braccio arm to a bone and send it to a position 10cm above the bone.

#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import time
import numpy

from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_matrix, euler_from_quaternion, quaternion_from_matrix
from math import pi, cos, sin
from moveit_msgs.msg import MoveItErrorCodes

class BraccioApproach(object):
    def __init__(self):
        global arm_number, arm_group, arm_group2, theta, braccio1_pose_pub, braccio2_pose_pub, get_objects, arm1_bone_number, arm2_bone_number
        super(BraccioApproach, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('approach', anonymous=True)
        
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

    def braccio1(self, bone_number):
        global braccio1_pose
    
        # Retrieve bone pose, height and orientation.
        bone_pose = get_objects[bone_number].pose
        bone_height = get_objects[bone_number].primitives[0].dimensions[0]
        q = bone_pose.orientation
        e = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        # Set position above bone (as a function of its length) to be dependent on the bone's yaw angle. At higher yaw angles, the gripper will be positioned closer to the bone's midpoint than at lower yaw angles.
        theta = e[2]
        if theta <0 and theta >= -pi/36:
            length_scale = 0.2
        elif theta == 0:
            length_scale = 0.3
        elif theta < -pi/36 and theta >= -2*pi/36:
            length_scale = 0.15
        elif theta > 0 and theta <= pi/36:
            length_scale = 0.3
        elif theta > pi/36 and theta <= 2*pi/36:
            length_scale = 0.15
        else:
            length_scale = 0.35
        # The length_scale can be set to zero in the line below if the position set by the length_scale value from above is unfeasible.
        # length_scale = 0

        # Calculate approach position.
        braccio1_pose = PoseStamped()
        braccio1_pose.header.frame_id = "odom"
        braccio1_pose.pose = self.arm_group.get_current_pose().pose
        braccio1_pose.pose.position.x = bone_pose.position.x + length_scale * bone_height * cos(theta)
        braccio1_pose.pose.position.y = bone_pose.position.y + length_scale * bone_height * sin(theta)
        braccio1_pose.pose.position.z = bone_pose.position.z + 0.1

        # Calculate required gripper orientation in the form of a rotation matrix.
        gripper_mat = quaternion_matrix([q.x, q.y, q.z, q.w])
        rot_mat = numpy.array([[0, 0, -1, 0], [0, 1, 0, 0], [1, 0, 0, 0],[0, 0, 0, 1]])
        rot_mat2 = numpy.array([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        rot_mat3 = numpy.array([[cos(theta), 0, sin(theta), 0], [0, 1, 0, 0], [-sin(theta), 0, cos(theta), 0], [0, 0, 0, 1]])
        rot_mat4 = numpy.array([[cos(theta), -sin(theta), 0, 0], [sin(theta), cos(theta), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        rot_mat = numpy.dot(rot_mat2, rot_mat)
        rot_mat = numpy.dot(rot_mat3, rot_mat)
        rot_mat = numpy.dot(rot_mat4, rot_mat)
        gripper_mat = numpy.dot(rot_mat, gripper_mat)

        # Convert resulting rotation matrix into a quaternion.
        q = quaternion_from_matrix(gripper_mat)
        braccio1_pose.pose.orientation.x = q[0]
        braccio1_pose.pose.orientation.y = q[1]
        braccio1_pose.pose.orientation.z = q[2]
        braccio1_pose.pose.orientation.w = q[3]

        # Publish and plan the route to the approach pose.
        braccio1_pose_pub.publish(braccio1_pose)
        braccio1_pose = self.arm_group.set_pose_target(braccio1_pose,"wrist_roll_link")   
        self.arm_group.plan(braccio1_pose) 

    def braccio2(self, bone_number):
        global braccio2_pose
        
        # Retrieve bone pose, height and orientation.
        bone_pose2 = get_objects[bone_number].pose
        bone_height2 = get_objects[bone_number].primitives[0].dimensions[0]
        q = bone_pose2.orientation
        e = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # Set position above bone.
        theta = e[2]
        # print(theta)
        if theta > 0 and theta <= pi/36:
            length_scale = 0.2
        elif theta == 0:
            length_scale = 0.3
        elif theta > pi/36 and theta <= 2*pi/36:
            length_scale = 0.15
        elif theta < 0 and theta >= -pi/36:
            length_scale = 0.3
        elif theta < -pi/36 and theta >= -2*pi/36:
            length_scale = 0.15
        else:
            length_scale = 0.35
        # The length_scale can be set to zero in the line below if the position set by the length_scale value from above is unfeasible.
        # length_scale = 0

        # Calculate approach position.
        braccio2_pose = PoseStamped()
        braccio2_pose.header.frame_id = "odom"
        # braccio2_pose.pose = self.arm_group2.get_current_pose().pose
        braccio2_pose.pose.position.x = bone_pose2.position.x - length_scale * bone_height2 * cos(theta)
        braccio2_pose.pose.position.y = bone_pose2.position.y + length_scale * bone_height2 * sin(-theta)
        braccio2_pose.pose.position.z = bone_pose2.position.z + 0.1
        
        # Calculate required gripper orientation in the form of a rotation matrix.
        gripper_mat = quaternion_matrix([q.x, q.y, q.z, q.w])
        rot_mat = numpy.array([[0, 0, -1, 0], [0, 1, 0, 0], [1, 0, 0, 0],[0, 0, 0, 1]])
        rot_mat2 = numpy.array([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        rot_mat3 = numpy.array([[cos(theta), 0, sin(theta), 0], [0, 1, 0, 0], [-sin(theta), 0, cos(theta), 0], [0, 0, 0, 1]])
        rot_mat4 = numpy.array([[cos(theta), -sin(theta), 0, 0], [sin(theta), cos(theta), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        rot_mat5 = numpy.array([[cos(pi), -sin(pi), 0, 0], [sin(pi), cos(pi), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        rot_mat = numpy.dot(rot_mat2, rot_mat)
        rot_mat = numpy.dot(rot_mat3, rot_mat)
        rot_mat = numpy.dot(rot_mat4, rot_mat)
        rot_mat = numpy.dot(rot_mat5, rot_mat)
        gripper_mat = numpy.dot(rot_mat, gripper_mat)

        # Convert resulting rotation matrix into a quaternion.
        q = quaternion_from_matrix(gripper_mat)
        braccio2_pose.pose.orientation.x = qx
        braccio2_pose.pose.orientation.y = qy
        braccio2_pose.pose.orientation.z = qz
        braccio2_pose.pose.orientation.w = qw

        # Publish and plan the route to the approach pose.
        braccio2_pose_pub.publish(braccio2_pose)
        braccio2_pose = self.arm_group2.set_pose_target(braccio2_pose,"wrist_roll_link2")
        self.arm_group2.plan(braccio2_pose)
    
    def assign_arm_to_bone(self):

        # Retrieve x-coordinates of bones and 1st Braccio arm
        bone1_x_position = self.scene.get_objects()['cylinder1'].pose.position.x
        bone2_x_position = self.scene.get_objects()['cylinder2'].pose.position.x
        arm1_x_position = self.arm_group.get_current_pose().pose.position.x

        bone_positions = [bone1_x_position, bone2_x_position]

        # Retrieve index of nearest bone.
        closest_index = self.closest(bone_positions, arm1_x_position)
        
        # Assign each arm to its nearest bone.
        if closest_index == 0:
            rospy.set_param('arm1_bone_number', 'cylinder1')
            rospy.set_param('arm2_bone_number', 'cylinder2')

        if closest_index == 1:
            rospy.set_param('arm1_bone_number', 'cylinder2')
            rospy.set_param('arm2_bone_number', 'cylinder1')

    def closest(self, bone_positions, arm1_x_position):

        # The index of the bone that is closest to the 1st Braccio arm is returned.
        closest_num = min(bone_positions, key = lambda x:abs(x-arm1_x_position))
        closest_index = bone_positions.index(closest_num)
        return closest_index

def main():
    movement = BraccioApproach()

    # Assign each arm to the nearest bone.
    movement.assign_arm_to_bone()
    arm1_bone_number = rospy.get_param('/arm1_bone_number')
    arm2_bone_number = rospy.get_param('/arm2_bone_number')

    if arm_number == 1:

        # Plan route to the approach pose for the 1st Braccio arm.
        movement.braccio1(arm1_bone_number)
            
        while True:
        # The initiate_approach parameter returns true if the user approves, and Braccio arm 1 approaches it's assigned bone.
            time.sleep(0.5)
            initiate_approach = rospy.get_param('/initiate_approach')
            print('Polling initiate approach')
            
            if initiate_approach == True:
                break
          
        arm_group.go(braccio1_pose) 
        time.sleep(5)
        rospy.set_param('/approach_complete', True)
        print('approach completed')

    if arm_number == 2:

        # Plan route to the approach pose for the 2nd Braccio arm.
        movement.braccio2(arm2_bone_number)

        while True:
        # If the user approves, Braccio arm 2 approaches it's assigned bone.
            time.sleep(0.5)
            initiate_approach = rospy.get_param('/initiate_approach')
            print('Polling initiate approach')
            
            if initiate_approach == True:
                break
          
        arm_group2.go(braccio2_pose) 
        time.sleep(5)
        rospy.set_param('/approach_complete', True)
        print('approach completed')

if __name__ == '__main__':
  main()