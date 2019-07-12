#!/usr/bin/env python
import rospy
import sys
import tf
import tf2_ros

import numpy as np
from gpd.msg import GraspConfig
from gpd.msg import GraspConfigList
from std_msgs.msg import Int32
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped 
import geometry_msgs.msg

FFC_recived = []
grasps = GraspConfig()
import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list

x_offset = 0
y_offset = -0.05
gripper_length = 0.275
vertical_threshold = -1
forward_dis = 0.15

# Get the msg from /cloud_indexed topic.
def callback(msg):
    global FFC_recived
    grasps = msg
    FFC_recived.append(grasps)

# Add a new frame for target object.
def add_target_frame(transfer_point):
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()
  
    static_transformStamped.header.stamp = rospy.Time(0)
    static_transformStamped.header.frame_id = 'r1_base_link' #parent frame
    static_transformStamped.child_frame_id = 'target_frame' #child frame
  
    # Give the position and orientation of frame, based on parent frame.
    static_transformStamped.transform.translation.x = transfer_point.pose.position.x
    static_transformStamped.transform.translation.y = transfer_point.pose.position.y
    static_transformStamped.transform.translation.z = transfer_point.pose.position.z
  
    static_transformStamped.transform.rotation.x = transfer_point.pose.orientation.x
    static_transformStamped.transform.rotation.y = transfer_point.pose.orientation.y
    static_transformStamped.transform.rotation.z = transfer_point.pose.orientation.z
    static_transformStamped.transform.rotation.w = transfer_point.pose.orientation.w
 
    broadcaster.sendTransform(static_transformStamped)

# Create and show a arrow maker to show graps pose in RVIZ.
def pub_grasp_marker(transfer_reach_point):        
    #Setup the marker.
    markerArray = MarkerArray()
    marker = Marker()
    marker.header.frame_id = '/r1_base_link'
    marker.type = marker.ARROW
    marker.action = marker.ADD
    marker.scale.x = 0.100
    marker.scale.y = 0.015
    marker.scale.z = 0.015
    marker.color.a = 1.0
    marker.color.g = 1.0  

    #Give the informations to marker.msg.
    marker.pose = transfer_reach_point.pose
    markerArray.markers.append(marker)
    pub_marker.publish(markerArray)


def reach_robot():
    # Instantiate a tf_Transform object.
    tf_listener = tf.TransformListener()
    tf_listener.waitForTransform('target_frame', 'r1_base_link', rospy.Time(0),rospy.Duration(4.0))

    # Transfer the point from camera to robot coordinate system.
    reach_point = PoseStamped()
    reach_point.header.frame_id = 'target_frame'
    reach_point.header.stamp = rospy.Time(0)
    reach_point.pose.position.x = -(gripper_length)
    
    # Transform the position and orientation from target frame to robot base frame.
    transfer_reach_point = tf_listener.transformPose('r1_base_link', reach_point)
    
    # Publish reach position marker to RVIZ. 
    pub_grasp_marker(transfer_reach_point)

    # Plan the motion trajectory.
    group.set_start_state_to_current_state()
    group.set_pose_target(transfer_reach_point)
    plan_success = group.plan()

    rospy.sleep(1)

    # Execute the motion trajectory
    if(plan_success.joint_trajectory.points):
        # Hit a key to excute the planning
        print 'Press make sure it is safe and input key to continue... (Y/N)'
        in_content = raw_input('Input:')
        if in_content == 'Y' or in_content == "y":
            rospy.sleep(1)
            group.execute(plan_success, wait=True)
                
            # Make sure the motion is stop
            group.stop()
            group.clear_pose_targets()
                
            rospy.sleep(1)
        elif in_content == 'N' or in_content == 'n':
            exit(0)
        else:
            print 'Wrong key.'
            exit(0)
    else:
        exit(0)
            

def robot_forward_move(forward_dis):
    # Instantiate a tf_Transform object.
    tf_listener = tf.TransformListener()
    tf_listener.waitForTransform('r1_link_t', 'r1_base_link', rospy.Time(0),rospy.Duration(4.0))

    # Transfer the point from camera to robot coordinate system.
    forward_point = PoseStamped()
    forward_point.header.frame_id = 'r1_link_t'
    forward_point.header.stamp = rospy.Time(0)
    forward_point.pose.position.x = forward_dis
    transfer_forward_point = tf_listener.transformPose('r1_base_link', forward_point)

    # Plan the motion trajectory.
    group.set_start_state_to_current_state()
    group.set_pose_target(transfer_forward_point)
    plan_success = group.plan()

    rospy.sleep(1)

    # Hit a key to excute the planning.
    raw_input('Press make sure it is safe and press Enter to continue...\n')

    # Execute the motion trajectory.
    if(plan_success):
        rospy.sleep(1)
        group.execute(plan_success, wait=True)

    # Make sure the motion is stop.
    group.stop()
    group.clear_pose_targets()

    rospy.sleep(1)

def move_to_target(top_grasp_tra, top_grasp_ori):
    # Getting basic information.
    planning_frame = group.get_planning_frame()
    print 'Reference frame: %s' % planning_frame
    eef_link = group.get_end_effector_link()
    print 'End effector: %s' % eef_link

    # Transfer the point from camera to robot coordinate system.
    original_point = PoseStamped()
    original_point.header.frame_id = 'kinect2_link'
    original_point.header.stamp = rospy.Time(0)
    original_point.pose.position.x = top_grasp_tra[0] + x_offset
    original_point.pose.position.y = top_grasp_tra[1] + y_offset
    original_point.pose.position.z = top_grasp_tra[2]
    original_point.pose.orientation.w = top_grasp_ori[3]
    original_point.pose.orientation.x = top_grasp_ori[0]
    original_point.pose.orientation.y = top_grasp_ori[1]
    original_point.pose.orientation.z = top_grasp_ori[2]
    transfer_point = tf_listener.transformPose('r1_base_link', original_point)

    # Add a target frame to localize.
    add_target_frame(transfer_point)
    
    # Open the gripper.
    pub_grippper.publish(0)
    
    # Move the robot to reach pose.
    reach_robot()

    # Move the robot forward to grasp the object.
    robot_forward_move(forward_dis)
    
    # Close the gripper
    pub_grippper.publish(1)
    
    rospy.loginfo('... Done ...')

    rospy.spin()


def main():
    # Subscribe to the ROS topic that contains the grasps.
    sub = rospy.Subscriber('/FFC_GraspConfig', GraspConfig, callback)
    
    # Wait for grasps to arrive.
    rospy.sleep(1)
    
    print 'Threshold value: ', vertical_threshold

    #Get the highest score grasp pose
    while not rospy.is_shutdown():    
        if len(FFC_recived)> 0:
            top_grasp = FFC_recived[0] # grasps are sorted in descending order by score.
            rospy.loginfo('Received GraspConfig.')
       
            # Check the robot and table is collided or not.
            a = np.array([top_grasp.approach.x, top_grasp.approach.y, top_grasp.approach.z])
            b = np.array([0 , 0.707106781187 ,0.707106781187]) #vertical appraoch vector value 
            dot_product = np.dot(a,b)

            # Give a threshold of collided
            if dot_product >= vertical_threshold :
                #Get translation and orientation data.
                print 'Value of dot product: ',dot_product
                #print 'Choose the data of highest grasp score:'
                top_grasp_tra = [top_grasp.bottom.x, top_grasp.bottom.y, top_grasp.bottom.z]
                #print 'The translation representation is:'
                #print 'x:%s \ny:%s \nz:%s' % (top_grasp_tra[0],top_grasp_tra[1], top_grasp_tra[2])

                R = np.matrix([[top_grasp.approach.x, top_grasp.binormal.x, top_grasp.axis.x], 
                               [top_grasp.approach.y, top_grasp.binormal.y, top_grasp.axis.y],
                               [top_grasp.approach.z, top_grasp.binormal.z, top_grasp.axis.z]])
                #print 'Rotation Matrix:\n',(R)
                
                # Transform the RPY to quaterion.
                euler_R = tf.transformations.euler_from_matrix(R)
                top_grasp_ori = tf.transformations.quaternion_from_euler(euler_R[0], euler_R[1], euler_R[2])

                #print 'The quaternion representation is:'
                #print 'x:%s \ny:%s \nz:%s \nw:%s' % (top_grasp_ori[0], top_grasp_ori[1], top_grasp_ori[2], top_grasp_ori[3])

                #Publish the marker and move robot by translation and orientation data.
                move_to_target(top_grasp_tra, top_grasp_ori)    
        
            else:
                rospy.logwarn('Bad grasp pose generated.')
                # rospy.sleep(1)
                main()



# Create a ROS node.
rospy.init_node('FFC_get_grasps')
rospy.loginfo('Start to get grasp data...')

# Publish to the ROS topic that grasp data.
pub_marker = rospy.Publisher('/grasp_marker', MarkerArray, queue_size=10)
# Publish to the ROS topic that gripper action.
pub_grippper = rospy.Publisher('/control_grasp', Int32, queue_size=10)

# Instantiate a tf_Transform object.
tf_listener = tf.TransformListener()
tf_listener.waitForTransform('kinect2_link', 'r1_base_link', rospy.Time(0),rospy.Duration(4.0))

# Initialize moveit_commander
moveit_commander.roscpp_initialize(sys.argv)

# Instantiate a RobotCommander object
robot = moveit_commander.RobotCommander()
# Give group_name and instantiate a MoveGroupCommander object
group_name = 'r1'
group = moveit_commander.MoveGroupCommander(group_name)
# Set max velocity and max acceleration of robot moving
group.set_goal_orientation_tolerance(0.025)
group.set_goal_position_tolerance(0.01)
group.set_max_velocity_scaling_factor(0.08)

main()
