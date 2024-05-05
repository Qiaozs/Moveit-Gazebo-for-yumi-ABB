#!/usr/bin/env python

import rospy,sys
from geometry_msgs.msg import PoseStamped,Pose
from moveit_commander import MoveGroupCommander
import moveit_commander
from copy import deepcopy
import numpy,math


def show_pose(move_group):
     current_pose = move_group.get_current_pose()   
     rospy.loginfo("x  = " + str(current_pose.pose.position.x))
     rospy.loginfo("y  = " + str(current_pose.pose.position.y))
     rospy.loginfo("z  = " + str(current_pose.pose.position.z))
     rospy.loginfo("ox = " + str(current_pose.pose.orientation.x))
     rospy.loginfo("oy = " + str(current_pose.pose.orientation.y))
     rospy.loginfo("oz = " + str(current_pose.pose.orientation.z))
     rospy.loginfo("w  = " + str(current_pose.pose.orientation.w))
     
def set_pose(x,y,z,ox,oy,oz,w):
        reference_frame = 'world'
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        target_pose.pose.orientation.x = ox
        target_pose.pose.orientation.y = oy
        target_pose.pose.orientation.z = oz
        target_pose.pose.orientation.w = w
        return target_pose
def move(move_group,target_pose, end_effector_link):
        move_group.set_pose_target(target_pose, end_effector_link)
        move_group.go()
        show_pose(move_group)
     
class CircleDemo:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('double_circle_demo',anonymous=True)
        arm_l=moveit_commander.MoveGroupCommander('arm_l')
        arm_r=moveit_commander.MoveGroupCommander('arm_r')
        
        # 设置机械臂运动的允许误差值
        arm_l.set_goal_joint_tolerance(0.001)
        arm_r.set_goal_joint_tolerance(0.001)   

        # 设置允许的最大速度和加速度
        arm_l.set_max_acceleration_scaling_factor(1)
        arm_l.set_max_velocity_scaling_factor(1)
        arm_r.set_max_acceleration_scaling_factor(1)
        arm_r.set_max_velocity_scaling_factor(1)

        # 获取终端link的名称
        end_effector_link_l = arm_l.get_end_effector_link()
        end_effector_link_r = arm_r.get_end_effector_link()

        # 设置目标位置所用的参考系
        reference_frame = 'world'
        arm_l.set_pose_reference_frame(reference_frame)
        arm_r.set_pose_reference_frame(reference_frame)

        # 当运动规划失败后，允许重新规划
        arm_l.allow_replanning(True)
        arm_r.allow_replanning(True)

        # 控制机械臂先回到初始化位置
        arm_l.set_named_target('home_l')
        arm_l.go()
        arm_r.set_named_target('home_r')
        arm_r.go()
        rospy.sleep(1)
        
        # 标定要到达的位置点
        target_pose_l_1=set_pose(0.5,0.4,0.5,  0, 0.7071068, 0, 0.7071068 )
        target_pose_r_1=set_pose(0.5,-0.2,0.5, 0.7071068, 0, 0.7071068, 0 )
        target_pose_l_2=set_pose(0.5,0.2,0.5,  0, 0.7071068, 0, 0.7071068 )
        target_pose_r_2=set_pose(0.5,-0.4,0.5, 0.7071068, 0, 0.7071068, 0 )
        target_pose_l_3=set_pose(0.3,0.5,0.5,  0, 0, 0, 1 )
        target_pose_r_3=set_pose(0.3,-0.5,0.5,  0, 0, 0,1 )
 
        # 设置机械臂终端运动的目标位姿并规划执行运动
        rospy.loginfo("左臂第 1 个位置：")
        move(arm_l,target_pose_l_1, end_effector_link_l)
        rospy.loginfo("右臂第 1 个位置：")
        move(arm_r,target_pose_r_1, end_effector_link_r)
        rospy.loginfo("左臂第 2 个位置：")
        move(arm_l,target_pose_l_2, end_effector_link_l)
        rospy.loginfo("右臂第 2 个位置：")
        move(arm_r,target_pose_r_2, end_effector_link_r)
        rospy.loginfo("左臂第 3 个位置：")
        move(arm_l,target_pose_l_3, end_effector_link_l)
        rospy.loginfo("右臂第 3 个位置：")
        move(arm_r,target_pose_r_3, end_effector_link_r)


        # 控制机械臂先回到初始化位置
        arm_l.set_named_target('home_l')
        arm_l.go()
        arm_r.set_named_target('home_r')
        arm_r.go()
        rospy.sleep(1)
        
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


if __name__ =="__main__":
    try:
        CircleDemo()
    except rospy.ROSInternalException:
        pass
