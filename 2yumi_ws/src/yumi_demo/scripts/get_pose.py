#!/usr/bin/env python

import rospy,sys
from geometry_msgs.msg import PoseStamped,Pose
from moveit_commander import MoveGroupCommander
import moveit_commander
from copy import deepcopy
import numpy,math


def show_pose(move_group,arm_l,arm_r):
     current_pose = move_group.get_current_pose()   
     rospy.loginfo(str(current_pose.pose.position.x)+","+
                   str(current_pose.pose.position.y)+","+
                   str(current_pose.pose.position.z)+","+
                   str(current_pose.pose.orientation.x)+","+
                   str(current_pose.pose.orientation.y)+","+
                   str(current_pose.pose.orientation.z)+","+
                   str(current_pose.pose.orientation.w))


class get_pose:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('get_pose',anonymous=True)
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
        # 设置目标位置所用的参考系
        reference_frame = 'world'
        arm_l.set_pose_reference_frame(reference_frame)
        arm_r.set_pose_reference_frame(reference_frame)

        # 当运动规划失败后，允许重新规划
        arm_l.allow_replanning(True)
        arm_r.allow_replanning(True)

        show_pose(arm_l,arm_l,arm_r)
        show_pose(arm_r,arm_l,arm_r)
       
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


if __name__ =="__main__":
    try:
        get_pose()
    except rospy.ROSInternalException:
        pass
