#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose, PoseStamped
from copy import deepcopy

import math
import numpy

class get_joint:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('get_joint', anonymous=True)
 
        # 初始化需要使用move group控制的机械臂中的arm group
        dual_arm = moveit_commander.MoveGroupCommander('dual_arm')

        # 设置机械臂运动的允许误差值
        dual_arm.set_goal_joint_tolerance(0.001)
  
        # 设置允许的最大速度和加速度
        dual_arm.set_max_acceleration_scaling_factor(1)
        dual_arm.set_max_velocity_scaling_factor(1)

        # # 控制机械臂先回到初始化位置
        # arm_l.set_named_target('start_l')
        # arm_l.go()
        # rospy.sleep(1)
     
        # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
        pose_1 = dual_arm.get_current_joint_values()
        rospy.loginfo(str(pose_1[0])+" ,"+
                      str(pose_1[1])+" ,"+
                      str(pose_1[2])+" ,"+
                      str(pose_1[3])+" ,"+
                      str(pose_1[4])+" ,"+
                      str(pose_1[5])+" ,"+
                      str(pose_1[6])+" ,"+
                      str(pose_1[7])+" ,"+
                      str(pose_1[8])+" ,"+
                      str(pose_1[9])+" ,"+
                      str(pose_1[10])+" ,"+
                      str(pose_1[11])+" ,"+
                      str(pose_1[12])+" ,"+
                      str(pose_1[13]))
        
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        get_joint()
    except rospy.ROSInterruptException:
        pass