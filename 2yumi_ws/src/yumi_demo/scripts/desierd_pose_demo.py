#!/usr/bin/env python

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose, PoseStamped
from copy import deepcopy


def  set_pose(x,y,z,ox,oy,oz,ow):
        reference_frame = 'world'
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.pose.position.x=x
        target_pose.pose.position.y=y
        target_pose.pose.position.z=z
        target_pose.pose.orientation.x=ox
        target_pose.pose.orientation.y=oy
        target_pose.pose.orientation.z=oz
        target_pose.pose.orientation.w=ow
        return target_pose
def show_pose(move_group):
     current_pose = move_group.get_current_pose()   
     rospy.loginfo("x  = " + str(current_pose.pose.position.x))
     rospy.loginfo("y  = " + str(current_pose.pose.position.y))
     rospy.loginfo("z  = " + str(current_pose.pose.position.z))
     rospy.loginfo("ox = " + str(current_pose.pose.orientation.x))
     rospy.loginfo("oy = " + str(current_pose.pose.orientation.y))
     rospy.loginfo("oz = " + str(current_pose.pose.orientation.z))
     rospy.loginfo("w  = " + str(current_pose.pose.orientation.w))

class TESTDEMO:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        # 初始化ROS节点
        rospy.init_node('desired_pose_demo', anonymous=True)
        # 初始化需要使用move group控制的机械臂中的arm group
        dual_arm = moveit_commander.MoveGroupCommander('dual_arm')
        arm_l =  moveit_commander.MoveGroupCommander('arm_l')
        arm_r = moveit_commander.MoveGroupCommander('arm_r')

        # 设置一些初始参数
        # 设置机械臂运动的允许误差值
        dual_arm.set_goal_joint_tolerance(0.001)  
       # 设置允许的最大速度和加速度
        dual_arm.set_max_acceleration_scaling_factor(1)
        dual_arm.set_max_velocity_scaling_factor(1)
        # 控制机械臂先回到初始化位置
        dual_arm.set_named_target('start')
        dual_arm.go()

        # 读取两臂的末端关节
        # 获取终端link的名称
        end_effector_link_l = arm_l.get_end_effector_link()
        end_effector_link_r = arm_r.get_end_effector_link()

        # 设置机械臂工作空间的目标位姿，位置使用x,y,z，姿态使用四元数描述，基于world坐标系
        target_pose_l_1=set_pose(0.5,0.15,0.5,  0, -0.7071068, 0.7071068 ,0 )
        target_pose_r_1=set_pose(0.5,-0.15,0.5, 0, 0.7071068, 0.7071068 ,0)
        target_pose_l_2=set_pose(0.5,0.15,0.6,  0, -0.7071068, 0.7071068 ,0 )
        target_pose_r_2=set_pose(0.5,-0.15,0.6, 0, 0.7071068, 0.7071068 ,0)
        target_pose_l_3=set_pose(0.3,0.5,0.5,  0, 0, 0, 1 )
        target_pose_r_3=set_pose(0.3,-0.5,0.5,  0, 0, 0,1 )
        # 设置机械臂当前的状态作为运动初始状态
        dual_arm.set_start_state_to_current_state()
        dual_arm.set_pose_target(target_pose_l_1, end_effector_link_l)
        dual_arm.set_pose_target(target_pose_r_1, end_effector_link_r)
        dual_arm.go()
        rospy.loginfo("左臂第 1 个位置：")
        show_pose(arm_l)
        rospy.loginfo("右臂第 1 个位置：")
        show_pose(arm_r)
        
        dual_arm.set_pose_target(target_pose_l_2, end_effector_link_l)
        dual_arm.set_pose_target(target_pose_r_2, end_effector_link_r)
        dual_arm.go()
        rospy.loginfo("左臂第 2 个位置：")
        show_pose(arm_l)
        rospy.loginfo("右臂第 2 个位置：")
        show_pose(arm_r)

        dual_arm.set_pose_target(target_pose_l_3, end_effector_link_l)
        dual_arm.set_pose_target(target_pose_r_3, end_effector_link_r)
        dual_arm.go()
        rospy.loginfo("左臂第 3 个位置：")
        show_pose(arm_l)
        rospy.loginfo("右臂第 3 个位置：")
        show_pose(arm_r)
        rospy.sleep(1)


if __name__ == "__main__":
    try:
        TESTDEMO()
    except rospy.ROSInterruptException:
        pass