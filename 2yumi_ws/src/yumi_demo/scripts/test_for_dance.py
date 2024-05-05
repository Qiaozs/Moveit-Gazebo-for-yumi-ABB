#!/usr/bin/env python

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose, PoseStamped
from copy import deepcopy
# 如果想要模仿跳舞姿态，一定是需要关注各个关节的情况，所以需要用运动学正解的方式做规划：

# 用于设定关节角度
def set_joint(joint_1_l,joint_2_l,joint_3_l,joint_4_l,joint_5_l,joint_6_l,joint_7_l,joint_1_r,joint_2_r,joint_3_r,joint_4_r,joint_5_r,joint_6_r,joint_7_r):
    # 弧度制
    joint_positions=[joint_1_l/57.2,joint_2_l/57.2,joint_3_l/57.2,joint_4_l/57.2,joint_5_l/57.2,joint_6_l/57.2,joint_7_l/57.2,
                     joint_1_r/57.2,joint_2_r/57.2,joint_3_r/57.2,joint_4_r/57.2,joint_5_r/57.2,joint_6_r/57.2,joint_7_r/57.2]
    return joint_positions

class Dance:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        # 初始化ROS节点
        rospy.init_node('test_for_dance', anonymous=True)
        # 初始化需要使用move group控制的机械臂中的arm group
        dual_arm = moveit_commander.MoveGroupCommander('dual_arm')
        # 设置机械臂运动的允许误差值
        dual_arm.set_goal_joint_tolerance(0.001)
        # 设置允许的最大速度和加速度
        dual_arm.set_max_acceleration_scaling_factor(1)
        dual_arm.set_max_velocity_scaling_factor(1)

        pose_1=set_joint(-28,19,37,-22,27,-17,-94,-83,32,82,-16,-35,-34,37)
        pose_2=set_joint(-5,14,2,-32,30,-2,-85,-61,34,56,-21,-32,-43,48)
        pose_3=set_joint(13,20,-5,-37,3,-11,-56,-53,31,57,-14,-18,-47,32)
        pose_4=set_joint(-53,19,91,-28,19,-10,-105,-41,40,55,-25,-36,-27,37)
        pose_5=set_joint(-5,7,36,-19,9,-24,-104,-26,43,41,-46,-81,-18,83)
        pose_6=set_joint(-93,-94,40,63,46,-36,127,94,-94,-40,63,127,42,43)
        pose_7=set_joint(-103,-69,29,45,42,-32,124,100,-76,-31,51,124,32,50)
        pose_8=set_joint(-121,-65,24,35,52,-42,124,122,-76,-27,43,101,41,58)

        # 控制机械臂先回到初始化位置
        dual_arm.set_named_target('start')
        dual_arm.go()
        rospy.sleep(1)
        pose_20=[-2.9393556109577803 ,-0.5258314322502287 ,-1.9620476967027312 ,0.1310519276542914 ,4.136768096227176 ,0.6774487074055582 ,-0.9671598650307418 ,2.9394000226923964 ,-0.47688798428211676 ,2.164000189278255 ,-0.028556217369361647 ,0.5773893357485935 ,0.23946338256685795 ,-3.6981704794187995]

        dual_arm.set_joint_value_target(pose_20) 
        dual_arm.go()
        rospy.sleep(1)

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        Dance()
    except rospy.ROSInterruptException:
        pass
