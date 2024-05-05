#!/usr/bin/env python

import rospy,sys
from geometry_msgs.msg import PoseStamped,Pose
from moveit_commander import MoveGroupCommander
import moveit_commander
from copy import deepcopy
import numpy,math


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

def circle_move_yz(move_group,target_pose,radius,circles,direction,start_pose):
     # 初始化路点列表
        waypoints = []
                
        # 将圆弧上的路径点加入列表
        # waypoints.append(target_pose.pose)

        centerA = target_pose.pose.position.y # 圆心
        centerB = target_pose.pose.position.z
                                           

        for th in numpy.arange(0, circles*6.28, 0.02):
            if start_pose == 1:
             a = 3.14
            if start_pose == 2:
             a = 4.71
            if start_pose == 3:
             a = 0
            if start_pose == 4:
             a = 1.57 
            if direction == 0:# 逆时针
                target_pose.pose.position.y = centerA + radius * math.cos(th+a)
                target_pose.pose.position.z = centerB + radius * 0.5*math.sin(th+a)
                wpose = deepcopy(target_pose.pose)
                waypoints.append(deepcopy(wpose))
            if direction ==1:# 顺时针
                target_pose.pose.position.y = centerA + radius * math.cos(th+a)
                target_pose.pose.position.z = centerB - radius * 0.5*math.sin(th+a)
                wpose = deepcopy(target_pose.pose)
                waypoints.append(deepcopy(wpose))
       
        fraction = 0.0   #路径规划覆盖率
        maxtries= 100   #最大尝试规划次数
        attempts = 0     #已经尝试规划次数

        # 设置机器臂当前的状态作为运动初始状态
        move_group.set_start_state_to_current_state()
 
        # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点，完成圆弧轨迹
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = move_group.compute_cartesian_path (
                                    waypoints,   # waypoint poses，路点列表
                                    0.01,        # eef_step，终端步进值
                                    0.0,         # jump_threshold，跳跃阈值
                                    True)        # avoid_collisions，避障规划
            
            # 尝试次数累加
            attempts += 1
            
            # 打印运动规划进程
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
        
        # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
        if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm.")
            move_group.execute(plan)
            rospy.loginfo("Path execution complete.")
        # 如果路径规划失败，则打印失败信息
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  

class EllipseDemo:
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


        target_pose_l_1=set_pose(0.5,0.3,0.5,  0, 0.7071068, 0, 0.7071068 )
        target_pose_r_1=set_pose(0.5,-0.3,0.5, 0.7071068, 0, 0.7071068, 0 )

 
        # 设置机械臂终端运动的目标位姿
        arm_l.set_pose_target(target_pose_l_1, end_effector_link_l)
        arm_l.go()
        arm_r.set_pose_target(target_pose_r_1, end_effector_link_r)
        arm_r.go()

        # circle_move 函数，参数为：move_group,target_pose,半径，圈数，方向（顺时针1，逆时针0），起始位置（圆心左1，圆心上2，圆心右3，圆心下4）
        circle_move_yz(arm_l,target_pose_l_1,0.15,1,0,1)
        circle_move_yz(arm_r,target_pose_r_1,0.15,1,1,3)    
        rospy.sleep(1)

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
        EllipseDemo()
    except rospy.ROSInternalException:
        pass
