#!/usr/bin/env python

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose, PoseStamped
from copy import deepcopy
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rospy.rostime import Duration 


def plan_plus(plan1, plan2):  

    # plan1和plan2是已经规划好的两个轨迹  
    # new_plan将是包含两个轨迹的新的轨迹  
    # 创建一个新的RobotTrajectory实例  
    new_plan = RobotTrajectory()  
    # 确保plan1和plan2使用相同的关节名称  
    if plan1.joint_trajectory.joint_names != plan2.joint_trajectory.joint_names:  
        raise ValueError("Joint names do not match between plans.")  
    new_plan.joint_trajectory.joint_names = plan1.joint_trajectory.joint_names  

    # 将第一个plan的点添加到new_plan中  
    for point in plan1.joint_trajectory.points:  
        new_plan.joint_trajectory.points.append(deepcopy(point))   

    # 获取第一个plan的最后时间点  
    last_time_from_start = plan1.joint_trajectory.points[-1].time_from_start  
    # 将第二个plan的点添加到new_plan中，并调整时间  
    for point in plan2.joint_trajectory.points:  
        new_point = deepcopy(point)  
        # 使用 last_time_from_start 作为偏移量  
        new_time_from_start = Duration(last_time_from_start.to_sec() + point.time_from_start.to_sec()+1)  # 0.001 是缓冲时间 
        # 确保 new_time_from_start 大于 new_plan 的最后一个时间戳（如果 new_plan 不为空）  
        new_point.time_from_start = new_time_from_start  
        new_plan.joint_trajectory.points.append(new_point)   
    return new_plan
 

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
        rospy.init_node('dance_demo', anonymous=True)
        # 初始化需要使用move group控制的机械臂中的arm group
        dual_arm = moveit_commander.MoveGroupCommander('dual_arm')
        # 设置机械臂运动的允许误差值
        dual_arm.set_goal_joint_tolerance(0.01)
        # 设置允许的最大速度和加速度
        dual_arm.set_max_acceleration_scaling_factor(1)
        dual_arm.set_max_velocity_scaling_factor(1)

        # 控制机械臂先回到初始化位置
        dual_arm.set_named_target('start')
        dual_arm.go()
        rospy.sleep(1)
     
        # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
        pose_1=set_joint(-28,19,37,-22,27,-17,-94,-83,32,82,-16,-35,-34,37)
        pose_2=set_joint(-5,14,2,-32,30,-2,-85,-61,34,56,-21,-32,-43,48)
        pose_3=set_joint(13,20,-5,-37,3,-11,-56,-53,31,57,-14,-18,-47,32)
        pose_4=set_joint(-53,19,91,-28,19,-10,-105,-41,40,55,-25,-36,-27,37)
        pose_5=set_joint(-5,7,36,-19,9,-24,-104,-26,43,41,-46,-81,-18,83)
        pose_6=set_joint(-93,-94,40,63,46,-36,127,94,-94,-40,63,127,42,43)
        pose_7=set_joint(-103,-69,29,45,42,-32,124,100,-76,-31,51,124,32,50)
        pose_8=set_joint(-121,-65,24,35,52,-42,124,122,-76,-27,43,101,41,58)
        pose_9= [-2.378073811379151 ,-1.9953166349097833 ,0.46603920732371407 ,0.9865315934556094 ,3.2208002051858915 ,-0.8032754226278662 ,0.0402993026458498 ,2.353506383163346 ,-1.964881140152487 ,-0.35457920604942395 ,0.9923653416533433 ,-2.9415747133107146 ,-0.5378347005363304 ,-0.4875705536927306]
        pose_10=[-2.3757386523797948 ,-1.8531088178747845 ,0.9756267407945822 ,1.0539064182531925 ,3.6757588310993397 ,-0.5076472589079755 ,0.15761466239388966 ,2.456162203235351 ,-1.822702203985492 ,0.18257911805844085 ,1.1227866110621694 ,-2.613125721434633 ,-0.23289067328525803 ,-0.344308899568869]
        pose_11=pose_9
        pose_12=[-2.480291563858944 ,-1.649626151963398 ,-0.3206447828117476 ,1.007644484951025 ,2.650161226897721 ,-0.3132099932636212 ,-0.19222198577717542 ,2.457699166082458 ,-1.6692900322739606 ,-1.0080630050245878 ,0.6899990646225556 ,-3.3306080972721186 ,-0.3338067956497923 ,-0.732143814753611]
        pose_13=pose_9
        pose_14=[-2.6415662367693926 ,-1.462060858242464 ,0.258045989116229 ,1.0327738642380817 ,3.109820567845553 ,-0.1717834426637479 ,-0.026691638549425356 ,2.576043933008055 ,-1.4150326562010083 ,-0.176068509474673 ,0.9121927395452145 ,-2.9005079915279994 ,-0.11215009531462172 ,-0.3693424659517843]
        pose_15=[-2.8022462622622495 ,-1.0774485749169598 ,0.21211348298598587 ,0.723871417686226 ,1.6652784717058804 ,-0.30345894580586474 ,1.3636888192972192 ,2.767047610861251 ,-1.1323667907903072 ,-0.08975407701778249 ,0.6107499877666704 ,-2.3870541904327522 ,-0.19218547809635123 ,-0.8592319129658197]
        pose_16=[-2.256776109472579 ,-0.7780145060226848 ,0.2676696411522439 ,0.6307470498591359 ,4.18003057531953 ,-0.2211577113950396 ,-1.2422289869285015 ,2.2147601355822104 ,-0.7448269254268185 ,-0.3055407392406453 ,0.3680304629413831 ,-4.468867769881939 ,-0.15082122775341045 ,1.3615566959363163]
        pose_17=[ -2.939295606069278 ,-0.7926056618233517 ,0.115200337410168 ,0.5656666522618821 ,4.468573761145153 ,0.8518891980905607 ,-1.442488364670556 ,2.894373626220954 ,-0.8521509312976621 ,-0.24516980266020916 ,0.5586611601016225 ,-4.83870466804385 ,0.9583572171329271 ,-1.689162582632881]
        pose_18=[-2.64572391310998 ,-0.8322889083102529 ,-1.316590170300696 ,0.1107496790282898 ,3.728458205674056 ,0.568084223424548 ,-1.1180239880191927 ,2.494086933720242 ,-0.917894397422141 ,1.0557330284817583 ,-0.40066626854838017 ,-1.7000293148873373 ,-0.7401181472180358 ,-0.6907941076626818]
        pose_19=[-2.452687250803482 ,-0.8513651758925285 ,-1.6035580119894632 ,-0.2304586897148866 ,3.9513782349450226 ,0.7027906776942618 ,-1.3241938305351617 ,1.9849304622795092 ,-1.0430239795318679 ,1.315087036095715 ,-1.172699209132742 ,-1.5935586930918326 ,-0.8141508015857042 ,-0.5466883588175238]
        pose_20=[-2.9393556109577803 ,-0.5258314322502287 ,-1.9620476967027312 ,0.1310519276542914 ,4.136768096227176 ,0.6774487074055582 ,-0.9671598650307418 ,2.9394000226923964 ,-0.47688798428211676 ,2.164000189278255 ,-0.028556217369361647 ,0.5773893357485935 ,0.23946338256685795 ,-3.6981704794187995]

        dual_arm.set_joint_value_target(pose_1) 
        dual_arm.go()
        dual_arm.set_joint_value_target(pose_2) 
        dual_arm.go()
        dual_arm.set_joint_value_target(pose_1) 
        plan1_tuple = dual_arm.plan()
        dual_arm.execute(plan1_tuple[1])

        dual_arm.set_joint_value_target(pose_2) 
        plan2_tuple = dual_arm.plan()
        dual_arm.execute(plan2_tuple[1])

        dual_arm.set_joint_value_target(pose_3) 
        plan3_tuple = dual_arm.plan()
        dual_arm.execute(plan3_tuple[1])

        dual_arm.set_joint_value_target(pose_4) 
        plan4_tuple = dual_arm.plan()
        dual_arm.execute(plan4_tuple[1])

        dual_arm.set_joint_value_target(pose_5) 
        plan5_tuple = dual_arm.plan()
        dual_arm.execute(plan5_tuple[1])

        # dual_arm.set_joint_value_target(pose_6) 
        # plan6_tuple = dual_arm.plan()
        # dual_arm.execute(plan6_tuple[1])

        # dual_arm.set_joint_value_target(pose_7) 
        # plan7_tuple = dual_arm.plan()
        # dual_arm.execute(plan7_tuple[1])

        # dual_arm.set_joint_value_target(pose_8) 
        # plan8_tuple = dual_arm.plan()
        # dual_arm.execute(plan8_tuple[1])

        dual_arm.set_joint_value_target(pose_9) 
        plan9_tuple = dual_arm.plan()
        dual_arm.execute(plan9_tuple[1])

        dual_arm.set_joint_value_target(pose_10) 
        plan10_tuple = dual_arm.plan()
        dual_arm.execute(plan10_tuple[1])

        dual_arm.set_joint_value_target(pose_11) 
        plan11_tuple = dual_arm.plan()
        dual_arm.execute(plan11_tuple[1])

        dual_arm.set_joint_value_target(pose_12) 
        plan12_tuple = dual_arm.plan()
        dual_arm.execute(plan12_tuple[1])

        dual_arm.set_joint_value_target(pose_13) 
        plan13_tuple = dual_arm.plan()
        dual_arm.execute(plan13_tuple[1])

        dual_arm.set_joint_value_target(pose_14) 
        plan14_tuple = dual_arm.plan()
        dual_arm.execute(plan14_tuple[1])

        dual_arm.set_joint_value_target(pose_15) 
        plan15_tuple = dual_arm.plan()
        dual_arm.execute(plan15_tuple[1])

        dual_arm.set_joint_value_target(pose_16) 
        plan16_tuple = dual_arm.plan()
        dual_arm.execute(plan16_tuple[1])

        dual_arm.set_joint_value_target(pose_17) 
        plan17_tuple = dual_arm.plan()
        dual_arm.execute(plan17_tuple[1])

        dual_arm.set_joint_value_target(pose_18) 
        plan18_tuple = dual_arm.plan()
        dual_arm.execute(plan18_tuple[1])

        dual_arm.set_joint_value_target(pose_19) 
        plan19_tuple = dual_arm.plan()
        dual_arm.execute(plan19_tuple[1])

        dual_arm.set_joint_value_target(pose_20) 
        plan20_tuple = dual_arm.plan()
        dual_arm.execute(plan20_tuple[1])

        plan1 = plan1_tuple[1]
        plan2 = plan2_tuple[1]
        plan3 = plan3_tuple[1]
        plan4 = plan4_tuple[1]
        plan5 = plan5_tuple[1]

        plan9 = plan9_tuple[1]
        plan10 = plan10_tuple[1]
        plan11 = plan11_tuple[1]
        plan12 = plan12_tuple[1]
        plan13 = plan13_tuple[1]
        plan14 = plan14_tuple[1]
        plan15 = plan15_tuple[1]
        plan16 = plan16_tuple[1]
        plan17 = plan17_tuple[1]
        plan18 = plan18_tuple[1]
        plan19 = plan19_tuple[1]
        plan20 = plan20_tuple[1]

        new_plan2 = plan_plus(plan1, plan2)
        new_plan3 = plan_plus(new_plan2, plan3)
        new_plan4 = plan_plus(new_plan3, plan4)
        new_plan5 = plan_plus(new_plan4, plan5)
        # new_plan6 = plan_plus(new_plan5, plan6)
        # new_plan7 = plan_plus(new_plan6, plan7)
        # new_plan8 = plan_plus(new_plan7, plan8)
        new_plan9 = plan_plus(new_plan5, plan9)
        new_plan10 = plan_plus(new_plan9, plan10)
        new_plan11 = plan_plus(new_plan10, plan11)
        new_plan12 = plan_plus(new_plan11, plan12)
        new_plan13 = plan_plus(new_plan12, plan13)
        new_plan14 = plan_plus(new_plan13, plan14)
        new_plan15 = plan_plus(new_plan14, plan15)
        new_plan16 = plan_plus(new_plan15, plan16)
        new_plan17 = plan_plus(new_plan16, plan17)
        new_plan18 = plan_plus(new_plan17, plan18)
        new_plan19 = plan_plus(new_plan18, plan19)
        new_plan20 = plan_plus(new_plan19, plan20)

        #控制机械臂先回到初始化位置
        dual_arm.set_named_target('start')
        dual_arm.go()
        rospy.sleep(1)

        dual_arm.set_goal_joint_tolerance(1)

        dual_arm.execute(new_plan20)
        rospy.sleep(1)

        #控制机械臂先回到初始化位置
        dual_arm.set_named_target('start')
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
