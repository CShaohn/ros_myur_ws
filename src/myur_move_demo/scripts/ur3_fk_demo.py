#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import sys,rospy
import moveit_commander

class ur3FKDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('ur3_fk_demo', anonymous=True)
        
        # 初始化需要使用move group控制的机械臂中的manipulator (见ur3_moveit_config/config/ur3.srdf)
        manipulator = moveit_commander.MoveGroupCommander('manipulator')
        # 设置机械臂和夹爪的允许误差值
        manipulator.set_goal_joint_tolerance(0.001)

         # 控制机械臂先回到初始化位置
        manipulator.set_named_target('up')
        manipulator.go()
        rospy.sleep(2)

        # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
        joint_positions = [-0.0867, -1.274, 0.02832, 0.0820, -1.273, -0.003]
        manipulator.set_joint_value_target(joint_positions)

        # 控制机械臂完成运动
        manipulator.go()
        rospy.sleep(1)
        
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        ur3FKDemo()
    except rospy.ROSInterruptException:
        pass
