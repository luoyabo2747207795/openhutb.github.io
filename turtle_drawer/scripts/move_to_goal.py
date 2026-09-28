#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
控制器节点 move_to_goal.py
--------------------------
功能:让 turtlesim 里的小车自动从当前位置移动到目标位置。

通信方式:
  * 订阅 /turtle1/pose        (turtlesim/Pose)      获取小车当前位姿
  * 订阅 /turtle_drawer/goal  (turtle_drawer/Goal)  接收目标位置
  * 发布 /turtle1/cmd_vel     (geometry_msgs/Twist) 输出速度指令

控制原理:
  一种经典的"比例控制器"(P 控制):
  1. 计算小车指向目标的期望角度:  angle_goal = atan2(gy-y, gx-x)
  2. 计算角度偏差:                e_theta  = angle_goal - 当前朝向
  3. 角速度 z  与角度偏差成正比,   z = Kp_ang * e_theta
  4. 线速度 x  与距离成正比,        x = Kp_lin * dist
     当需要大幅转向时,线速度自动降低,避免过冲。

运行:  rosrun turtle_drawer move_to_goal.py
"""

import math

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

# 从自定义消息包中导入 Goal 类型
from turtle_drawer.msg import Goal


class TurtleController:
    """小车控制器:根据位姿和目标计算速度指令。"""

    def __init__(self):
        # +--- 控制参数(在 init_node 之后读取,~ 命名空间才有效,支持 rosparam 覆盖)---+
        self.kp_ang   = rospy.get_param("~ang_gain", 4.0)     # 角度比例系数
        self.kp_lin   = rospy.get_param("~lin_gain", 1.5)     # 距离比例系数
        self.max_speed = rospy.get_param("~max_speed", 1.5)   # 最大线速度
        self.max_turn  = rospy.get_param("~max_turn", 2.5)    # 最大角速度
        self.stop_dist = rospy.get_param("~stop_dist", 0.05)  # 到达判定距离(米)
        self.angle_stop = rospy.get_param("~angle_stop", 0.02)# 角度收敛判定(弧度)
        self.slow_angle = rospy.get_param("~slow_angle", 0.6) # 需减速的大角度阈值
        # --------------------------------------------------------------------#

        # 当前位姿 ((x, y), 朝向角 theta),一开始未知填 None
        self.pose = None

        # 当前目标,若为空表示无新目标
        self.goal = None

        # 已到达目标,等待新目标
        self.arrived = False

        # --- 发布器:向 turtlesim 发送速度 ---
        self.cmd_pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)

        # --- 订阅器 ---
        rospy.Subscriber("/turtle1/pose", Pose, self.pose_callback)
        rospy.Subscriber("/turtle_drawer/goal", Goal, self.goal_callback)

    # ---------- 两个回调 ----------
    def pose_callback(self, msg):
        """保存最新位姿。"""
        self.pose = (msg.x, msg.y, msg.theta)

    def goal_callback(self, msg):
        """收到新目标,清空到达标志并开始新一次运动。"""
        self.goal = (msg.x, msg.y)
        self.arrived = False
        rospy.loginfo("收到新目标: (%.2f, %.2f)", msg.x, msg.y)

    # ---------- 工具函数 ----------
    @staticmethod
    def _normalize_angle(a):
        """把角度归一化到 [-pi, pi],避免 2*pi 跳变导致的错误转向。"""
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    # ---------- 核心:计算并发布速度 ----------
    def update(self):
        """每个控制周期调用一次。"""
        if self.pose is None or self.goal is None:
            return                    # 信息不完整,先不动作

        gx, gy = self.goal
        x, y, theta = self.pose

        # 1. 到目标的方向角和距离
        dist = math.hypot(gx - x, gy - y)
        angle_goal = math.atan2(gy - y, gx - x)

        # 2. 角度偏差(绕哪一侧转)
        ang_err = self._normalize_angle(angle_goal - theta)

        # 3. 到达判定
        if dist < self.stop_dist and abs(ang_err) < self.angle_stop:
            if not self.arrived:
                self.arrived = True
                rospy.loginfo("已到达目标 (%.2f, %.2f)", gx, gy)
            self._publish(0.0, 0.0)
            return

        # 4. 计算角速度(比例)
        angular = min(self.kp_ang * ang_err, self.max_turn)
        angular = max(angular, -self.max_turn)

        # 5. 计算线速度:与距离成正比;若需大幅转向则减速,更稳
        linear = self.kp_lin * dist
        if abs(ang_err) > self.slow_angle:
            linear *= 0.3
        linear = min(linear, self.max_speed)

        self._publish(linear, angular)

    def _publish(self, linear, angular):
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)


def main():
    rospy.init_node("move_to_goal")
    rospy.loginfo("控制器节点已启动,等待目标发布...")

    controller = TurtleController()

    # 以 50Hz 运行控制循环
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        controller.update()
        rate.sleep()


if __name__ == "__main__":
    main()