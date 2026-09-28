#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
规划器节点 draw_shape.py
------------------------
功能:按顺序把"正方形/螺旋线"上的路径点一个个发布到 /turtle_drawer/goal,
     配合控制器 move_to_goal.py,小车就会沿着这些目标点画出图形。

通信方式:
  * 发布 /turtle_drawer/goal (turtle_drawer/Goal)  依次发送目标点

设计思路(体现"规划-控制"分层):
  * 规划器(本节点)只负责"要去哪"——计算出一串路径点
  * 控制器(move_to_goal节点)只负责"怎么到"——把每个点走完
  * 通过自定义消息把两者解耦,是 ROS1 典型的协作模式

运行示例:
  rosrun turtle_drawer draw_shape.py square    # 画正方形
  rosrun turtle_drawer draw_shape.py spiral    # 画螺旋线
"""

import sys
import math

import rospy
from turtle_drawer.msg import Goal

# turtlesim 画布大约 0~11 x 0~11,中心在 (5.5, 5.5)
CORNER = 1.5     # 离画布边缘的留白,保证图形在视野内


class ShapePlanner:
    """把几何图形切分成一串可供小车依次前往的目标点。"""

    def __init__(self, pub):
        self.pub = pub

    def _send(self, x, y, pause=0.5):
        """发布一个目标点并稍作停留,让小车有时间走到位。"""
        goal = Goal()
        goal.x = float(x)
        goal.y = float(y)
        self.pub.publish(goal)
        rospy.loginfo("规划:前往 (%.2f, %.2f)", goal.x, goal.y)
        rospy.sleep(pause)          # 期间控制器会持续推进

    @staticmethod
    def _pause_for_arrival():
        """给控制器留出走到当前点的时间(与 move_to_goal 的 stop 逻辑配合)。"""
        rospy.sleep(1.5)

    def draw_square(self, size):
        """依次发布正方形的四个角点(start -> 其余角 -> 回到起点)。"""
        cx = cy = 5.5
        half = size / 2.0
        corners = [
            (cx - half, cy - half),
            (cx + half, cy - half),
            (cx + half, cy + half),
            (cx - half, cy + half),
            (cx - half, cy - half),   # 封口,回到起点
        ]
        for x, y in corners:
            self._send(x, y, pause=0.6)
            self._pause_for_arrival()

    def draw_spiral(self, turns, spacing):
        """
        阿基米德螺旋线:r = spacing * k。
        以中心为起点,角度每步增加,半径不断增大。
        """
        k = 0
        points = []
        steps = int(360 * turns)          # 每一度生成一个采样点
        for i in range(steps):
            ang = math.radians(i)
            r = spacing * k
            x = 5.5 + r * math.cos(ang)
            y = 5.5 + r * math.sin(ang)
            points.append((x, y))
            k += 1
        for x, y in points:
            self._send(x, y, pause=0.12)  # 点很密,每次只短暂停留

    def run(self, shape):
        if shape == "square":
            rospy.loginfo("开始画正方形(边长 7)")
            self.draw_square(size=7.0)
            self.draw_square(size=1.0)    # 多画一个小的,效果更明显
        elif shape == "spiral":
            rospy.loginfo("开始画阿基米德螺旋线")
            self.draw_spiral(turns=2.0, spacing=0.25)
        else:
            rospy.logerr("未知图形: %s (支持 square / spiral)", shape)
        rospy.loginfo("图形规划完成.")


def main():
    rospy.init_node("draw_shape")

    pub = rospy.Publisher("/turtle_drawer/goal", Goal, queue_size=10)

    # 图形名的优先级:launch 参数 ~shape > 命令行参数 > 默认螺旋线
    shape = rospy.get_param("~shape", "spiral")
    if len(sys.argv) > 1:
        shape = sys.argv[1]

    planner = ShapePlanner(pub)
    # 稍等控制器就绪
    rospy.sleep(2.0)
    planner.run(shape)


if __name__ == "__main__":
    main()