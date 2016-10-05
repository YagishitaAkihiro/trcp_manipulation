#!/usr/bin/env python
#coding:utf-8

from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
import rospy, time

from dynamixel_controllers.srv import *


def set_slope(sl):
        print("Changing Dynamixel Compliance Slope to %d..." % sl)
        try:
            controllers = ['/base_lift_controller','/base_pan_controller','/elbow_flex_controller','/wrist_flex_controller']
            for controller in controllers:
                # 各コントローラ毎に，ゲインを指示するための service proxy (ROS の service を送るためのクライアント) を作成
                set_compliance_slope = rospy.ServiceProxy(controller + '/set_compliance_slope', SetComplianceSlope)
                temp = set_compliance_slope(sl)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

if __name__ == '__main__':

    group = MoveGroupCommander("whole_arm")
    rospy.init_node("temp_arm_demo")
      	 
    target_position = [0.05, 0.0, 0.24]  # x, y, z それぞれの位置
    target_orientation = Quaternion(*quaternion_from_euler(0.0, 1.571, 0.0, 'sxyz'))
    # 位置と傾きを用いて姿勢を生成
    target_pose = Pose(Point(*target_position), target_orientation)
    group.set_pose_target(target_pose)
    # 姿勢への移動指示を実行
    set_slope(64)
    group.go()
    
    time.sleep(1)

    while True:
      # 新たな姿勢を指定
      target_position = [0.15, 0.12, 0.05]
      target_pose = Pose(Point(*target_position), target_orientation)
      group.set_pose_target(target_pose)
      group.go()

      time.sleep(1)

      target_position = [0.05, 0.0, 0.24]
      target_pose = Pose(Point(*target_position), target_orientation)
      group.set_pose_target(target_pose)
      group.go()

