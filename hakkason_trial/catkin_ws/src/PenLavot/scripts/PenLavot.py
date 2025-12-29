#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class PenLavot:
    def __init__(self):
        rospy.init_node('move_when_far_enough')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # distance を受け取るトピック（必要に応じてトピック名/型を変更してください）
        rospy.Subscriber('/face/depth', Float32, self.distance_cb)
        self.distance = 0.0#[m]
        self.rate = rospy.Rate(10)  # 10 Hz

    def distance_cb(self, msg):
        # サブスクライブコールバックで常に最新の distance を保持
        self.distance = msg.data

    def run(self):
        rospy.loginfo("Node started: will publish v_x=0.4 when distance >= 40.0")
        try:
            while not rospy.is_shutdown():
                cmd = Twist()
                # ここが「逆」の条件: distance >= 0.5[m] -> 前進
                if self.distance >= 0.5:
                    cmd.linear.x = 0.1
                else:
                    cmd.linear.x = 0.0

                self.pub.publish(cmd)
                # ログは過度に出さないように少し抑える
                rospy.logdebug("distance=%.2f, cmd.linear.x=%.2f" % (self.distance, cmd.linear.x))
                self.rate.sleep()
        except rospy.ROSInterruptException:
            pass
        finally:
            # 終了時に必ず停止コマンドを送る
            stop = Twist()
            self.pub.publish(stop)
            rospy.loginfo("Node shutting down, sent stop command.")

if __name__ == '__main__':
    node = PenLavot()
    node.run()