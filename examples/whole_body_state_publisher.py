# -*- coding: utf-8 -*-
"""
Created on Tue Oct 23 11:37:03 2018

@author: rorsolino
"""

#!/usr/bin/env python
# license removed for brevity
import rospy
from context import jet_leg 
from jet_leg.wholeBodyStateInterface import WholeBodyStateInterface
from dwl_msgs.msg import WholeBodyState, BaseState, JointState, ContactState
from std_msgs.msg import String

def talker():
    
    msg = WholeBodyState();
    wbState = WholeBodyState()
    wbStateInterface = WholeBodyStateInterface()
    pub = rospy.Publisher('chatter', WholeBodyState, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        
        msg = wbStateInterface.writeToMessage(wbState)
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass