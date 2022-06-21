#!/usr/bin/env python

import smach
import rospy

from std_msgs.msg import String


class WaitForMsg(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['success'], input_keys=['in_data'])
        
        self.msgContent = ''
        self.receivedMsg = False
        
        #Subscriber
        self.msg_sub = rospy.Subscriber('/start_landing', String, self.resultCallback, queue_size=1)
        rospy.sleep(0.5)

    def resultCallback(self, msg):
        rospy.loginfo('Received msg')
        self.msgContent = msg.data
        self.receivedMsg = True

    def execute(self, userdata):
        while not rospy.is_shutdown():
            if self.receivedMsg:
                self.msg_sub.unregister()
                return 'success'