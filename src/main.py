#!/usr/bin/env python
# -*- coding: utf-8 -*-
#----------------------------------------------------------
# Title: Storing Groceriesのマスターノード
# Author: Koya Okuse　Akihiko Nshijima
# Date: 2022/08/05 ~ 
# Memo: 競技内容はStoring Groceries
#----------------------------------------------------------
import sys
import rospy
import roslib
import actionlib
import smach
import smach_ros

from std_msgs.msg import String, Float64
#認識関係
from happymimi_recognition_msg.srv import RcognitionList
#ナビゲーション&Enter Room
from happymimi_navigation.srv import NaviLocation
from enter_room.srv import EnterRoom
#音声関係
from happymimi_voice_msgs.srv import TTS, YesNo, StrTrg
#マニピュレーション
from happymimi_manipulation_msgs.srv import RecognitionToGrasping, RcognitionToGraspingRequest

# tts_srv
tts_srv = rospy.ServiceProxy('/tts', StrTrg)

#Enter Room
class Enter(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['enter_finish'])
        # Service
        self.enter_srv = rospy.ServiceProxy('enter_room_server', EnterRoom)

    def execute(self, userdata):
        rospy.loginfo("Executing state: ENTER")
        tts_srv("Start storig groceries")
        self.enter_srv(distance=0.8, velocity=0.2)
        return 'enter_finish'

#棚への移動
class MoveShelf(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes = ['success'])

        # Publisher
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size = 1 )
        # Service
        self.navi_srv = rospy.ServicePrixy ('/navi_location_server', NaviLocation)
        
    def execute(self, userdata):
        rospy.loginfo('Executing state: MOVESHELF')
        tts.srv("Move to the shelf")
        self.navi_srv('shelf')
        return 'success'

#棚の中身の確認
class ShelfCheck(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['success','failed'],
                             input_keys = ['object_name_in'],
                             output_keys = ['object_name_out'])

        self.recognition_srv = rospy.ServiceProxy('/recognition/list', RecognitionList)
        self.bc = BaseControl()
        self.rotate = 90
        #棚の中の物体を格納
        sel.recog_result = []


    def execute(self, userdata):
        rospy.loginfo("Executing state: SHELF_CHECK")
        tts.srv("Check the contents of the shelf.")
        
        return 'success'


class ExeAction(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['action_success',
                                         'action_failure'],
                             input_keys = ['cmd_in_action',
                                           'cmd_in_data'])

    def execute(self, userdata):
        rospy.loginfo("Executing state: EXE_ACTION")
        


if __name__ == '__main__':
    rospy.init_node('main')
    sm_top = smach.StateMachine(outcomes = ['finish_sm_top'])
    sm_top.userdata.cmd_count = 1
    with sm_top:
        smach.StateMachine.add(
                'ENTER',
                Enter(),
                transitions = {'enter_finish':'MOVESHELF'})

        smach.StateMachine.add(
                'MOVESHELF',
                MoveShelf(),
                transitions = {'decide_finish':'LISTEN_COMMAND',
                               'all_cmd_finish':'finish_sm_top'},
                remapping = {'cmd_count_in':'cmd_count'})

        smach.StateMachine.add(
                'LISTEN_COMMAND',
                ListenCommand(),
                transitions = {'listen_success':'EXE_ACTION',
                               'listen_failure':'LISTEN_COMMAND',
                               'next_cmd':'DECIDE_MOVE'},
                remapping = {'cmd_out_action':'ap_action',
                             'cmd_out_data':'ap_data',
                             'cmd_count_in':'cmd_count',
                             'cmd_count_out':'cmd_count'})

        smach.StateMachine.add(
                'EXE_ACTION',
                ExeAction(),
                transitions = {'action_success':'DECIDE_MOVE',
                               'action_failure':'DECIDE_MOVE'},
                remapping = {'cmd_in_action':'ap_action',
                             'cmd_in_data':'ap_data'})

    outcome = sm_top.execute()