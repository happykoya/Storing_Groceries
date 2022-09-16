#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#----------------------------------------------------------
# Title: Storing Groceriesのマスターノード
# Author: Koya Okuse　Akihiko Nshijima　Takumi Nozaki
# Date: 2022/08/05 ~
# Memo: 競技内容はStoring Groceries
#----------------------------------------------------------
import sys
import rospy
import roslib
import actionlib
import smach
import smach_ros

# メッセージの方で使用
from std_msgs.msg import String, Float64
# 認識関係
from happymimi_recognition_msg.srv import RecognitionList
# ナビゲーション&Enter Room
from happymimi_navigation.srv import NaviLocation
from enter_room.srv import EnterRoom
# 音声関係
from happymimi_voice_msgs.srv import TTS, YesNo, StrTrg
# マニピュレーション
from happymimi_manipulation_msgs.srv import RecognitionToGrasping, RecognitionToGraspingRequest
# いるかわからない...
from base_control import BaseControl


# tts_srv
tts_srv = rospy.ServiceProxy('/tts', StrTrg)


# 部屋の中へ移動
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


# 棚への移動
class MoveShelf(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['moved_shelf'])

        # Service
        self.navi_srv = rospy.ServiceProxy('/navi_location_server', NaviLocation)
        
    def execute(self, userdata):
        rospy.loginfo('Executing state: MOVESHELF')
        tts_srv("Move to the shelf")
        self.navi_srv('shelf')
        return 'moved_shelf'


# 棚の中身の確認
class ShelfCheck(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['check_success', 'check_failure'],
                             input_keys = ['object_name_in'],
                             output_keys = ['object_name_out'])
        
        # Publisher
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size = 1)

        # Service
        # 物体の一覧を取得する（list型）
        self.recognition_srv = rospy.ServiceProxy('/recognition/list', RecognitionList)
        self.grasp_srv = rospy.ServiceProxy('/recognition_to_grasping', RecognitionToGrasping)
        self.arm_srv = rospy.ServiceProxy('/servo/arm', StrTrg)

        # Variable
        self.bc = BaseControl()
        self.rotate = 90
        self.all_object = ['cup', 'bottle']

        # 棚の中の物体を格納(2重リストにする？)
        '''
        理想: recog_result = {"1段":["cup", "cup"],
                              "2段":["bottle", "snack"],
                              "3段":["big bottle", "any"]}
        '''
        self.shelf_result = {1:[], 2:[], 3:[]}

    def execute(self, userdata):
        rospy.loginfo("Executing state: SHELF_CHECK")
        tts_srv("Check the contents of the shelf.")
        # 頭部を可動（角度の数値は要調整!）
        '''
        self.head_pub.publish(25.0)
        rospy.sleep(2.0)
        self.shelf_result = self.recog_srv('cup','left')
        '''
        # 上を動作を頭部(カメラ)の角度を調整しながら繰り返す...
        # ↓最適化したいよね...

        # 1段目（下からカウント）
        head_angles = [15.0, -15.0, -10.0]
        for i in range(3):
            self.head_pub.publish(head_angles[i])
            rospy.sleep(2.0)
            self.shelf_result[i + 1] = self.recognition_srv('any', 'left').object_list

        print(self.shelf_result)
        return 'success'


class PickandPlace(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['place_success',
                                         'place_failure'],
                             input_keys = ['cmd_in_action',
                                           'cmd_in_data'])

        # Service
        self.recognition_srv = rospy.ServiceProxy('/recognition/list', RecognitionList)
        self.grasp_srv = rospy.ServiceProxy('/recognition_to_grasping', RecognitionToGrasping)
        self.arm_srv = rospy.ServiceProxy('/servo/arm', StrTrg)

        # Variable
        # テーブル上の物体を格納
        self.table_result = []

    def execute(self, userdata):
        rospy.loginfo("Executing state: PickandPlace")
        tts_srv("Moving TallTable")

        # テーブルへの移動
        self.navi_srv('Tall table')

        # テーブル上の物体の一覧を取得(左からソート)
        self.table_result = self.recognition_srv('any', 'left').object_list
        # 物体の把持（左側の物体から）
        if self.grasp_srv(self.table_result[0]):
            return 'action_success'
        return 'action_failure'


if __name__ == '__main__':
    rospy.init_node('storing_groceries')
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
                transitions = {'moved_shelf':'SHELF_CHECK'},
                remapping = {'cmd_count_in':'cmd_count'})

        smach.StateMachine.add(
                'SHELF_CHECK',
                ShelfCheck(),
                transitions = {'check_success':'PICKANDPLACE',
                            　 'non_something':'EXITROOM'
                               'check_failure':'SHELF_CHECK'},
                remapping = {'cmd_out_action':'ap_action',
                             'cmd_out_data':'ap_data',
                             'cmd_count_in':'cmd_count',
                             'cmd_count_out':'cmd_count'})

        smach.StateMachine.add(
                'PICKANDPLACE',
                PickandPlace(),
                transitions = {'place_success':'SHELF_CHECK',
                               'place_failure':'SHELF_CHECK'},
                remapping = {'cmd_in_action':'ap_action',
                             'cmd_in_data':'ap_data'})

        smach.StateMachine.add('Exit',Exit(),transitions = {'to_finish':finish_sm})

    outcome = sm_top.execute()
