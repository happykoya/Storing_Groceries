#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#----------------------------------------------------------
# Title: Storing Groceriesの認識関係
# Author: Takumi Nozaki
# Date: 2022/09/06~
#----------------------------------------------------------
import time
import rospy
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes
from happymimi_recognition_msg.srv import RecognitionList, RecognitionListResponse


class Recognition:
    bbox = []

    def __init__(self):
        # Subscriber
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.boundingBoxCB)
        rospy.Subscriber('/camera/color/image_raw', Image, self.realsenseCB)
        rospy.Service('/recognition/list', RecognitionList, self.listObject)

        self.realsence_image = Image()
        self.default_object_list = ['cup', 'bottle', 'snack', 'big bottle']
        # darknetからpublishされた時刻を記録
        self.update_time = 0
        # darknetからpublishされたかどうかの確認
        self.update_flg = False

    def boundingBoxCB(self, bb):
        self.update_time = time.time()
        self.update_flg = True
        Recognition.bbox = bb.bounding_boxes

    def realsenseCB(self, image):
        self.realsense_image = image

    def listObject(self, request, bb=None):
        object_name = request.target_name
        sort_option = request.sort_option
        response = RecognitionListResponse()
        coordinate_list = []
        if bb is None:
            bb = Recognition.bbox
        bbox_list = [i.Class for i in bb]
        if object_name == 'any':
            object_list = self.default_object_list
        else:
            object_list = [object_name]

        # 座標を格納したlistを作成
        for i in range(len(bbox_list)):
            if object_name == '' or object_name in object_list:
                center = [(bb[i].ymin + bb[i].ymax) / 2, (bb[i].xmin + bb[i].xmax) / 2]
                coordinate_list.append([bbox_list[i], center])

        # ソート
        if sort_option == 'left':
            coordinate_list.sort(key=lambda x: x[1][1])
        if sort_option == 'right':
            coordinate_list.sort(key=lambda x: x[1][1], reverse=True)

        response.object_list = [i[0] for i in coordinate_list]
        return response


if __name__ == '__main__':
    rospy.init_node('recognition')
    recognition = Recognition()
    rospy.spin()
