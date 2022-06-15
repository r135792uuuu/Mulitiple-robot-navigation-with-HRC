# -*- coding:utf-8 -*-
#!/usr/bin/env python3
"""
@ By: zhaocong 
@ Date: 2021-11-12
"""
import cv2
from cv_bridge import CvBridge, CvBridgeError#此处引用ros与opencv的桥梁
from sensor_msgs.msg import Image#传图片
import mediapipe as mp
import rospy#与ros建立接口
from geometry_msgs.msg import Twist
class HandDetector:
    """
    使用mediapipe库查找手。导出地标像素格式。添加了额外的功能。
    如查找方式，许多手指向上或两个手指之间的距离。而且提供找到的手的边界框信息。
    """
    
    def __init__(self, mode=False, maxHands=2,modelComplexity=1, detectionCon=0.5, minTrackCon=0.5):
        """
        :param mode: 在静态模式下，对每个图像进行检测
        :param maxHands: 要检测的最大手数
        :param detectionCon: 最小检测置信度
        :param minTrackCon: 最小跟踪置信度
        """
        self.mode = mode
        self.maxHands = maxHands
        self.detectionCon = detectionCon
        self.minTrackCon = minTrackCon
        self.modelComplex = modelComplexity

        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(self.mode, self.maxHands,
                                        self.modelComplex,self.detectionCon, self.minTrackCon)
        self.mpDraw = mp.solutions.drawing_utils
        self.tipIds = [4, 8, 12, 16, 20]
        self.fingers = []
        self.lmList = []

    def findHands(self, img, draw=True):
        """
        从图像(BRG)中找到手部。
        :param img: 用于查找手的图像。
        :param draw: 在图像上绘制输出的标志。
        :return: 带或不带图形的图像
        """
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) # 将传入的图像由BGR模式转标准的Opencv模式——RGB模式，
        self.results = self.hands.process(imgRGB)

        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(img, handLms,
                                               self.mpHands.HAND_CONNECTIONS)
        return img

    def findPosition(self, img, handNo=0, draw=True):
        """
        查找单手的地标并将其放入列表中像素格式。还可以返回手部周围的边界框。
        :param img: 要查找的主图像
        :param handNo: 如果检测到多只手，则为手部id
        :param draw: 在图像上绘制输出的标志。(默认绘制矩形框)
        :return: 像素格式的手部关节位置列表；手部边界框
        """

        xList = []
        yList = []
        bbox = []
        bboxInfo =[]
        self.lmList = []
        if self.results.multi_hand_landmarks:
            myHand = self.results.multi_hand_landmarks[handNo]
            for id, lm in enumerate(myHand.landmark):
                h, w, c = img.shape
                px, py = int(lm.x * w), int(lm.y * h)
                xList.append(px)
                yList.append(py)
                self.lmList.append([px, py])
                if draw:
                    cv2.circle(img, (px, py), 5, (255, 0, 255), cv2.FILLED)
            xmin, xmax = min(xList), max(xList)
            ymin, ymax = min(yList), max(yList)
            boxW, boxH = xmax - xmin, ymax - yminbi
            bbox = xmin, ymin, boxW, boxH
            cx, cy = bbox[0] + (bbox[2] // 2), \
                     bbox[1] + (bbox[3] // 2)
            bboxInfo = {"id": id, "bbox": bbox,"center": (cx, cy)}

            if draw:
                cv2.rectangle(img, (bbox[0] - 20, bbox[1] - 20),
                              (bbox[0] + bbox[2] + 20, bbox[1] + bbox[3] + 20),
                              (0, 255, 0), 2)

        return self.lmList, bboxInfo

    def fingersUp(self):
        """
        查找列表中打开并返回的手指数。会分别考虑左手和右手
        ：return：竖起手指的数组(列表)，数组长度为5，
        其中，由大拇指开始数，立起标为1，放下为0。
        """
        if self.results.multi_hand_landmarks:
            myHandType = self.handType()
            fingers = []
            # Thumb
            if myHandType == "Right":
                if self.lmList[self.tipIds[0]][0] > self.lmList[self.tipIds[0] - 1][0]:
                    fingers.append(1)
                else:
                    fingers.append(0)
            else:
                if self.lmList[self.tipIds[0]][0] < self.lmList[self.tipIds[0] - 1][0]:
                    fingers.append(1)
                else:
                    fingers.append(0)

            # 4 Fingers
            for id in range(1, 5):
                if self.lmList[self.tipIds[id]][1] < self.lmList[self.tipIds[id] - 2][1]:
                    fingers.append(1)
                else:
                    fingers.append(0)
        return fingers

    def handType(self):
        """
        检查传入的手部是左还是右
        ：return: "Right" 或 "Left"
        """
        if self.results.multi_hand_landmarks:
            if self.lmList[17][0] < self.lmList[5][0]:
                return "Right"
            else:
                return "Left"
def Gesture_recognition(detector,Img):
    twist=Twist()
    rospy.init_node('finger_publisher',anonymous=True)
    while True:
        Img = detector.findHands(Img)
        lmList, bbox = detector.findPosition(Img)
        flag='other'
        if lmList:
            x_1, y_1 = bbox["bbox"][0], bbox["bbox"][1]
            x1, x2, x3, x4, x5 = detector.fingersUp()
            if (x2 == 1 and x3 == 1) and (x4 == 0 and x5 == 0 and x1 == 0):
                twist.linear.y = 2
                print ('右手2，向上跑，此时速度为y=2.5')
                cv2.putText(Img, "2_right", (x_1, y_1), cv2.FONT_HERSHEY_PLAIN, 3,
                            (0, 0, 255), 3)
                
                flag='right'
            elif (x2 == 1 and x3 == 1 and x4 == 1) and (x1 == 0 and x5 == 0):
                twist.linear.y = -5
                print ('3，向下跑，此时速度为y=5')
                cv2.putText(Img, "3", (x_1, y_1), cv2.FONT_HERSHEY_PLAIN, 3,
                            (0, 0, 255), 3)
                
                flag='stop'
            elif (x2 == 1 and x3 == 1 and x4 == 1 and x5 == 1) and (x1 == 0):
                twist.linear.x = -2
                twist.angular.z= 2
                print ('4指，逆时针画圆')
                cv2.putText(Img, "4_back", (x_1, y_1), cv2.FONT_HERSHEY_PLAIN, 3,
                            (0, 0, 255), 3)
               
                flag='back'
            elif x1 == 1 and x2 == 1 and x3 == 1 and x4 == 1 and x5 == 1:
                twist.linear.x = 2
                twist.angular.z= 2
                print ('五指，顺时针画圆')
                cv2.putText(Img, "5_Go", (x_1, y_1), cv2.FONT_HERSHEY_PLAIN, 3,
                            (0, 0, 255), 3)
                 
                flag='go'
            elif x2 == 1 and (x1 == 0, x3 == 0, x4 == 0, x5 == 0):
                twist.linear.x = 1.0
                print ('左手1，向右面跑，此时速度为x=2.5')
                cv2.putText(Img, "1_left", (x_1, y_1), cv2.FONT_HERSHEY_PLAIN, 3,
                            (0, 0, 255), 3)
                 
                flag='left'
            elif x1 and (x2 == 0, x3 == 0, x4 == 0, x5 == 0):
                twist.linear.x = -5
                print ('拳头，向左跑，此时速度为x=-5')
                cv2.putText(Img, "GOOD!", (x_1, y_1), cv2.FONT_HERSHEY_PLAIN, 3,
                            (0, 0, 255), 3)
                
                flag='other'
       

        pub=rospy.Publisher('tbmn_01/cmd_vel',Twist,queue_size=10)#给小乌龟发送速度指令
        pub.publish(twist)
        rate = rospy.Rate(10)#设置频率
        return Img,flag
if __name__ == '__main__':
    detector = HandDetector()
    capture = cv2.VideoCapture(0)

    while(True):
     ret,Img = capture.read()

     Img, handflag = Gesture_recognition(detector, Img)
     cv2.imshow('frame', Img)
 
     if cv2.waitKey(1) == ord('q'):
        break
