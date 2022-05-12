#!/usr/bin/env python
# -*- coding: utf-8 -*-

####################################################################
# 프로그램명 : hough_drive.py
# 작 성 자 : 자이트론
# 생 성 일 : 2020년 08월 12일
# 수 정 일 : 2021년 03월 16일
# 검 수 인 : 조 이현
# 본 프로그램은 상업 라이센스에 의해 제공되므로 무단 배포 및 상업적 이용을 금합니다.
####################################################################

import time
import rospy, rospkg
import numpy as np
import cv2, random, math
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image
from yolov3.msg import BoundingBoxes, BoundingBox

from movingAverage import MovingAverage


import sys
import os
import signal

def signal_handler(sig, frame):
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

xycar_motor_msg = xycar_motor()
image = np.empty(shape=[0])
bridge = CvBridge()
pub = None
Width = 640
Height = 480
Offset = 360
Gap = 40
obj_id = -1
obj_coordinate = None
detect_size = 415


def img_callback(data):
    global image    
    image = bridge.imgmsg_to_cv2(data, "bgr8")


# def yolo_trt_callback(data) :
#     global obj_id, obj_coordinate
#     max_box_size = 0
#     for bbox in data.bounding_boxes:
#         # TODO: get max size box
#         x_size = abs(bbox.xmax - bbox.xmin)
#         print("bbox ID:", bbox.id, "x_size:",x_size, "xmin:",bbox.xmin,"xmax:",bbox.xmax)
#         if max_box_size < x_size:
#             max_box_size = x_size
#             obj_id = bbox.id
#             obj_coordinate = {'xmin':min(bbox.xmin,bbox.xmax),
#                                 'xmax': max(bbox.xmin,bbox.xmax),
#                                 'ymin': min(bbox.ymin,bbox.ymax),
#                                 'ymax': max(bbox.ymin,bbox.ymax)}

def yolo_trt_callback(data) :
    global obj_id, obj_coordinate
    for bbox in data.bounding_boxes:
        obj_id = bbox.id
        if obj_id == 5:
            size_thrs = 35*85#traffic_thres 40 90, 26 36, 28 39
        else: 
            size_thrs = 25 * 35
        size = (bbox.xmax-bbox.xmin)*(bbox.ymax-bbox.ymin)
        
        if size > size_thrs:
            obj_coordinate = {'xmin':min(bbox.xmin,bbox.xmax),
                                'xmax': max(bbox.xmin,bbox.xmax),
                                'ymin': min(bbox.ymin,bbox.ymax),
                                'ymax': max(bbox.ymin,bbox.ymax)}

# publish xycar_motor msg
def drive(Angle, Speed): 
    global pub, xycar_motor_msg
    xycar_motor_msg = xycar_motor()
    xycar_motor_msg.angle = Angle
    xycar_motor_msg.speed = Speed
    pub.publish(xycar_motor_msg)


# draw lines
def draw_lines(img, lines):
    global Offset
    for line in lines:
        x1, y1, x2, y2 = line[0]
        color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
        img = cv2.line(img, (x1, y1+Offset), (x2, y2+Offset), color, 2)
    return img


# draw rectangle
def draw_rectangle(img, lpos, rpos, offset=0):
    center = (lpos + rpos) / 2

    cv2.rectangle(img, (lpos - 5, 15 + offset),
                       (lpos + 5, 25 + offset),
                       (0, 255, 0), 2)
    cv2.rectangle(img, (rpos - 5, 15 + offset),
                       (rpos + 5, 25 + offset),
                       (0, 255, 0), 2)
    cv2.rectangle(img, (center-5, 15 + offset),
                       (center+5, 25 + offset),
                       (0, 255, 0), 2)    
    cv2.rectangle(img, (315, 15 + offset),
                       (325, 25 + offset),
                       (0, 0, 255), 2)
    return img


# draw bbox
def draw_bbox(img):
    global Width, Height, detect_size, obj_coordinate
    if obj_coordinate is None:
        return
    xmin = int(obj_coordinate["xmin"]*Width/detect_size)
    ymin = int(obj_coordinate["ymin"]*Height/detect_size)
    xmax = int(obj_coordinate["xmax"]*Width/detect_size)
    ymax = int(obj_coordinate["ymax"]*Height/detect_size)

    cv2.rectangle(img, (xmin, ymin), (xmax, ymax), (255, 0, 0))

# left lines, right lines
def divide_left_right(lines):
    global Width
    low_slope_threshold = 0.3
    high_slope_threshold = 10

    slopes = []
    new_lines = []

    for line in lines:
        x1, y1, x2, y2 = line[0]
        if x2 - x1 == 0:
            slope = 0
        else:
            slope = float(y2 - y1) / float(x2 - x1)
        if (abs(slope) > low_slope_threshold) and (abs(slope) < high_slope_threshold):
            slopes.append(slope)
            new_lines.append(line[0])
    
    # divide lines left to right
    left_lines = []
    right_lines = []
    left_x_sum = 0
    right_x_sum = 0

    # for j in range(len(slopes)):
    #     Line = new_lines[j]
    #     slope = slopes[j]
    #     x1, y1, x2, y2 = Line
    #     if slope < 0:
    #         left_lines.append([Line.tolist()])
    #         left_x_sum += (x1 + x2)/2
    #     else:
    #         right_lines.append([Line.tolist()])
    #         right_x_sum += (x1 + x2)/2

    # if len(left_lines) != 0 and len(right_lines) != 0:
    #     left_x_avg = left_x_sum / len(left_lines)
    #     right_x_avg = right_x_sum / len(right_lines)

    #     if left_x_avg > right_x_avg:
    #         left_lines = []
    #         right_lines = []

    for j in range(len(slopes)):
        Line = new_lines[j]
        slope = slopes[j]
        x1, y1, x2, y2 = Line
        if slope < 0 and (x2 < Width/2 + 40):
            left_lines.append([Line.tolist()])
        elif slope > 0 and (x1 > Width/2 - 40):
            right_lines.append([Line.tolist()])
    
    return left_lines, right_lines


# get average m, b of lines
def get_line_params(lines):
    # sum of x, y, m
    x_sum = 0.0
    y_sum = 0.0
    m_sum = 0.0

    size = len(lines)
    if size == 0:
        return 0, 0

    for line in lines:
        x1, y1, x2, y2 = line[0]

        x_sum += x1 + x2
        y_sum += y1 + y2
        m_sum += float(y2 - y1) / float(x2 - x1)

    x_avg = x_sum / (size * 2)
    y_avg = y_sum / (size * 2)
    m = m_sum / size
    b = y_avg - m * x_avg

    return m, b


# get lpos, rpos
def get_line_pos(img, lines, left=False, right=False):
    global Width, Height
    global Offset, Gap

    m, b = get_line_params(lines)

    if m == 0 and b == 0:
        if left:
            pos = 0
        if right:
            pos = Width
    else:
        y = Gap / 2
        pos = (y - b) / m

        b += Offset
        x1 = (Height - b) / float(m)
        x2 = ((Height/2) - b) / float(m)

        cv2.line(img, (int(x1), Height), (int(x2), (Height/2)), (255, 0,0), 3)

    return img, int(pos), m


# show image and return lpos, rpos
def process_image(frame):
    global Width
    global Offset, Gap

    # gray
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    # blur
    kernel_size = 5
    blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)

    # canny edge
    low_threshold = 150
    high_threshold = 200

    edge_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)
    #cv2.imshow('edge_img', edge_img)

    # HoughLinesP
    roi = edge_img[Offset : Offset+Gap, 0 : Width]
    #cv2.imshow('roi', roi)

    all_lines = cv2.HoughLinesP(roi,1,math.pi/180,40,35,10)

    # divide left, right lines
    if all_lines is None:
        #cv2.imshow('calibration', frame)
        return 0, 640, 0, 0
    left_lines, right_lines = divide_left_right(all_lines)

    # get center of lines
    frame, lpos, l_slope = get_line_pos(frame, left_lines, left=True)
    frame, rpos, r_slope = get_line_pos(frame, right_lines, right=True)

    # draw lines
    frame = draw_lines(frame, left_lines)
    frame = draw_lines(frame, right_lines)
    frame = cv2.line(frame, (230, 235), (410, 235), (255,255,255), 2)
                                 
    # draw rectangle
    frame = draw_rectangle(frame, lpos, rpos, offset=Offset)

    # draw bbox
    draw_bbox(frame)
    # show image
    # cv2.imshow('calibration', frame)

    return lpos, rpos, l_slope, r_slope

def get_stop_line(frame):
    
    offset1 = 380
    gap = 20
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    roi = gray[offset1:offset1+gap+1, :]

    num = (roi<100)
    if num.sum() > 2600:
        return True
    return False

def get_stop_sign(frame):
    global Width, Height
    global obj_coordinate, detect_size
    if obj_coordinate is None:
        return False
    x_ratio = float(Width)/float(detect_size)
    y_ratio = float(Height)/float(detect_size)
    xmin = obj_coordinate['xmin']
    xmax = obj_coordinate['xmax']
    ymin = obj_coordinate['ymin']
    ymax = obj_coordinate['ymax']
    x = int(xmin*x_ratio)
    y = int(ymin*y_ratio)
    w = int((xmax-xmin)*x_ratio)
    h = int((ymax-ymin)*y_ratio)
    traffic_light_roi = frame[y:y+h, x:x+w]
    
    offset = traffic_light_roi.shape[0]/3
    green_box = traffic_light_roi[:offset, ...]
    yellow_box = traffic_light_roi[offset:offset*2, ...]
    red_box = traffic_light_roi[offset*2:, ...]
    
    mean = np.array([np.mean(green_box), np.mean(yellow_box), np.mean(red_box)])
    if mean.argmax() == 2:
        return True
    else:
        return False 



def get_steer_angle(curr_position, lslope, rslope):
    # Lane tracking algorithm here

    k = 1.0

    if -0.2 < curr_position < 0.2 :  # 좀 더 천천히 조향해도 괜찮은 상황
        k = 1.0
    
    elif curr_position > 0.4 or curr_position < -0.4 :  # 신속하게 가운데로 들어와야 함
        k = 4.0

    else:   # 그 중간의 경우 계수는 linear 변화
        k = 20.0 * abs(curr_position) - 3.0

    steer_angle = k * math.atan(curr_position)* 180 / math.pi

    return steer_angle


def velocity_control(l_slope, r_slope):

    if abs(l_slope) < 0.45 or abs(r_slope) < 0.45:
        speed = 15

    else:
        speed = 50

    return speed


def drive_normal(direction=None, time=0.0):
    global pub
    global image
    global sleep_rate
    global Width, Height
    global xycar_motor_msg
    global steer_mm, steering_gain
    global prev_angle
    
    
    steering_gain = 0.8
    steer_mm = MovingAverage(10)
    rate = rospy.Rate(sleep_rate)
    cnt = 0
    if time == 0.0:
        max_cnt = 1# drive only one loop
    else:
        max_cnt = sleep_rate * time# drive 'time' seconds

    while cnt < max_cnt:
        while not image.size == (640*480*3):
            continue
        print("drive_normal")

        lpos, rpos, lslope, rslope = process_image(image)
        
        if direction is not None and direction is "left":
            rpos = lpos + 470
        elif direction is not None and direction is "right":
            lpos = rpos - 470

        position = (float(lpos + rpos - Width)) / Width

        steer_angle = steering_gain * get_steer_angle(position, lslope, rslope)
        if steer_angle < 0 and rpos == 640:
            steer_angle = -10
        elif steer_angle > 0 and lpos == 0:
            steer_angle = 10

        if lpos == 0 and rpos == 640:
            steer_angle = prev_angle

        steer_mm.add_sample(steer_angle)
        wmm_angle = steer_mm.get_wmm()
        speed = 5
        xycar_motor_msg.angle = wmm_angle
        xycar_motor_msg.speed = speed
        # print("lpos = ", lpos, "rpos = ", rpos, "position = ", position, "angle = ", steer_angle, "wmm_angle = ", wmm_angle)
        pub.publish(xycar_motor_msg)

        prev_angle = steer_angle

        cnt += 1
        rate.sleep()


def drive_left():
    print("drive_left")
    drive_normal(direction="left", time=2.5)


def drive_right():
    print("drive_right")
    drive_normal(direction="right", time=2.5)


def drive_stop():
    print("drive_stop")
    stop_proc()

def stop_proc(time=5.0):
    # time is max trial time(seconds) to stop
    print("stop_process")
    global pub
    global image
    global sleep_rate
    global xycar_motor_msg
    global prev_angle
    global obj_id

    find_stop_line = False
    proc_count = 0
    max_count = time * sleep_rate
    
    while proc_count < max_count:
        while not image.size == (640*480*3):
            continue
        print("process count", proc_count)
        if find_stop_line and get_stop_line(image):
            print("stop!!!")
            xycar_motor_msg.angle = prev_angle
            xycar_motor_msg.speed = 0
            pub.publish(xycar_motor_msg)
            time.sleep(5)
            break# stop process will be end after sleep 5 seconds

        if (obj_id == 2 or obj_id == 3):
            find_stop_line = True
        elif obj_id == 5 and get_stop_sign(image):
            find_stop_line = True
        else:
            drive_normal()# it takes 1/sleep_rate seconds
        
        proc_count += 1
    
    drive_normal(time=2.0)

def find_cross_walk():
    print("find_cross_walk")
    stop_proc()
    
def find_u_turn():
    print("find_u_turn")
    drive_normal()

def find_traffic_light():
    print("find_traffic_light")
    stop_proc()


def start():
    global pub
    global image
    global Width, Height
    global prev_angle
    global steer_mm, steering_gain
    global sleep_rate
    global obj_id

    time.sleep(9)

    prev_angle = 0
    sleep_rate = 12# while loop will iterate "sleep_rate" times in a second

    rospy.init_node('auto_drive')
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

    image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    rospy.Subscriber('/yolov3/detections', BoundingBoxes, yolo_trt_callback, queue_size=1)
    print "---------- Xycar A2 v1.0 ----------"

    while True:
        while not image.size == (640*480*3):
            continue
        
        # print("x1: {}, y1: {}, x2: {}, y2: {}".format(round(obj)))
        print("start loop obj id",obj_id)
        if obj_id != -1:
            if obj_id == 0:
                drive_left()
            elif obj_id == 1:
                drive_right()
            elif obj_id == 2:
                drive_stop()
            elif obj_id == 3:
                find_cross_walk()
            elif obj_id == 4:
                find_u_turn()
            elif obj_id == 5:
                find_traffic_light()
            obj_id = -1
        else:
            drive_normal()

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    rospy.spin()

if __name__ == '__main__':

    start()


