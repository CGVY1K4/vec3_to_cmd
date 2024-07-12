#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Vector3, Twist
from std_msgs.msg import Bool

#from move_to_pose import PathFinderController
import numpy as np

# 사람 있는지 확인
human_detected = 0

# 빈 리스트를 생성하여 모든 경로 시퀀스를 저장할 준비를 합니다.
paths = []
received_paths = False

# subscribing boolean value from person_check node
###########################################
def bool_callback(msg):
    global human_detected
    rospy.loginfo("Received boolean message: {}".format(msg.data))
    if(msg.data == True):
        human_detected = 1
    else:
        human_detected = 0
    rospy.loginfo("check: {}".format(human_detected))

def bool_subscriber():
    # rospy.init_node("check_person_subscriber", anonymous=False)

    rospy.Subscriber('check_person', Bool, bool_callback)
    rospy.loginfo("Bool subscriber started.")
    # rospy.spin()
###########################################

# subscribing path from astar
###########################################
def path_callback(msg):
    # 토픽 메시지 수신 시 호출되는 콜백 함수
    # rospy.loginfo("Received Vector3 message: ({}, {}, {})".format(msg.x, msg.y, msg.z))
    # 수신된 Vector3 메시지를 경로 시퀀스에 추가합니다.
    paths.append((msg.x, msg.y, msg.z))

    global received_paths
    received_paths = True

def path_subscriber():
    # ROS 노드 초기화
    # rospy.init_node('path_subscriber', anonymous=False)

    # 'vector3' 토픽을 구독하는 subscriber 생성
    print('Ready to go!')
    rospy.Subscriber('path_goal', Vector3, path_callback)

    while not rospy.is_shutdown() and not received_paths:
        rospy.loginfo("Waiting for paths...")
        rospy.sleep(5)
###########################################
# subscribing current position from stella slam
###########################################



index = 0
direction = 0
angular_vel = 0
def currpos_callback(msg):
    global index
    global human_detected
    global direction
    global angular_vel
    linear = 1
    linear_check = 0
    threshold = 10

    # bool_subscriber()
    
    #oh_0123
    #map_val =  384  # 맵 y축 크기. hybrid A*랑 2D 맵과 origin이 달라서 이렇게 차이 보정 해줘야 한다
                    # x축은 보정 안 해줘도 된다

    #oh_0420
    map_val =  550

    

    if index >= len(paths):
        print('End of path')
        exit()
    
    rospy.loginfo("current pos: x = {}, y = {}, z = {}".format(msg.x,msg.y,msg.z))
    rospy.loginfo("check is : {}".format(human_detected))

    linear_vel = 0
    angular_vel = 0

    cp = [msg.x, msg.y]
    cp_x, cp_y = cp
    gp = [paths[index][0], map_val - paths[index][1]]           # hybrid A*에서는 왼쪽 아래가 (0,0)위치이고, 2d occupancy map에서는 왼쪽 위에가 (0,0)이어서 offset 적용해준 것임
    gp_x, gp_y = gp
    gp_yaw = paths[index][2]
    x_diff = gp_x-cp_x
    y_diff = gp_y-cp_y

    if not human_detected:
        linear_vel = 0.1        # linear velocity는 0.1로 설정한다

        # path를 1개씩 넘기면 간격이 너무 작아서 4개씩 넘기도록 함
        # 목표 지점에 도착했는 확인 하는 조건문
        if(abs(x_diff) < 5 and abs(y_diff) < 5):
            index = index+8
            x_diff = gp_x-cp_x      # 이거 여기서 왜 다시 한걸까
            y_diff = gp_y-cp_y
        
        # 좌우 이동
        if(abs(x_diff)> abs(y_diff)): 
            is_x_axis = True
            if(x_diff > 0):
                if(y_diff > 0):
                    direction = -1 # 우회전이 음수
                else:
                    direction = 1
                #path를 지나쳤을 때 후진
                # if(gp_x-cp_x > 0 ): # gp_x-cp_x > 0 일때 정상적인 주행
                #     linear = 1
                # else:               # path를 지나친 상황
                #     linear = -1
                #     angle = 0
            else:
                if(y_diff > 0):
                    direction = 1 # 좌회전이 양수
                else:
                    direction = -1
                # if(gp_x-cp_x > 0 ):
                #     linear = -1
                #     angle = 0
                # else:
                #     linear = 1
        # 위아래 이동
        elif(abs(x_diff) < abs(y_diff)):
            is_x_axis = False
            if(y_diff > 0): # 아래로 내려간다
                if(x_diff > 0):
                    direction = 1
                else:
                    direction =-1
                # if(gp_y-cp_y > 0 ):
                #     linear = 1
                # else:
                #     linear = -1
                #     angle = 0
            else:
                if(x_diff > 0):
                    direction = -1
                else:
                    direction = 1
                # if(gp_y-cp_y > 0 ):
                #     linear = -1
                #     angle = 0
                # else:
                #     linear = 1
            #direction *= -1 # 이거 한번 추가 해 봄
        
        angular_vel = direction * calculate_angular_velocity(cp, gp, is_x_axis)

    #path를 지나쳤을 때 후진을 유지
    if(linear == -1):
        if(linear_check < 10):
            linear == -1
            linear_check = linear_check + 1
        else:
            linear_check = 0
        
    twist_msg = Twist()
    twist_msg.linear.x = linear_vel * linear
    twist_msg.angular.z = angular_vel

    print('path\'s coordinate: ({},{}, {})'.format(gp_x, gp_y, gp_yaw))
    print('x = {}, y = {}'.format(cp_x, cp_y))
    print('lin_vel: ', linear_vel)
    print('ang_vel: ', angular_vel)

    twist_pub.publish(twist_msg)


# return rad/sec value
def calculate_angular_velocity(cp, gp, is_x_axis):
    cp_x, cp_y = cp
    gp_x, gp_y = gp

    import math
    # 좌우로 이동하는 것이면 atan2(y,x)
    # 상하로 이동하는 것이면 atan2(x,y)



    x_diff = gp_x-cp_x
    y_diff = gp_y-cp_y

    if is_x_axis:
        abs_theta = abs(math.atan2(y_diff, x_diff))
        #abs_theta = cp_y-gp_y
    else:
        abs_theta = abs(math.atan2(x_diff, y_diff))
        #abs_theta = cp_x-gp_x
    
    abs_theta  = abs_theta*57.2958
    deg_per_sec = 0
    print(abs_theta)
    if abs_theta < 1:
        deg_per_sec = 0
    elif abs_theta < 2:
        deg_per_sec = 0.5 # 0.1
    elif abs_theta < 5:
        deg_per_sec = 1 #0.3
    elif abs_theta < 8:
        deg_per_sec = 3 #0.5
    elif abs_theta < 12:
        deg_per_sec = 4  # 1
    elif abs_theta < 20:
        deg_per_sec = 5  #2
    elif abs_theta < 30:
        deg_per_sec = 6 # 2
    elif abs_theta < 45:
        deg_per_sec = 9 # 5
    elif abs_theta < 60:
        deg_per_sec = 12 # 7
    else:
        deg_per_sec = 15 # 10
    # if abs_theta< 0.5:
    #     if abs_theta <0.6:
    #         deg_per_sec = 1
    #     elif abs_theta < 0.8:
    #         deg_per_sec = 2
    #     elif abs_theta < 1.0:
    #         deg_per_sec = 3
    #     elif abs_theta < 1.2:
    #         deg_per_sec = 4
    #     elif abs_theta < 1.7:
    #         deg_per_sec = 6
    #     else:
    #         deg_per_sec = 8

    return deg_per_sec/57.2958 # conversion to radian


def currpos_subscriber():
    # rospy.init_node('currpos_subscriber_node')
    rospy.Subscriber('current_pose', Vector3, currpos_callback)
    # rospy.spin()

###########################################
def trackloss_callback(msg):
    global direction
    global angular_vel
    twist_msg = Twist()
    if (msg.data == True):
        #후진
        twist_msg.linear.x = -0
        twist_msg.angular.z = angular_vel / 2
        #주행 중이던 angular_vel 을 그대로 준다.
        # twist_msg.linear.x = 0
        # twist_msg.angular.z = angular_vel
        twist_pub.publish(twist_msg)

    print('lin_vel: ',twist_msg.linear.x)
    print('ang_vel: ',twist_msg.angular.z)



def tracking_loss():
    # rospy.init_node('currpos_subscriber_node')
    rospy.Subscriber('run_slam/loss', Bool, trackloss_callback)
    rospy.loginfo("df")
    # rospy.spin()    
###########################################

def process_paths():
    # 모든 경로 시퀀스가 수신되었을 때 호출되는 함수
    # paths 리스트에 있는 모든 경로 시퀀스를 반복하여 처리
    rospy.loginfo('process_paths called')

    prev = paths[0]

    # 경로 시퀀스 처리 예시: 각 점의 좌표 출력
    for i in range(len(paths)):

        # twist 메시지 발행
        if(i == 0):
            rospy.loginfo("Path %d:", i+1)
            rospy.loginfo("Point: (%f, %f, %f)", prev[0], prev[1], prev[2])
            continue
        curr = paths[i]
        lin_vel = curr[0]-prev[0]
        ang_vel = curr[1]-prev[1]
        rospy.loginfo("Path %d", i+1)
        rospy.loginfo("previous point: (%f, %f, %f)", prev[0], prev[1], prev[2])
        rospy.loginfo("current point: (%f, %f, %f)", curr[0], curr[1], curr[2])
        rospy.loginfo("tangential: %f, normal: %f", lin_vel, ang_vel)
        
        
        # Twist 메시지 생성
        twist_msg = Twist()
        # twist_msg.linear.x = lin_vel
        twist_msg.linear.x = 0.25
        twist_msg.angular.z = ang_vel*1

        # twist 메시지 발행
        twist_pub.publish(twist_msg)
        # rospy.sleep(0.14)
        # current position subscribe, compare with target point
        
        prev = curr

if __name__ == '__main__':
    try:
        # ROS 노드 초기화
        rospy.init_node('twist_publisher', anonymous=False)

        # Twist 메시지를 발행할 publisher 생성
        twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        #subscriber started
        path_subscriber()
        currpos_subscriber()
        bool_subscriber()
        tracking_loss()

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
