#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Vector3, Twist
from std_msgs.msg import Bool

from move_to_pose import PathFinderController
import numpy as np

import stanley_controller as stanley

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

# 
target_speed = 0.1  # [m/s]
prev_velocity = 0.0

import time
prev_time = 0

def currpos_callback(msg):
    # bool_subscriber()
    global index
    global human_detected


    global prev_velocity 
    global prev_time

    cx = msg.x
    cy = msg.y
    cyaw = msg.z
    last_target_idx = 

    state = stanley.State(cx, cy, cyaw, prev_velocity)

    
    # acceleration을 speed로 변환 시키기 위해서 사용
    # 처음 callback이 불렸을 때 0.1로 되고 그 이후부터 callback이 호출된 시간 간격을 사용하게 된다
    dt = 0.1
    if prev_time != 0:
        dt = time.time() - prev_time
    prev_time = time.time_ns() / 1_000_000_000 # get seconds in fractions

    rospy.loginfo("dt is {}".format(dt))

    map_val =  384  # 맵 크기. hybrid A*랑 2D 맵과 origin이 달라서 이렇게 차이 보정 해줘야 한다
    if index >= len(paths):
        print('End of path')
        exit()
    rospy.loginfo("current pos: x = {}, y = {}, z = {}".format(msg.x,msg.y,msg.z))
    twist_msg = Twist()

    rospy.loginfo("check is : {}".format(human_detected))


    
    
    if (human_detected == 1):
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0
    else:

        # path를 1개씩 넘기면 간격이 너무 작아서 4개씩 넘기도록 함
        # 목표 지점에 도착했는 확인 하는 조건문
        if(abs(paths[index][0]-msg.x) < 2 and abs(map_val-paths[index][1]-msg.y) < 2):
            index = index+4

        ai = stanley.pid_control(target_speed, state.v)
        di, target_idx = stanley.stanley_control(state, paths[0], paths[1], paths[2], index)

        linear_velocity = ai
        prev_velocity = linear_velocity

        # deg * rad/deg / sec = rad/sec
        angular_velocity = (di/57.29578)


        
        twist_msg.linear.x = linear_velocity
        twist_msg.angular.z = angular_velocity

    # twist_msg.angular.z = ((map_val-paths[index][1])-msg.y)/30
    print('path\'s coordinate: ({},{}, {})'.format(paths[index][0],map_val-paths[index][1], paths[index][2]))
    print('x = {}, y = {}'.format(msg.x,msg.y))
    print('lin_vel: ',twist_msg.linear.x)
    print('ang_vel: ',twist_msg.angular.z)

    if twist_msg.linear.x > 0.1:
        twist_msg.linear.x = 0.1
    if twist_msg.angular.z > 0.1:
        twist_msg.angular.z = 0.1


    twist_pub.publish(twist_msg)
    prev_velocity = twist_msg.linear.x
    


def currpos_subscriber():
    # rospy.init_node('currpos_subscriber_node')
    rospy.Subscriber('current_pose', Vector3, currpos_callback)
    # rospy.spin()
    # if (check == 1):
    #     twist_msg.linear.x = 0
    #     twist_msg.angular.z = 0
    # else:

    #     # path를 1개씩 넘기면 간격이 너무 작아서 4개씩 넘기도록 함
    #     # 목표 지점에 도착했는 확인 하는 조건문
    #     if(abs(paths[index][0]-msg.x) < 2 and abs(map_val-paths[index][1]-msg.y) < 2):
    #         index = index+4


###########################################
def tracloss_callback(msg):
    twist_msg = Twist()
    if (msg.data == True):
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0.05
        twist_pub.publish(twist_msg)

    print('lin_vel: ',twist_msg.linear.x)
    print('ang_vel: ',twist_msg.angular.z)



def tracking_loss():
    # rospy.init_node('currpos_subscriber_node')
    rospy.Subscriber('run_slam/loss', Bool, tracloss_callback)
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

        rospy.spin()

        rospy.loginfo("All paths received. Processing...")

    except rospy.ROSInterruptException:
        pass
