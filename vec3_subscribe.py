#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Vector3, Twist

def cot(x):
    if x==0:
        return 0
    else:
        return 1 / math.tan(x)

# 빈 리스트를 생성하여 모든 경로 시퀀스를 저장할 준비를 합니다.
paths = []
received_paths = False

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
    rospy.init_node('path_subscriber', anonymous=False)

    # 'vector3' 토픽을 구독하는 subscriber 생성
    print('Ready to go!')
    rospy.Subscriber('path_goal', Vector3, path_callback)

    while not rospy.is_shutdown() and not received_paths:
        rospy.loginfo("Waiting for paths...")
        rospy.sleep(5)
###########################################

# subscribing current position from stella slam
###########################################
flag = 0
def currpos_callback(msg):
    global flag
    index = flag
    if index >= len(paths):
        print('End of path')
        exit()
    rospy.loginfo("current pos: x = {}, y = {}, z = {}".format(msg.x,msg.y,msg.z))
    twist_msg = Twist()
    if(abs(paths[index][0]-msg.x)<2 and abs(384-paths[index][1]-msg.y)<2):
        flag = flag+4
    twist_msg.linear.x = 0.2
    # twist_msg.linear.x = (math.sqrt(math.pow(paths[index][0]-msg.x,2)+math.pow(384-paths[index][1]-msg.y,2)))/10
    '''
    if twist_msg.linear.x >= 3:
        twist_msg.linear.x = 3
    '''
    twist_msg.angular.z = ((384-paths[index][1])-msg.y)/30
    print('path\'s coordinate: ({},{})'.format(paths[index][0],384-paths[index][1]))
    print('x = {}, y = {}'.format(msg.x,msg.y))
    print('lin_vel: ',twist_msg.linear.x)
    print('ang_vel: ',twist_msg.angular.z)

    twist_pub.publish(twist_msg)

    '''
    # using current position and path, check if on path
    # skip 2 points in path
    for i in range(len(paths)):
        if(i%4!=0):
            continue
        # need to compare path[i] and msg
        while(True):
            if(abs(paths[i][0]-msg.x)<0.2 and abs(paths[i][1]-msg.y)<0.2):
                break
            twist_msg.linear.x = 0.25
            twist_msg.angular.z = paths[i][1]-msg.y
            print('x = {}, y = {}'.format(msg.x,msg.y))
            print('ang_vel: ',twist_msg.angular.z)
    '''

def currpos_subscriber():
    # rospy.init_node('currpos_subscriber_node')
    rospy.Subscriber('current_pose', Vector3, currpos_callback)
    rospy.spin()
    
    
###########################################

def process_paths():
    # 모든 경로 시퀀스가 수신되었을 때 호출되는 함수
    # paths 리스트에 있는 모든 경로 시퀀스를 반복하여 처리
    rospy.loginfo('process_paths called')

    prev = paths[0]

    # 경로 시퀀스 처리 예시: 각 점의 좌표 출력
    for i in range(len(paths)):
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
        # cot_val = cot(ang_vel/lin_vel)
        # print('angle diff: ', cot_val)

        
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
        # rospy.init_node('twist_publisher', anonymous=False)

        # Twist 메시지를 발행할 publisher 생성
        twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # subscriber started
        path_subscriber()
        currpos_subscriber()

        rospy.loginfo("All paths received. Processing...")
        # process_paths()

    except rospy.ROSInterruptException:
        pass
