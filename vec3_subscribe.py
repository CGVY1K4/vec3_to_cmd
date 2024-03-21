#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Vector3, Twist
from std_msgs.msg import Bool

check = 0

# 빈 리스트를 생성하여 모든 경로 시퀀스를 저장할 준비를 합니다.
paths = []
received_paths = False

# subscribing boolean value from person_check node
###########################################
def bool_callback(msg):
    global check
    rospy.loginfo("Received boolean message: {}".format(msg.data))
    if(msg.data == True):
        check = 1
    else:
        check = 0
    rospy.loginfo("check: {}".format(check))

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
flag = 0
flag2 = 0
def currpos_callback(msg):
    # bool_subscriber()
    global flag
    global flag2
    global check
    index = flag
    if index >= len(paths):
        print('End of path')
        exit()
    rospy.loginfo("current pos: x = {}, y = {}, z = {}".format(msg.x,msg.y,msg.z))
    twist_msg = Twist()

    rospy.loginfo("check is : {}".format(check))

    if (check == 1):
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0
    else:
        if(abs(paths[index][0]-msg.x)<2 and abs(384-paths[index][1]-msg.y)<2):
            flag = flag+4
        twist_msg.linear.x = 0.1
        '''
        if twist_msg.linear.x >= 3:
            twist_msg.linear.x = 3
        '''
        
        if (384-paths[index][1]-msg.y)>0.5:
            if(paths[index][0]-msg.x)>0.5:
                if(384-paths[index][1]-msg.y) < 0.8:
                    twist_msg.angular.z = -0.07
                elif(384-paths[index][1]-msg.y) < 1.0:
                    twist_msg.angular.z = -0.1
                elif(384-paths[index][1]-msg.y) < 1.3:
                    twist_msg.angular.z = -0.12
                elif(384-paths[index][1]-msg.y) < 1.6:
                    twist_msg.angular.z = -0.15
                else:
                    twist_msg.angular.z = -0.17
        else:
            twist_msg.angular.z = 0
        

    # twist_msg.angular.z = ((384-paths[index][1])-msg.y)/30
    print('path\'s coordinate: ({},{})'.format(paths[index][0],384-paths[index][1]))
    print('x = {}, y = {}'.format(msg.x,msg.y))
    print('lin_vel: ',twist_msg.linear.x)
    print('ang_vel: ',twist_msg.angular.z)

    twist_pub.publish(twist_msg)


def currpos_subscriber():
    # rospy.init_node('currpos_subscriber_node')
    rospy.Subscriber('current_pose', Vector3, currpos_callback)
    # rospy.spin()

###########################################
def tracloss_callback(msg):
    rospy.loginfo("check1")
    twist_msg = Twist()
    rospy.loginfo("check2")
    if (msg.data == True):
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0.1
        twist_pub.publish(twist_msg)
    rospy.loginfo("check3")

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
