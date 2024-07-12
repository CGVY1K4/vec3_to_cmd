#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Vector3, Twist
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Int32
import numpy as np
import math

# 사람 있는지 확인
human_detected = 0

# 빈 리스트를 생성하여 모든 경로 시퀀스를 저장할 준비를 합니다.
paths = []
# received_paths = False


### oh_0123
#map_val =  384  # 맵 y축 크기. hybrid A*랑 2D 맵과 origin이 달라서 이렇게 차이 보정 해줘야 한다
                # x축은 보정 안 해줘도 된다

### oh_0420
map_val =  550

# subscribing boolean value from person_check node
###########################################
def check_person_callback(msg):
    global human_detected
    rospy.loginfo("Received boolean message: {}".format(msg.data))
    if(msg.data == True):
        human_detected = 1
    else:
        human_detected = 0
    rospy.loginfo("check: {}".format(human_detected))

def path_callback(msg):
    # rospy.loginfo("Received Vector3 message: ({}, {}, {})".format(msg.x, msg.y, msg.z))
    global paths
    global map_val
    paths.clear()
    for pose in msg.poses:
        # rospy.loginfo(f"{pose.pose.position}")
        pos = pose.pose.position



        paths.append((pos.x, map_val - pos.y))
        # paths.append((msg.x, msg.y, msg.z))

    print(paths)
    
    global index
    index = 0

    # twist_msg = Twist()
    # twist_msg.linear.x = 0
    # twist_msg.angular.z = 0
    # twist_pub.publish(twist_msg)

    

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
    global paths
    linear = 1
    linear_check = 0
    threshold = 10


    if index >= len(paths):
        # print('End of path')
        return
    
    
    rospy.loginfo("current pos: x = {}, y = {}, z = {}".format(msg.x,msg.y,msg.z))
    rospy.loginfo("check is : {}".format(human_detected))

    linear_vel = 0
    angular_vel = 0

    cp = [msg.x, msg.y]
    cp_x, cp_y = cp
    gp = [paths[index][0], paths[index][1]]           # hybrid A*에서는 왼쪽 아래가 (0,0)위치이고, 2d occupancy map에서는 왼쪽 위에가 (0,0)이어서 offset 적용해준 것임
    gp_x, gp_y = gp
    # gp_yaw = paths[index][2]
    x_diff = gp_x-cp_x
    y_diff = gp_y-cp_y
    ############
    yaw = msg.z
    yaw = yaw+(math.pi/2)
    if(yaw > math.pi):
        yaw = yaw - 2*math.pi
    yaw = yaw*(-1)

    #linear_vel = 0.1        # linear velocity는 0.1로 설정한다

    # path를 1개씩 넘기면 간격이 너무 작아서 4개씩 넘기도록 함
    # 목표 지점에 도착했는 확인 하는 조건문
    if(abs(x_diff) < 5 and abs(y_diff) < 5):
        index = index+8
        x_diff = gp_x-cp_x      # 이거 여기서 왜 다시 한걸까
        y_diff = gp_y-cp_y

    theta = math.atan2(y_diff, x_diff)
    final_theta = theta - yaw
    if(abs(final_theta) > math.pi):
        final_theta = yaw/abs(yaw)*(2*math.pi - abs(final_theta))
    if(abs(final_theta) > 0.5):
        final_theta = 0.5 * final_theta / abs(final_theta)
    final_theta = -1 * final_theta   

    if(index < 50 and abs(final_theta) > (math.pi / 6)):
        linear_vel = 0
        final_theta = 0.1*(final_theta/abs(final_theta))
    else:
        linear_vel = 0.1
    
    twist_msg = Twist()
    if(human_detected):
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0

    else:
        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = final_theta

    angular_vel = final_theta

    print('path\'s coordinate: ({},{}, {})'.format(gp_x, gp_y, 'yaw'))
    print('x = {}, y = {}'.format(cp_x, cp_y))
    print('lin_vel: ', linear_vel)
    print('ang_vel: ', angular_vel)

    twist_pub.publish(twist_msg)

    index_msg = Int32()
    index_msg = index
    path_index_pub.publish(index_msg)


###########################################
def trackloss_callback(msg):
    global direction
    global angular_vel
    twist_msg = Twist()
    if (msg.data == True):
        #후진
        twist_msg.linear.x = -0.05
        twist_msg.angular.z = angular_vel / 4 
        #주행 중이던 angular_vel 을 그대로 준다.
        # twist_msg.linear.x = 0
        # twist_msg.angular.z = angular_vel
        twist_pub.publish(twist_msg)

    print('lin_vel: ',twist_msg.linear.x)
    print('ang_vel: ',twist_msg.angular.z)

    # global paths
    # paths = []

###########################################


if __name__ == '__main__':
    try:
        rospy.init_node('vec3_to_cmd_node', anonymous=False)

        twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        path_index_pub = rospy.Publisher('path_index', Int32, queue_size=10)

        rospy.Subscriber('/run_hybrid_astar/searched_path', Path, path_callback)
        rospy.Subscriber('current_pose', Vector3, currpos_callback)
        rospy.Subscriber('check_person', Bool, check_person_callback)
        rospy.Subscriber('run_slam/loss', Bool, trackloss_callback)

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
