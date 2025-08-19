#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import math
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion, Pose
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int8, Bool, Float32, Int16MultiArray
from tf.transformations import quaternion_from_euler

# ---------------------------
# [1] 전역 변수 선언
# ---------------------------

# 자율주행 관련 변수
x_meter = 0
y_meter = 0
z_orient = 0
w_orient = 0

destination = 0  # 목적지 (1,2,3,4)
is_reached = 0
kill_switch = False
move_client = None
current_goal = None

# 기체 추종 관련 변수
x_cam = 0
y_cam = 0
x_slam = 0
y_slam = 0

# 드론 데이터 관련 변수
height = 0.0    
battery = 0.0

# 시나리오 제어 변수
take_off_scenario = False
landing_scenario = False
start_check = False

# ---------------------------
# [2] 좌표 정의 (index 0 미사용)
# ---------------------------
point_array = [
    (0, 0, 0, 0),               # 0 - 미사용
    (2.2184, 7.074, -0.213, 0.977),  # 1 - 격납고
    (4.879, 7.404, -0.842, 0.539),  # 2 - 승하차장
    (3.697, 7.363, 0.9716, 0.2366),  # 3 - 착륙장
    (3.645, 7.5836, -0.366, 0.9306),  # 4 - 이륙장
    (0, 0, 0, 0)                # 4 - 충전소 (좌표 미설정)
]

waypoint_1A = (3.821, 6.224, -0.247, 0.968)
waypoint_1B = (4.5814, 7.0412, -0.90956, 0.4156)
waypoint_1C = (4.32447, 6.97966, -0.288, 0.9576)
waypoint_1D = (4.1786, 6.263, -0.9187, 0.395)

waypoint_2A = (3.745, 7.597, -0.451, 0.893)
waypoint_2B = (3.745, 7.597, -0.451, 0.893)
waypoint_2C = (3.745, 7.597, -0.451, 0.893)
waypoint_2D = (3.745, 7.597, -0.451, 0.893)

# ---------------------------
# [3] 함수
# ---------------------------

# 자율주행 관련 함수
def getAmclPose(data):
    global x_meter, y_meter, z_orient, w_orient
    x_meter = data.pose.pose.position.x
    y_meter = data.pose.pose.position.y
    z_orient = data.pose.pose.orientation.z
    w_orient = data.pose.pose.orientation.w

def getDestination(data):
    global destination
    destination = data.data

def getNavStatus(data):
    global is_reached
    if data.status.status == 3:
        is_reached = 1

def pubNavGoal(pose):
    global move_client, current_goal, kill_switch
    if move_client is None:
        move_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        move_client.wait_for_server()
        rospy.loginfo("Connected to move_base server")

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = Pose(Point(pose[0], pose[1], 0), Quaternion(0, 0, pose[2], pose[3]))
    current_goal = goal

    if not kill_switch:
        move_client.send_goal(goal)
        wait = move_client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available or goal could not be reached")
            rospy.signal_shutdown("Action server not available or goal could not be reached")
    else:
        rospy.logwarn("사람 감지로 goal 전송 중지됨")

def move(target_pose, waypoint_name="목적지"):
    global is_reached    
    pubNavGoal(target_pose)
    while not rospy.is_shutdown():
        if is_reached == 1:
            is_reached = 0
            rospy.loginfo("{} 도착".format(waypoint_name))
            break
        rospy.sleep(0.1)

def kill_callback(msg):
    global kill_switch, move_client, current_goal
    if msg.data and not kill_switch:
        rospy.logwarn("[안전] 사람 감지! 이동 중지.")
        kill_switch = True
        if move_client is not None:
            move_client.cancel_all_goals()
    elif not msg.data and kill_switch:
        rospy.loginfo("사람 감지 해제: 이동 재개.")
        kill_switch = False
        if move_client is not None and current_goal is not None:
            move_client.send_goal(current_goal)


# 기체 추종 관련 함수
def convert_cam_to_slam(x_cam, y_cam):
    x_convert = 4.049 - ((float(x_cam) - 187.96) / (725.54 - 187.96)) * (4.049 - 3.183) # 좌우 반전
    y_convert = 7.107 + (682.32 - float(y_cam)) / (682.32 - 50.18) * (8.126 - 7.107)
    return x_convert, y_convert

def drone_position_callback(msg):
    global x_cam, y_cam, x_slam, y_slam
    x_cam = msg.data[0]
    y_cam = msg.data[1]
    x_slam, y_slam = convert_cam_to_slam(x_cam, y_cam)
    rospy.loginfo("변환된 SLAM 좌표: x={:.3f}, y={:.3f}".format(x_slam, y_slam))

def drone_landing():
    global x_slam, y_slam

    rospy.loginfo("기체 추종 모드 시작")
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        try:
            amcl_msg = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped, timeout=1.0)
            x_meter = amcl_msg.pose.pose.position.x
            y_meter = amcl_msg.pose.pose.position.y
        except rospy.ROSException:
            rospy.logwarn("현재 위치를 가져오지 못했습니다. 재시도...")
            continue

        if x_slam == 0 and y_slam == 0:
            rospy.logwarn("드론 좌표 수신 대기중...")
            rate.sleep()
            continue

        # 드론 방향을 향하는 yaw 각도 계산 (라디안)
        delta_x = x_slam - x_meter
        delta_y = y_slam - y_meter
        yaw = math.atan2(delta_y, delta_x)

        # yaw를 quaternion (x, y, z, w)로 변환 (roll=0, pitch=0)
        quat = quaternion_from_euler(0, 0, yaw)
        z_orient = quat[2]
        w_orient = quat[3]

        # target_angle = (x_meter, y_meter, z_orient, w_orient)
        target_pose = (x_slam, y_slam, z_orient, w_orient)

        pubNavGoal(target_pose)

        dist_to_target = math.sqrt((x_slam - x_meter) ** 2 + (y_slam - y_meter) ** 2)
        dist_pub.publish(dist_to_target)

        rospy.loginfo("로봇:(%.2f, %.2f) 드론:(%.2f, %.2f)  yaw: %.2f rad, 거리: %.2fm",
                      x_meter, y_meter, x_slam, y_slam, yaw, dist_to_target)

        if dist_to_target < 0.2 and height < 10:
            rospy.Subscriber('/sensor/height', Float32, height_callback, queue_size=1)
            rospy.loginfo("추종 종료")
            break

        rate.sleep()

# 관제 신호 대기 함수
def getStartCheck(data):
    global start_check
    if data.data == True:
        start_check = True

def waitStarting():
    global start_check
    rospy.loginfo("승인 신호 대기 중")
    while not start_check and not rospy.is_shutdown():
        rospy.sleep(0.1)
    start_check = False
    rospy.loginfo("승인 신호 수신: 진행\n")

# 드론 데이터 관련 함수
def battery_callback(msg):
    global battery
    battery = msg.data

def height_callback(msg):
    global height
    height = msg.data

# 시나리오 콜백 함수
def takeoff_callback(msg):
    global take_off_scenario
    take_off_scenario = msg.data

def landing_callback(msg):
    global landing_scenario
    landing_scenario = msg.data

# ---------------------------
# [4] ROS 노드 및 토픽 Subsceribe/Publish
# ---------------------------

rospy.init_node("autodrive_ctrl_node")
rate = rospy.Rate(10)

rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, getAmclPose, queue_size=1)
rospy.Subscriber("/destination", Int8, getDestination, queue_size=1)
rospy.Subscriber("/move_base/result", MoveBaseActionResult, getNavStatus, queue_size=1)
rospy.Subscriber("/kill", Bool, kill_callback, queue_size=1)
rospy.Subscriber("/start_check", Bool, getStartCheck, queue_size=1)

rospy.Subscriber("/take_off_scenario", Bool, takeoff_callback, queue_size=1) # 이륙 시나리오
rospy.Subscriber("/landing_scenario", Bool, landing_callback, queue_size=1)  # 착륙 시나리오

rospy.Subscriber('/drone_center', Int16MultiArray, drone_position_callback, queue_size=1)   #autodrive_vision -> Publish 필요!!

rospy.Subscriber('/sensor/battery', Float32, battery_callback, queue_size=1)
rospy.Subscriber('/sensor/height', Float32, height_callback, queue_size=1)

arrived_pub = rospy.Publisher('/landing_zone_reached', Bool, queue_size=1)  
dist_pub = rospy.Publisher('/dist_to_target', Float32, queue_size=1)

status_take_off_pub = rospy.Publisher('/take_off_status', Int8, queue_size=1)  
status_landing_pub = rospy.Publisher('/landing_status', Int8, queue_size=1)  


# ---------------------------
# [7] 시나리오 메인 루프
# ---------------------------

while not rospy.is_shutdown():
    if take_off_scenario:
        # -- 이륙 시나리오 --
        rospy.loginfo("=== 이륙 시나리오 시작 ===")

        # 1. 격납고 -> 승하차장 이동
        status_take_off_pub.publish(1)
        rospy.loginfo("격납고 -> 승하차장 이동 시작")

        move(waypoint_1A, "waypoint_1A")
        move(point_array[2], "승하차장")

        # 2. 승객 탑승 및 승인 신호 대기
        status_take_off_pub.publish(2)
        # 승객 탑승 완료까지 대기
        rospy.loginfo("승객 탑승 중...")
        waitStarting()

        # 3. 승하차장 -> 이착륙장 이동
        status_take_off_pub.publish(3)
        move(waypoint_1B, "waypoint_1B")
        move(point_array[4], "이착륙장")

        #4. 드론 이륙 대기
        status_take_off_pub.publish(4)
        rospy.loginfo("드론 이륙 대기")
        while not rospy.is_shutdown():
            if height >= 50.0:
                rospy.loginfo("드론 고도 %.2fm, 이륙 완료. 격납고 복귀 시작" % height)
                break
            rospy.sleep(0.5)

        # 5. 이착륙장 -> 격납고 복귀
        status_take_off_pub.publish(5)
        move(waypoint_1C, "waypoint_1C")
        #move(waypoint_1D, "waypoint_1D")
        move(point_array[1], "격납고")

        # 6. 시나리오 종료 및 다음 명령 대기
        status_take_off_pub.publish(6)
        take_off_scenario = False
        rospy.loginfo("이륙 시나리오 종료, 다음 대기")

    elif landing_scenario:
        # -- 착륙 시나리오 --
        rospy.loginfo("=== 착륙 시나리오 시작 ===")

        # 1. 격납고 -> 이착륙장 이동
        status_landing_pub.publish(1)
        rospy.loginfo("격납고 -> 이착륙장 이동 시작")
        move(waypoint_1A, "waypoint_2A")
        #move(waypoint_2B, "waypoint_2B")
        pubNavGoal(point_array[3])
        while not rospy.is_shutdown():
            if is_reached == 1:
                is_reached = 0
                rospy.loginfo("이착륙장 도착")
                arrived_pub.publish(True)
                rospy.loginfo("도착 Topic 발행 완료")
                break
            rospy.sleep(0.1)
           
        # 2. 드론 추종
        status_landing_pub.publish(2)
        rospy.loginfo("드론 추종 시작")
        drone_landing()
        rospy.loginfo("드론 착륙 완료")

        #3. 이착륙장 -> 승하차장 이동
        status_landing_pub.publish(3)
        move(waypoint_2C, "waypoint_2C")
        move(waypoint_2D, "waypoint_2D")
        move(point_array[2], "승하차장")
 
        # 승객 하차 완료까지 대기
        rospy.loginfo("승객 하차 중...")
        waitStarting()

        # 4. 승하차장에서 배터리 상태 확인 후 격납고 또는 충전소 이동
        rospy.loginfo("드론 배터리 잔량 확인: %.2f%%", battery)
        if battery > 50:
            status_landing_pub.publish(4)
            rospy.loginfo("배터리 충분, 격납고로 이동")
            move(point_array[1], "격납고")  # 승하차장 -> 격납고

        else:
            status_landing_pub.publish(5)
            rospy.loginfo("배터리 부족, 충전소로 이동")
            move(point_array[4], "충전소")  # 승하차장 -> 충전소

            # 배터리 충전 완료까지 대기
            rospy.loginfo("배터리 충전 중...")
            charge_start_time = rospy.Time.now()
            charge_duration = rospy.Duration(5)  # 10초 충전 대기
            while not rospy.is_shutdown():
                if rospy.Time.now() - charge_start_time >= charge_duration:
                    status_landing_pub.publish(6)
                    rospy.loginfo("충전 완료!")
                    break
                rospy.sleep(1.0)

            # 충전 완료 후 격납고로 복귀
            rospy.loginfo("격납고로 복귀 시작")
            move(point_array[1], "격납고")

        # 5. 시나리오 종료 및 다음 명령 대기
        status_landing_pub.publish()
        landing_scenario = False
        rospy.loginfo("착륙 시나리오 종료, 다음 대기")

    else:
        rate.sleep()