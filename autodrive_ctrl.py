#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion, Pose
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int8, Bool, Int16, Image, Float32

import threading
import socketio

# ---------------------------
# [1] 전역 변수 선언
# ---------------------------
x_meter, y_meter, z_orient, w_orient = (0, 0, 0, 0)
destination = 0
is_reached = 0
kill_switch = False
move_client = None
current_goal = None
landing = False
drone_height = 0.0  # ← Flask에서 수신한 드론 고도
drone_battery = 0.0
current_pose = (0, 0)
drone_error = [0, 0]
diff_pose = (0, 0)
is_landing = False

# ---------------------------
# [2] 좌표 정의
# ---------------------------
point_array = [
    (0, 0, 0, 0),
    (1.904, 7.198, -0.229, 0.973),
    (4.958, 7.673, -0.847, 0.532),
    (3.784, 7.834, -0.275, 0.961)
]

waypoint_in  = (4.522, 6.578, 0.972, 0.233)
waypoint_out = (4.522, 6.578, -0.855, 0.519)

location_names = {
    1: '격납고',
    2: '승하차장',
    3: '이착륙장'
}

# ---------------------------
# [3] ROS 콜백 함수
# ---------------------------
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

def getLandingStatus(data):
    global landing
    landing = data.data

def getDroneerror(data):
    global drone_error
    drone_error = data.data
        
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

# ---------------------------
# [4] 목표 전송 함수
# ---------------------------
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
            rospy.logerr("Goal 실패 or 서버 미응답")
            rospy.signal_shutdown("goal 실패")
    else:
        rospy.logwarn("사람 감지 상태로 goal 전송 중지됨")


def drone_landing():
    global current_pose, error_pose, diff_pose, is_landing
    is_landing = True

    while is_landing:
        current_pose = (x_meter, y_meter)
        error_pose = (drone_error[0], drone_error[1])
        diff_pose = (drone_error[0] + 100, drone_error[1] + 100)       # 좌표 수정
        pubNavGoal(diff_pose)
        
        if drone_height < 10:
            is_landing = False
            
            
            
            break
    
# ---------------------------
# [5] WebSocket 처리
# ---------------------------
sio = socketio.Client()

@sio.on('update_drone_info')
def on_update_drone_info(data):
    global destination
    try:
        if 'destination' in data:
            new_dest = int(data['destination'])
            if new_dest in [1, 2, 3]:
                rospy.loginfo(f"[웹소켓] 목적지 수신: {new_dest}")
                destination = new_dest
    except Exception as e:
        rospy.logerr(f"[웹소켓] drone_info 처리 오류: {e}")

@sio.on('sensor_update')
def on_sensor_update(data):
    global drone_height
    global drone_battery
    try:
        if 'height' in data:
            drone_height = float(data['height'])
        elif 'bat' in data:
            drone_battery = float(data['bat'])
    except Exception as e:
        rospy.logerr(f"[웹소켓] sensor_update 처리 오류: {e}")

def socket_thread():
    server_url = "http://192.168.0.100:5000"
    try:
        rospy.loginfo(f"[웹소켓] 서버 연결 시도: {server_url}")
        sio.connect(server_url)
        rospy.loginfo("[웹소켓] 서버 연결 성공")
    except Exception as e:
        rospy.logerr(f"[웹소켓] 서버 연결 실패: {e}")

# ---------------------------
# [6] 메인 루프
# ---------------------------
def main():
    global destination, is_reached, drone_height

    rospy.init_node("autodrive_ctrl_node")
    r = rospy.Rate(10)

    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, getAmclPose, queue_size=1)
    rospy.Subscriber("/destination", Int8, getDestination, queue_size=1)
    rospy.Subscriber("/move_base/result", MoveBaseActionResult, getNavStatus, queue_size=1)
    rospy.Subscriber("/kill", Bool, kill_callback, queue_size=1)
    rospy.Subscriber("/landing", Bool, getLandingStatus, queue_size=1)
    rospy.Subscriber("/drone_error", Int16, getDroneerror , queue_size=1)

    arrived_pub = rospy.Publisher('/landing', Bool, queue_size=1)

    threading.Thread(target=socket_thread, daemon=True).start()

    while not rospy.is_shutdown():

        if landing:
            rospy.loginfo("이착륙 시작")
            if drone_height < 10:
                rospy.loginfo("기체 이륙 시작")
                while True:     #고도값 계속 동기화되는지 확인 필요
                    if drone_height > 50:
                        landing=False
                        break
                    
            else:
                rospy.loginfo("기체 착륙 시작 : 기체 추종 모드")
                drone_landing()
                

        if destination != 0 and destination in location_names:
            print("\n" + "="*24 + " UAM 운반 임무 개시 " + "="*24)
            rospy.loginfo(f"도착지 : {location_names[destination]}")

            if destination == 3:
                rospy.loginfo("이착륙장 이동: Waypoint_in 경유")
                pubNavGoal(waypoint_in)
            else:
                rospy.loginfo("격납고/승하차장 이동: Waypoint_out 경유")
                pubNavGoal(waypoint_out)

            while not rospy.is_shutdown() and is_reached != 1:
                rospy.sleep(0.1)
            is_reached = 0

            rospy.loginfo(f"{location_names[destination]}으로 이동 중...")
            pubNavGoal(point_array[destination])

            while not rospy.is_shutdown() and is_reached != 1:
                rospy.sleep(0.1)

            rospy.loginfo(f"{location_names[destination]} 도착")
            is_reached = 0

            if destination == 3:
                arrived_pub.publish(True)
                rospy.sleep(0.5)

            destination = 0
            rospy.loginfo("새 목적지를 기다립니다")

        r.sleep()

if __name__ == '__main__':
    main()
