#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import json
import base64
import threading
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Int8, Float32, Bool, Int32, UInt8
from cv_bridge import CvBridge
from autobahn.twisted.websocket import (
    WebSocketClientProtocol,
    WebSocketClientFactory,
    connectWS,
)
from twisted.internet import reactor
from twisted.internet.error import ReactorNotRunning

WEBSOCKET_URI = 'ws://170.20.10.3:3000'  

bridge = CvBridge()

take_off = None
landing = None
factory = None
take_off_scenario = False
landing_scenario = False
start_check = False
emergency_stop = False
dist = None
reached = None
robot_status = None
rpm = None
current = None
state = None
kill = None
rpm_left = None
rpm_right = None
current_left = None
current_right = None
state_left = None
state_right = None

def take_off_callback(msg):
    global take_off
    take_off = msg
    if factory.client:
        factory.client.send_msg('take_off_status', take_off, 'Int')
    else:
        rospy.logwarn("WebSocket 클라이언트 없음, 이륙 정보 전송 안함")

def landing_callback(msg):
    global landing
    landing = msg
    if factory.client:
        factory.client.send_msg('landing_status', landing, 'Int')
    else:
        rospy.logwarn("WebSocket 클라이언트 없음, 착륙 정보 전송 안함")

def dist_callback(msg):
    global dist
    dist = msg
    if factory.client:
        factory.client.send_msg('dist_to_target', dist, 'Float')

def reached_callback(msg):
    global reached
    reached = msg
    if factory.client:
        factory.client.send_msg('reached', reached, 'Bool')

def kill_callback(msg):
    global kill
    kill = msg
    if factory.client:
        factory.client.send_msg('kill', kill, 'Bool')
        
def rpm_left_callback(msg):
    global rpm_left
    rpm_left = msg
    if factory.client:
        factory.client.send_msg('md/rpm_left', rpm_left, 'Float')

def rpm_right_callback(msg):
    global rpm_right
    rpm_right = msg
    if factory.client:
        factory.client.send_msg('md/rpm_right', rpm_right, 'Float')

def current_left_callback(msg):
    global current_left
    current_left = msg
    if factory.client:
        factory.client.send_msg('md/current_left', current_left, 'Float')

def current_right_callback(msg):
    global current_right
    current_right = msg
    if factory.client:
        factory.client.send_msg('md/current_right', current_right, 'Float')

def state_left_callback(msg):
    global state_left
    state_left = msg
    if factory.client:
        factory.client.send_msg('md/state_left', state_left, 'UInt')

def state_right_callback(msg):
    global state_right
    state_right = msg
    if factory.client:
        factory.client.send_msg('md/state_right', state_right, 'UInt')
        
class RosWebSocketClient(WebSocketClientProtocol):
    def onConnect(self, response):
        rospy.loginfo("WebSocket 서버에 연결됨: {}".format(response.peer))

    def onOpen(self):
        rospy.loginfo("WebSocket 연결 완료")
        self.factory.client = self

    def onClose(self, wasClean, code, reason):
        rospy.logwarn("WebSocket 연결 종료됨: {} (code: {}, clean: {})".format(reason, code, wasClean))
        self.factory.client = None
        reconnect(self.factory)

    def _send_json_threadsafe(self, obj):
        try:
            payload = json.dumps(obj, ensure_ascii=False).encode('utf-8')
        except Exception as e:
            rospy.logerr("JSON 직렬화 실패: %s", e)
            return
        reactor.callFromThread(self.sendMessage, payload)

    def send_msg(self, topic, payload, msg_type):

        if not self.factory.client:
            rospy.logwarn("WebSocket 클라이언트 없음, 전송 안함 (topic: %s)", topic)
            return
        try:
            # 타입별 안전 캐스팅
            if msg_type in ('Int', 'Int8', 'Int16', 'Int32', 'UInt', 'UInt8', 'UInt16', 'UInt32'):
                value = int(payload.data)
            elif msg_type in ('Float', 'Float32', 'Float64'):
                value = float(payload.data)
            elif msg_type in ('Bool',):
                value = bool(payload.data)
            elif msg_type in ('String',):
                value = str(payload.data)
            else:
                value = payload.data  

            msg = {'topic': topic, 'type': msg_type, 'data': value}
            self._send_json_threadsafe(msg)
        except Exception as e:
            rospy.logerr("WS 전송 오류(topic: %s): %s", topic, e)

    def send_image(self, msg, topic, cam_name=None):
        try:
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            ret, jpeg = cv2.imencode('.jpg', cv_image, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if not ret:
                rospy.logwarn("JPEG 인코딩 실패 (cam=%s)", cam_name)
                return

            b64_data = base64.b64encode(jpeg.tobytes()).decode('utf-8')
            data_dict = {'topic': topic, 'type': 'image', 'data': b64_data}
            self._send_json_threadsafe(data_dict)
        except Exception as e:
            rospy.logerr("이미지 전송 오류(cam=%s): %s", cam_name, e)

    def onMessage(self, payload, isBinary):
        try:
            data = json.loads(payload.decode('utf-8'))

            if data.get('topic') == 'sensor' and 'bat' in data and 'height' in data:
                bat = float(data['bat'])
                height = float(data['height'])
                self.factory.battery_pub.publish(bat)
                self.factory.height_pub.publish(height)

            if data.get('topic') == 'take_off_scenario':
                self.factory.take_off_scenario_pub.publish(bool(data.get('data', False)))

            if data.get('topic') == 'landing_scenario':
                self.factory.landing_scenario_pub.publish(bool(data.get('data', False)))

            if data.get('topic') == 'start_check':
                self.factory.start_check_pub.publish(bool(data.get('data', False)))

            if data.get('topic') == 'emergency_stop':
                self.factory.emergency_stop_pub.publish(bool(data.get('data', False)))

        except Exception as e:
            rospy.logerr("메시지 처리 오류: {}".format(str(e)))

class RosWebSocketClientFactory(WebSocketClientFactory):
    protocol = RosWebSocketClient
    def __init__(self, *args, **kwargs):
        super(RosWebSocketClientFactory, self).__init__(*args, **kwargs)
        self.client = None

        self.battery_pub = rospy.Publisher('/sensor/battery', Float32, queue_size=1)
        self.height_pub = rospy.Publisher('/sensor/height', Float32, queue_size=1)
        self.take_off_scenario_pub = rospy.Publisher('/take_off_scenario', Bool, queue_size=1)
        self.landing_scenario_pub = rospy.Publisher('/landing_scenario', Bool, queue_size=1)
        self.start_check_pub = rospy.Publisher('/start_check', Bool, queue_size=1)
        self.emergency_stop_pub = rospy.Publisher('/emergency_stop', Bool, queue_size=1)

def reconnect(factory):
    try:
        rospy.loginfo("재연결 시도 중...")
        connectWS(factory)  
    except Exception as e:
        rospy.logerr("재연결 실패: {}".format(str(e)))

def ros_main():
    global factory
    rospy.init_node('ros_websocket_client', anonymous=True)
    rospy.loginfo("ROS 노드 시작됨")

    factory = RosWebSocketClientFactory(WEBSOCKET_URI)
    connectWS(factory)
    rospy.loginfo("WebSocket 최초 연결 시도... 주소: {}".format(WEBSOCKET_URI))

    rospy.Subscriber(
        '/sky_camera/image_raw', Image,
        lambda msg: factory.client.send_image(msg, topic='sky_camera/image_raw', cam_name='sky')
        if factory.client else rospy.logwarn("WebSocket 연결 없음. (sky) 이미지 전송 안함.")
    )
    rospy.Subscriber(
        '/front_camera/image_raw', Image,
        lambda msg: factory.client.send_image(msg, topic='front_camera/image_raw', cam_name='front')
        if factory.client else rospy.logwarn("WebSocket 연결 없음. (front) 이미지 전송 안함.")
    )

    rospy.Subscriber('/take_off_status', Int8, take_off_callback, queue_size=1)
    rospy.Subscriber('/landing_status', Int8, landing_callback, queue_size=1)
    rospy.Subscriber('/dist_to_target', Float32, dist_callback, queue_size=1)
    rospy.Subscriber('/landing_zone_reached', Bool, reached_callback, queue_size=1)
    rospy.Subscriber('/kill', Bool, kill_callback, queue_size=1)

    rospy.Subscriber('/md/rpm_left', Float32, rpm_left_callback, queue_size=10)
    rospy.Subscriber('/md/rpm_right', Float32, rpm_right_callback, queue_size=10)
    rospy.Subscriber('/md/current_left', Float32, current_left_callback, queue_size=10)
    rospy.Subscriber('/md/current_right', Float32, current_right_callback, queue_size=10)
    rospy.Subscriber('/md/state_left', UInt8, state_left_callback, queue_size=10)
    rospy.Subscriber('/md/state_right', UInt8, state_right_callback, queue_size=10)

    thread = threading.Thread(target=reactor.run, args=(False,))
    thread.daemon = True
    thread.start()
    rospy.loginfo("Twisted reactor 백그라운드 실행 시작")

    rospy.spin()
    rospy.loginfo("ROS spin 종료")

    try:
        reactor.stop()
        rospy.loginfo("Twisted reactor 종료 완료")
    except ReactorNotRunning:
        rospy.logwarn("Twisted reactor 이미 종료됨")

if __name__ == '__main__':
    ros_main()
