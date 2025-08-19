#!/usr/bin/env python
# -*- coding: utf-8 -*-

import can
import json
import time
import signal
import sys

import rospy
from std_msgs.msg import String, Int32, Float32, UInt8

TOPIC_STATUS_JSON = "/robot/status"
TOPIC_RPM         = "/md200t/rpm"
TOPIC_CURRENT     = "/md200t/current"
TOPIC_STATE       = "/md200t/status"

# ROS 파라미터 기본값
DEFAULT_CAN_INTERFACE = "slcan0"   # USB-to-CAN 장치명
REQUEST_PID           = 0xC1       # MD200T 상태 요청 PID
CAN_ID_REQ            = 0x181      # 요청용 CAN ID (11bit)
CAN_ID_RES            = 0x181      # 응답용 CAN ID (11bit)
CAN_TIMEOUT_S         = 1.0        # 응답 대기 시간 (초)
DEFAULT_POLL_PERIOD_S = 0.5        # 폴링 주기 (초)
# ===============================

bus = None
running = True

def handle_shutdown(signum, frame):
    global running
    running = False

def init_can(interface_name):
    try:
        b = can.interface.Bus(channel=interface_name, bustype="socketcan")
        rospy.loginfo("[CAN] 초기화 성공: %s", interface_name)
        return b
    except OSError as e:
        rospy.logerr("[CAN] 초기화 실패(%s): %s", interface_name, e)
        sys.exit(1)

def send_can_request():
    """MD200T 상태 요청 프레임 전송"""
    data = [REQUEST_PID, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    msg = can.Message(arbitration_id=CAN_ID_REQ, data=data, is_extended_id=False)
    try:
        bus.send(msg)
        # rospy.logdebug("[CAN] 상태 요청 전송: %s", msg)
    except can.CanError as e:
        rospy.logwarn("[CAN] 전송 실패: %s", e)

def receive_can_response(timeout=CAN_TIMEOUT_S):
    """MD200T 응답 대기 후 데이터 파싱"""
    start_time = time.time()
    while time.time() - start_time < timeout and not rospy.is_shutdown():
        msg = bus.recv(timeout=0.2)
        if msg and msg.arbitration_id == CAN_ID_RES and len(msg.data) >= 5:
            if msg.data[0] == REQUEST_PID:
                rpm = (msg.data[2] << 8) | msg.data[1]
                current = ((msg.data[4] << 8) | msg.data[3]) / 10.0
                status = msg.data[5] if len(msg.data) > 5 else 0
                return {
                    "rpm": int(rpm),
                    "current": float(current),
                    "status": int(status)
                }
    return None

def main():
    global bus

    rospy.init_node("md200t_can_status", anonymous=False)

    can_interface = rospy.get_param("~can_interface", DEFAULT_CAN_INTERFACE)
    poll_period   = float(rospy.get_param("~poll_period", DEFAULT_POLL_PERIOD_S))

    bus = init_can(can_interface)

    pub_json   = rospy.Publisher(TOPIC_STATUS_JSON, String, queue_size=1)
    pub_rpm    = rospy.Publisher(TOPIC_RPM, Int32, queue_size=1)
    pub_curr   = rospy.Publisher(TOPIC_CURRENT, Float32, queue_size=1)
    pub_state  = rospy.Publisher(TOPIC_STATE, UInt8, queue_size=1)

    rospy.loginfo("[SYSTEM] MD200T 상태 폴링 시작 (period=%.2fs, iface=%s)", poll_period, can_interface)

    signal.signal(signal.SIGINT, handle_shutdown)
    signal.signal(signal.SIGTERM, handle_shutdown)

    rate = rospy.Rate(1.0 / poll_period if poll_period > 0 else 2.0)

    while running and not rospy.is_shutdown():
        try:
            send_can_request()
            res = receive_can_response(timeout=CAN_TIMEOUT_S)
            if res is not None:
                pub_rpm.publish(res["rpm"])
                pub_curr.publish(res["current"])
                pub_state.publish(res["status"])
                pub_json.publish(True)
            else:
                rospy.logwarn_throttle(5.0, "[CAN] 응답 없음 (timeout=%.1fs)", CAN_TIMEOUT_S)

        except Exception as e:
            rospy.logerr_throttle(5.0, "[ERROR] 폴링 루프: %s", e)

        rate.sleep()

    rospy.loginfo("[SYSTEM] 종료 중...")
    try:
        bus.shutdown()
    except Exception:
        pass

if __name__ == "__main__":
    main()
