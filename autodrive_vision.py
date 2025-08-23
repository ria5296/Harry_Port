#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, time, subprocess
import rospy, cv2
import numpy as np
from std_msgs.msg import Bool, Int16MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from jetson_inference import detectNet
from jetson_utils import cudaFromNumpy


HSV_S_NORM_MAX = 0.45
HSV_V_NORM_MIN = 0.40  

def is_light_like_center(bgr, box):

    x1, y1, x2, y2 = map(int, box)
    if x2 <= x1 or y2 <= y1:
        return False

    cx = (x1 + x2) // 2
    cy = (y1 + y2) // 2

    h, w = bgr.shape[:2]
    cx = max(0, min(w - 1, cx))
    cy = max(0, min(h - 1, cy))
    b, g, r = bgr[cy, cx].tolist()
    hsv = cv2.cvtColor(np.uint8([[[b, g, r]]]), cv2.COLOR_BGR2HSV)[0, 0]
    S = hsv[1] / 255.0
    V = hsv[2] / 255.0
    return (S <= HSV_S_NORM_MAX) and (V >= HSV_V_NORM_MIN)

net = detectNet(argv=[
    '--model=/home/hp/catkin_ws/src/autodrive_vision/src/ssd-mobilenet.onnx',
    '--labels=/home/hp/catkin_ws/src/autodrive_vision/src/labels.txt',
    '--input-blob=input_0', '--output-cvg=scores', '--output-bbox=boxes',
    '--threshold=0.3', '--device=0', '--precision=fp16'
])

bridge = CvBridge()
cap = None
active_cam = None              
want_cam = "front"
fail_read_count = 0

front_img_pub = sky_img_pub = kill_pub = error_pub = None
USB_DEV_BY_ID = "/dev/v4l/by-id/usb-Image_Processor_USB_2.0_PC_Cam-video-index0"

# ---------- helpers ----------
def restart_nvargus_if_needed():
    try:
        status = subprocess.check_output(
            ["systemctl", "is-active", "nvargus-daemon"],
            stderr=subprocess.STDOUT
        ).decode().strip()
        if status != "active":
            rospy.logwarn("[dual_cam] nvargus-daemon not active → restart")
            subprocess.check_call(["sudo", "systemctl", "restart", "nvargus-daemon"])
            time.sleep(0.7)
    except Exception as e:
        rospy.logwarn("[dual_cam] nvargus check failed: %s", e)

def _resolve_usb_dev():
    return USB_DEV_BY_ID if os.path.exists(USB_DEV_BY_ID) else "/dev/video1"

def _is_free(dev_path):
    real = os.path.realpath(dev_path)
    try:
        with open(os.devnull, 'wb') as dn:
            rc = subprocess.call(["fuser", real], stdout=dn, stderr=dn)
        return rc != 0
    except Exception:
        return True

def _wait_until_free(dev_path, timeout=2.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if _is_free(dev_path):
            return True
        time.sleep(0.1)
    return _is_free(dev_path)

def close_camera():
    global cap, active_cam
    if cap is not None:
        try: cap.release()
        except Exception: pass
    cap = None
    active_cam = None

def _open_with_gst(gst, name):

    global cap, active_cam
    rospy.loginfo("[dual_cam] open %s: %s", name, gst)
    c = None
    try:
        c = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)
    except Exception as e:
        rospy.logerr("[dual_cam] VideoCapture exception: %s", e)
        return False

    if c and c.isOpened():
        try: c.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception: pass
        cap = c
        active_cam = name
        rospy.loginfo("[dual_cam] %s opened (fast-return)", name)
        return True

    time.sleep(0.2)
    try:
        if c: c.release()
    except Exception: pass
    try:
        c = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)
    except Exception:
        c = None
    if c and c.isOpened():
        try: c.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception: pass
        cap = c
        active_cam = name
        rospy.loginfo("[dual_cam] %s opened (retry)", name)
        return True

    rospy.logerr("[dual_cam] open %s failed", name)
    return False

def open_front_camera():
    dev = _resolve_usb_dev()
    _wait_until_free(dev, timeout=2.0)
    gst = (
        "v4l2src device={} io-mode=2 ! "
        "video/x-raw, format=YUY2, width=320, height=240, framerate=30/1 ! "
        "videoconvert ! video/x-raw, format=BGR ! "
        "appsink drop=true max-buffers=1 sync=false"
    ).format(dev)
    return _open_with_gst(gst, "front")

def open_sky_camera():
    restart_nvargus_if_needed()
    gst = (
        "nvarguscamerasrc sensor-id=0 "
        "exposuretimerange=\"8000000 8000000\" gainrange=\"1 4\" wbmode=0 ! "
        "video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1 ! "
        "nvvidconv flip-method=2 ! video/x-raw,format=BGRx ! "
        "videoconvert ! video/x-raw,format=BGR ! "
        "appsink drop=true max-buffers=1 sync=false"
    )
    ok = _open_with_gst(gst, "sky")
    if ok and cap:
        try:
            cap.grab(); cap.read()
            cap.grab(); cap.read()
        except Exception:
            pass
    return ok

def switch_camera(target):
    if target == active_cam:
        return
    rospy.loginfo("[dual_cam] switch: %s -> %s", active_cam, target)

    close_camera()

    if target == "front":
        time.sleep(0.8)
        ok = open_front_camera()
    else:
        restart_nvargus_if_needed()
        time.sleep(1.0)
        ok = open_sky_camera()

    if not ok:
        rospy.logerr("[dual_cam] switch failed (target=%s)", target)

def landing_cb(msg):
    global want_cam
    want_cam = "sky" if msg.data else "front"
    rospy.loginfo("[dual_cam] /landing=%s → want_cam=%s", msg.data, want_cam)

def main():
    global front_img_pub, sky_img_pub, kill_pub, error_pub, fail_read_count
    rospy.init_node("dual_camera_node")

    rospy.Subscriber('/landing_zone_reached', Bool, landing_cb)

    front_img_pub = rospy.Publisher('/front_camera/image_raw', Image, queue_size=1)
    sky_img_pub   = rospy.Publisher('/sky_camera/image_raw',  Image, queue_size=1)
    kill_pub      = rospy.Publisher('/kill',                  Bool,  queue_size=1)
    error_pub     = rospy.Publisher('/drone_error',           Int16MultiArray, queue_size=1)
    drone_center_pub = rospy.Publisher('drone_center', Int16MultiArray, queue_size=1)
    switch_camera("front")  # 초기 전방(이제 USB)

    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        if want_cam != active_cam:
            switch_camera(want_cam)
            rate.sleep(); continue

        if cap is None or not cap.isOpened():
            rospy.logwarn("[dual_cam] cap invalid, reopen %s", want_cam)
            switch_camera(want_cam)
            rate.sleep(); continue

        cap.grab(); cap.grab()
        ret, frame = cap.read()
        if not ret:
            fail_read_count += 1
            if fail_read_count >= 3:
                rospy.logwarn("[dual_cam] read() failed x%d → reopen %s", fail_read_count, want_cam)
                switch_camera(want_cam)
                fail_read_count = 0
            rate.sleep(); continue
        else:
            fail_read_count = 0

        cuda_img = cudaFromNumpy(frame)
        detections = net.Detect(cuda_img)

        if active_cam == "front":
            human_detected = any(
                net.GetClassDesc(det.ClassID) == "human" and det.Confidence > 0.7
                for det in detections
            )
            kill_pub.publish(Bool(data=human_detected))
            try:
                front_img_pub.publish(bridge.cv2_to_imgmsg(frame, encoding="bgr8"))
            except Exception:
                pass
        else:
            h, w, _ = frame.shape
            for det in detections:
                if net.GetClassDesc(det.ClassID) == "drone" and det.Confidence > 0.3:
                    box = (det.Left, det.Top, det.Right, det.Bottom)

                    if is_light_like_center(frame, box):
                        continue

                    cx = int((det.Left + det.Right) / 2.0)
                    cy = int((det.Top + det.Bottom) / 2.0)
                    drone_center_pub.publish(Int16MultiArray(data=[cx, cy]))
                    break
            try:
                sky_img_pub.publish(bridge.cv2_to_imgmsg(frame, encoding="bgr8"))
            except Exception:
                pass

        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    finally:
        close_camera()