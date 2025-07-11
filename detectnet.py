#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, cv2
from jetson_inference import detectNet
from jetson_utils import cudaFromNumpy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32, Bool, Int32MultiArray
from cv_bridge import CvBridge

net = detectNet(argv=[
    '--model=/home/hp/catkin_ws/src/autodrive_vision/src/ssd-mobilenet.onnx',
    '--labels=/home/hp/catkin_ws/src/autodrive_vision/src/labels.txt',
    '--input-blob=input_0',
    '--output-cvg=scores',
    '--output-bbox=boxes'
])

kill = False
bridge = CvBridge()
detected_objects = {}  
frame_count = {}
MAX_MISSING_FRAMES = 10

def kill_callback(data):
    global kill
    kill = data.data

rospy.init_node("autodrive_detectnet")
name_publisher = rospy.Publisher('/detectnet/name', String, queue_size=1)
accuracy_publisher = rospy.Publisher('/detectnet/accuracy', Float32, queue_size=1)
video_publisher = rospy.Publisher('/video', Image, queue_size=10)
bbox_publisher = rospy.Publisher('/detectnet/bbox', Int32MultiArray, queue_size=10)
rospy.Subscriber('/kill', Bool, kill_callback, queue_size=1)
drone_crd_publisher = rospy.Publisher('/detectnet/crd', Int32MultiArray, queue_size=10)
human_detect_publisher = rospy.Publisher('/detectnet/human',Bool,queue_size=1)

cap0 = cv2.VideoCapture(
    "v4l2src device=/dev/video1 ! "
    "video/x-raw, format=YUY2, width=320, height=240, framerate=30/1 ! "
    "videoconvert ! "
    "video/x-raw, format=BGR ! appsink",
    cv2.CAP_GSTREAMER
)

cap1 = cv2.VideoCapture("nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=(int)640, height=(int)480, framerate=(fraction)30/1 ! "
                        "nvvidconv flip-method=2 ! video/x-raw, width=(int)320, height=(int)240, format=(string)BGRx ! "
                        "videoconvert ! video/x-raw, format=(string)BGR ! appsink")
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    ret0, frame0 = cap0.read()
    ret1, frame1 = cap1.read()
    if not ret0:
        continue
    if not ret1:
        continue
    video0_msg = bridge.cv2_to_imgmsg(frame0, "bgr8")
    video_publisher.publish(video0_msg)
    
    video1_msg = bridge.cv2_to_imgmsg(frame1, "bgr8")
    video_publisher.publish(video1_msg)
    
    img0_cuda = cudaFromNumpy(frame0)
    detections0 = net.Detect(img0_cuda)

    img1_cuda = cudaFromNumpy(frame1)
    detections1 = net.Detect(img1_cuda)

    bbox_list = []
    crd_list = []
    #when pedestrian detected
    human_detected=False
    for detection_human in detections1:
        if detection_human.Confidence >0.80:
            human_detected=True
            name = net.GetClassDesc(detection_human.ClassID)
            rospy.loginfo("탐지된 객체: %s (%.2f)" % (name, detection_human.Confidence))
            name_publisher.publish(name)
            accuracy_publisher.publish(detection_human.Confidence)
            break
    human_detect_publisher.publish(human_detected)
    if human_detected:
        rospy.loginfo("human_detected. End system")
                # Keep the window open and only update frames without processing other objects
        while human_detected:
            ret0, frame0 = cap0.read()
            ret1, frame1 = cap1.read()
            
            if not ret0 or not ret1:
                break  # Break if we cannot capture frames
            
            video0_msg = bridge.cv2_to_imgmsg(frame0, "bgr8")
            video_publisher.publish(video0_msg)
    
            video1_msg = bridge.cv2_to_imgmsg(frame1, "bgr8")
            video_publisher.publish(video1_msg)

            # Check again for human detection
            img0_cuda = cudaFromNumpy(frame0)
            detections0 = net.Detect(img0_cuda)

            img1_cuda = cudaFromNumpy(frame1)
            detections1 = net.Detect(img1_cuda)

            # Check for human detection
            for detection_human in detections1:
                if detection_human.Confidence > 0.80:
                    human_detected = True
                    break  # Continue detecting human if still detected
                else:
                    human_detected = False  # Human is no longer detected

            human_detect_publisher.publish(human_detected)
            
            if cv2.waitKey(1) == ord('q') or kill:
                break

            rate.sleep()


    if not human_detected:
        for detection in detections0:
            if detection.Confidence < 0.85:
                continue
            
            name = net.GetClassDesc(detection.ClassID)

            frame_width = frame0.shape[1]
            frame_height = frame0.shape[0]

            center_x = (detection.Left + detection.Right) // 2
            center_y = (detection.Top + detection.Bottom) // 2

            rospy.loginfo("탐지된 객체: %s (%.2f) FPS: %.1f" % (name, detection.Confidence, net.GetNetworkFPS()))
            name_publisher.publish(name)
            accuracy_publisher.publish(detection.Confidence)

            obj_id = len(detected_objects)
            detected_objects[obj_id] = (name, (int(detection.Left), int(detection.Top), int(detection.Right), int(detection.Bottom)))
            frame_count[obj_id] = 0    

            bbox_list.extend([int(detection.Left), int(detection.Top), int(detection.Right), int(detection.Bottom)])
            cv2.rectangle(frame0, (int(detection.Left), int(detection.Top)),
                        (int(detection.Right), int(detection.Bottom)), (42, 42, 165), 2)
            drone_diff_x = (frame_width//2) - center_x
            drone_diff_y = (frame_height//2) - center_y - 50
            crd_list.extend([int(drone_diff_x),int(drone_diff_y)])
            rospy.loginfo("drone x crd: %d, drone y crd: %d" % (drone_diff_x,drone_diff_y))
            crd_msg = Int32MultiArray(data=crd_list)
            drone_crd_publisher.publish(crd_msg)

        if bbox_list:
            bbox_msg = Int32MultiArray(data=bbox_list)
            bbox_publisher.publish(bbox_msg)

        to_remove = [obj_id for obj_id, count in frame_count.items() if count > MAX_MISSING_FRAMES]
        for obj_id in to_remove:
            detected_objects.pop(obj_id, None)
            frame_count.pop(obj_id, None)

        cv2.imshow('autodrive_detectnet', frame0)
        cv2.imshow('USB Camera',frame1)
        if cv2.waitKey(1) == ord('q') or kill == True:
            #kill_publisher.publish(True)
            break
        rate.sleep()
cv2.destroyAllWindows()
cap0.release()
cap1.release()