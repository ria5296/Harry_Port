import can
import time
import rospy
from std_msgs.msg import Bool, Int32, Float32

CAN_INTERFACE = 'slcan0'                     
DEVICE_CAN_ID = 0x101                        
PID_REQ_PID_DATA = 0x04                      
PID_RPM = 0x8A                               
PID_CURRENT = 0x8B                           

DEVICE_NAME = 'MD200T'                       

REQUEST_INTERVAL_SEC = 0.5                  

try:
    bus = can.interface.Bus(channel=CAN_INTERFACE, bustype='socketcan')
    print(f"[INFO] CAN 인터페이스 연결됨: {CAN_INTERFACE}")
except Exception as e:
    print(f"[ERROR] CAN 인터페이스 연결 실패: {e}")
    exit(1)

def send_pid_request(pid):
    data = [PID_REQ_PID_DATA, pid] + [0x00] * 6
    msg = can.Message(arbitration_id=DEVICE_CAN_ID, data=data, is_extended_id=False)
    try:
        bus.send(msg)
    except can.CanError as e:
        print(f"[ERROR] CAN 전송 실패: {e}")

def wait_for_response(expected_pid, timeout=1.0):
    start_time = time.time()
    while time.time() - start_time < timeout:
        msg = bus.recv(timeout)
        if msg is None or len(msg.data) < 3:
            continue
        if msg.arbitration_id == DEVICE_CAN_ID and msg.data[0] == expected_pid:
            return msg.data
    return None

print("[INFO] 상태 요청 및 MQTT 전송 시작")

pub_can = rospy.Publisher('/robot/status', Bool, queue_size=1)
pub_rpm = rospy.Publisher('/md200t/rpm', Int32, queue_size=1)
pub_current = rospy.Publisher('/md200t/current', Float32, queue_size=1)

try:
    while True:
        pub_can.publish(True)
        
        send_pid_request(PID_RPM)
        rpm_data = wait_for_response(PID_RPM)
        rpm = (rpm_data[2] << 8 | rpm_data[1]) if rpm_data else -1

        send_pid_request(PID_CURRENT)
        current_data = wait_for_response(PID_CURRENT)
        current = ((current_data[2] << 8 | current_data[1]) / 10.0) if current_data else -1.0

        pub_rpm.publish(rpm)
        pub_current.publish(current)
        
        print(f"[MD200T] RPM: {rpm} rpm | Current: {current:.1f} A")

        time.sleep(REQUEST_INTERVAL_SEC)

except KeyboardInterrupt:
    pub_can.publish(False)
    print("\n[INFO] 종료됨 (CTRL+C)")
