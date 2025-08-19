from flask import Flask, render_template, Response, jsonify
from flask_socketio import SocketIO
from threading import Thread
import time
from serial_reader import read_serial
import re
import socketio  # 젯슨에서 수신하기 위한 클라이언트용
import rospy
import actionlib
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion, Pose
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int8, Int16, Image, Bool, Float32

app = Flask(__name__)
socketio_server = SocketIO(app, cors_allowed_origins='*', async_mode='threading')

# 젯슨 클라이언트 연결 설정
JETSON_IP = '192.168.0.101'  # LTE 라우터 내 할당된 젯슨 IP로 수정
JETSON_PORT = 6000
sio_jetson = socketio.Client()


rospy.Subscriber("/sky_camera/image_raw", Image, getImage , queue_size=10)

latest_sensor_data = {
    'bat': None,
    'roll': None,
    'pitch': None,
    'height': None
}

def parse_sensor_line(line):
    pattern = r"BAT\s*:\s*(\d+),\s*Roll\s*:\s*(-?\d+),\s*Pitch\s*:\s*(-?\d+),\s*Height\s*:\s*(\d+)"
    match = re.match(pattern, line)
    if match:
        return {
            'bat': int(match.group(1)),
            'roll': int(match.group(2)),
            'pitch': int(match.group(3)),
            'height': int(match.group(4))
        }
    return None

def serial_thread():
    global latest_sensor_data
    for data in read_serial():
        parsed = parse_sensor_line(data)
        if parsed:
            latest_sensor_data = parsed
            socketio_server.emit('sensor_update', {'sensor': parsed}, namespace='/')
            # 젯슨에도 전송
            if sio_jetson.connected:
                sio_jetson.emit('sensor_update', parsed)

def connect_to_jetson():
    try:
        sio_jetson.connect(f'http://{JETSON_IP}:{JETSON_PORT}')
        print(f"젯슨에 연결됨: {JETSON_IP}:{JETSON_PORT}")
    except Exception as e:
        print(f"젯슨 연결 실패: {e}")

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/map_stream')
def map_stream():
    with open('static/test_map.jpg', 'rb') as f:
        frame = f.read()
    def stream():
        while True:
            time.sleep(0.1)
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
    return Response(stream(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/camera_stream')
def camera_stream():
    with open('static/test_camera.jpg', 'rb') as f:
        frame = f.read()
    def stream():
        while True:
            time.sleep(0.1)
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
    return Response(stream(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/sensor', methods=['GET'])
def get_sensor_data():
    return jsonify({'sensor': latest_sensor_data})

# WebSocket 이벤트 처리
@socketio_server.on('drone_info')
def handle_drone_info(data):
    print("드론 정보 수신:", data)
    socketio_server.emit('update_drone_info', data, namespace='/', broadcast=True)
    if sio_jetson.connected:
        sio_jetson.emit('update_drone_info', data)

@socketio_server.on('system_info')
def handle_system_info(data):
    print("시스템 정보 수신:", data)
    socketio_server.emit('update_system_info', data, namespace='/', broadcast=True)
    if sio_jetson.connected:
        sio_jetson.emit('update_system_info', data)

if __name__ == '__main__':
    Thread(target=serial_thread, daemon=True).start()
    Thread(target=connect_to_jetson, daemon=True).start()
    socketio_server.run(app, host='0.0.0.0', port=5000)
