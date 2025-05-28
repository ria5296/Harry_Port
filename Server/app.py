from flask import Flask, render_template, Response, jsonify
from flask_socketio import SocketIO
from threading import Thread
import time
from serial_reader import read_serial
import re

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins='*', async_mode='threading')

latest_sensor_data = {
    'bat': None,
    'roll': None,
    'pitch': None,
    'height': None
}

def parse_sensor_line(line):
    # "BAT:xx,ROLL:xx,PITCH:xx,HEIGHT:xx" 형태를 파싱 (공백 포함 대응)
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
        print("시리얼 데이터 수신:", data)
        parsed = parse_sensor_line(data)
        if parsed:
            latest_sensor_data = parsed
            socketio.emit('sensor_update', {'sensor': parsed}, namespace='/')

@app.route('/')
def index():
    return render_template('index.html')

def gen_dummy_map():
    with open('static/test_map.jpg', 'rb') as f:
        frame = f.read()
    while True:
        time.sleep(0.1)
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

def gen_dummy_camera():
    with open('static/test_camera.jpg', 'rb') as f:
        frame = f.read()
    while True:
        time.sleep(0.1)
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/map_stream')
def map_stream():
    return Response(gen_dummy_map(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/camera_stream')
def camera_stream():
    return Response(gen_dummy_camera(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/sensor', methods=['GET'])
def get_sensor_data():
    return jsonify({'sensor': latest_sensor_data})

# WebSocket 이벤트
@socketio.on('drone_info')
def handle_drone_info(data):
    print("드론 정보 수신:", data)
    socketio.emit('update_drone_info', data, namespace='/', broadcast=True)

@socketio.on('system_info')
def handle_system_info(data):
    print("시스템 정보 수신:", data)
    socketio.emit('update_system_info', data, namespace='/', broadcast=True)

if __name__ == '__main__':
    Thread(target=serial_thread, daemon=True).start()
    socketio.run(app, host='0.0.0.0', port=5000)
