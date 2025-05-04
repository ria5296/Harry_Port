from flask import Flask, render_template, Response
from flask_socketio import SocketIO
import time

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins='*', async_mode='threading')

@app.route('/')
def index():
    return render_template('index.html')

# 테스트용 SLAM 맵 스트리밍 (고정 이미지)
def gen_dummy_map():
    with open('static/test_map.jpg', 'rb') as f:
        frame = f.read()
    while True:
        time.sleep(0.1)
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

# 테스트용 카메라 스트리밍 (고정 이미지)
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

# WebSocket 이벤트
@socketio.on('drone_info')
def handle_drone_info(data):
    print("드론 정보 수신:", data)
    socketio.emit('update_drone_info', data, broadcast=True)

@socketio.on('system_info')
def handle_system_info(data):
    print("시스템 정보 수신:", data)
    socketio.emit('update_system_info', data, broadcast=True)

if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=5000)
