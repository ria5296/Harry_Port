from flask import Flask, render_template, jsonify, Response
from flask_socketio import SocketIO
from threading import Thread
import time
from serial_reader import read_serial
import re
import json

app = Flask(__name__)
socketio_server = SocketIO(app, cors_allowed_origins='*', async_mode='threading')

latest_sensor_data = {
    'bat': '-',
    'roll': '-',
    'pitch': '-',
    'height': '-'
}

latest_system_info = {
    'robot_status': '-',
    'cpu': '-',
    'ram': '-',
    'net': '-'
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
            
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/sensor', methods=['GET'])
def get_sensor_data():
    return jsonify({'sensor': latest_sensor_data})

@socketio_server.on('sensor_update')
def handle_sensor_update(data):
    global latest_sensor_data
    latest_sensor_data = data
    socketio_server.emit('sensor_update', {'sensor': data}, broadcast=True)

@socketio_server.on('system_info')
def handle_system_info(data):
    global latest_system_info
    latest_system_info = data
    socketio_server.emit('system_info', data, broadcast=True)

if __name__ == '__main__':
    t = Thread(target=serial_thread)
    t.daemon = True
    t.start()
    socketio_server.run(app, host='0.0.0.0', port=5000)

