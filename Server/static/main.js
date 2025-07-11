const socket = io();

// 시스템 정보 수신
socket.on('system_info', data => {
  document.getElementById('robot_status').textContent = data.robot_status;
  document.getElementById('cpu').textContent = data.cpu;
  document.getElementById('ram').textContent = data.ram;
  document.getElementById('net').textContent = data.net;
});

// 센서 정보 수신
socket.on('sensor_update', data => {
  const sensor = data.sensor;
  if (sensor) {
    document.getElementById('bat').textContent = sensor.bat;
    document.getElementById('roll').textContent = sensor.roll;
    document.getElementById('pitch').textContent = sensor.pitch;
    document.getElementById('height').textContent = sensor.height;
  }
});
