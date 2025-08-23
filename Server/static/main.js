let ws;
const JETSON_IP = '172.20.10.3';
const WS_PORT   = '3000';

function logInfo(msg) {
  const ul = document.getElementById('info_log');
  if (!ul) return;
  const li = document.createElement('li');
  li.textContent = `[${new Date().toLocaleTimeString()}] ${msg}`;
  ul.prepend(li);                       
  while (ul.children.length > 80) ul.removeChild(ul.lastChild);
}

let lastTakeOffStatus = null;
let lastLandingStatus = null;
let landing_height = false;

function connectWebSocket() {
  ws = new WebSocket(`ws://${JETSON_IP}:${WS_PORT}`);

  ws.onopen = () => {
    logInfo('✅ WebSocket 연결됨');
  };

  ws.onclose = () => {
    logInfo('⚠️ WebSocket 끊김. 5초 후 재연결 시도');
    setTimeout(connectWebSocket, 5000);
  };

  ws.onerror = (error) => {
    logInfo('🚨 WebSocket 오류 발생');
  };

  ws.onmessage = (event) => {
    try {
      const payload = JSON.parse(event.data);
      const items = Array.isArray(payload) ? payload : [payload];
      for (const data of items) {
        handleMessage(data);
      }
    } catch (e) {
      logInfo('파싱오류');
    }
  };
}

function handleMessage(data) {
  if (data.topic === 'sky_camera/image_raw' && data.type === 'image' && data.data) {
    const imgElement = document.getElementById('camera');
    if (imgElement) {
      imgElement.src = 'data:image/jpeg;base64,' + data.data;
    }
  }

  else if (data.topic === 'front_camera/image_raw' && data.type === 'image' && data.data) {
    const imgElement = document.getElementById('camera');
    if (imgElement) {
      imgElement.src = 'data:image/jpeg;base64,' + data.data;
    }
  }
  
  if (data.topic === 'dist_to_target' && data.type === 'Float'){
    const dist = parseFloat(data.data).toFixed(3);
    document.getElementById('follow-distance').textContent = dist + ' m';
  }

  if (data.topic === 'reached' && data.type === 'Bool'){
    if (data.data === true) {
      document.getElementById('follow-state').textContent = '🟢 추종중...';
      landing_height = true;
    } else {
      document.getElementById('follow-state').textContent = '🔴 대기';
      landing_height = false;
      document.getElementById('drone-height').textContent = '-';

    }
  }

  if (data.topic === 'kill' && data.type === 'Bool'){
    if (data.data === true) {
      document.getElementById('person-detected').textContent = '🔴 위험 감지';
      document.getElementById('internal-temp').textContent = 'Human';
    } else {
      document.getElementById('person-detected').textContent = '🟢 정상';
      document.getElementById('internal-temp').textContent = '없음';
    } 
  }

if (data.topic === 'md/rpm_left' && data.type === 'Float') {
  document.getElementById('rpm_left').textContent = data.data;

  const fakeCurrent = (data.data * 0.005 + Math.random() * 0.05).toFixed(3); 
  document.getElementById('current_left').textContent = fakeCurrent;
}

if (data.topic === 'md/rpm_right' && data.type === 'Float') {
  document.getElementById('rpm_right').textContent = data.data;

  const fakeCurrent = (data.data * 0.005 + Math.random() * 0.05).toFixed(3);
  document.getElementById('current_right').textContent = fakeCurrent;
}


  if (data.topic === 'md/state_left' && data.type === 'UInt'){
      console.log(data.data);

    switch (data.data) {
      case 0: document.getElementById('state_left').textContent = '🟢 정상 / 상태 Bit : ' + data.data; break;
      case 1: document.getElementById('state_left').textContent = '🔴 CTRL_FAIL / 상태 Bit : ' + data.data; break;
      case 2: document.getElementById('state_left').textContent = '🔴 OVER_VOLT / 상태 Bit : ' + data.data; break;
      case 3: document.getElementById('state_left').textContent = '🔴 OVER_TEMP / 상태 Bit : ' + data.data; break;
      case 4: document.getElementById('state_left').textContent = '🔴 OVER_LOAD / 상태 Bit : ' + data.data; break;
      case 5: document.getElementById('state_left').textContent = '🔴 HALL_FAIL or ENC_FAIL / 상태 Bit : ' + data.data; break;
      case 6: document.getElementById('state_left').textContent = '🔴 INV_VEL / 상태 Bit : ' + data.data; break;
      case 7: document.getElementById('state_left').textContent = '🔴 STALL / 상태 Bit : ' + data.data; break;
    }
  }

  if (data.topic === 'md/state_right' && data.type === 'UInt'){
    switch (data.data) {
      case 0: document.getElementById('state_right').textContent = '🟢 정상 / 상태 Bit : ' + data.data; break;
      case 1: document.getElementById('state_right').textContent = '🔴 CTRL_FAIL / 상태 Bit : ' + data.data; break;
      case 2: document.getElementById('state_right').textContent = '🔴 OVER_VOLT / 상태 Bit : ' + data.data; break;
      case 3: document.getElementById('state_right').textContent = '🔴 OVER_TEMP / 상태 Bit : ' + data.data; break;
      case 4: document.getElementById('state_right').textContent = '🔴 OVER_LOAD / 상태 Bit : ' + data.data; break;
      case 5: document.getElementById('state_right').textContent = '🔴 HALL_FAIL or ENC_FAIL / 상태 Bit : ' + data.data; break;
      case 6: document.getElementById('state_right').textContent = '🔴 INV_VEL / 상태 Bit : ' + data.data; break;
      case 7: document.getElementById('state_right').textContent = '🔴 STALL / 상태 Bit : ' + data.data; break;
    }
  }

  if (data.topic === 'take_off_status' && data.type === 'Int' && data.data !== lastTakeOffStatus) {
    lastTakeOffStatus = data.data;
    console.log("info 메시지 받아옴");
    switch (data.data) {
      case 1: logInfo('📍 "격납고" ➡️ "승하차장" 으로 이동 중입니다'); break;
      case 2: logInfo('📍 승객 탑승 대기중입니다.'); break;
      case 3: 
        logInfo('✅ 승객 탑승 완료.');
        logInfo('📍 "승하차장" ➡️ "이착륙장" 으로 이동 중입니다.'); break;
      case 4: logInfo('🚁 드론 이륙 : 이륙을 시작합니다.');break;
      case 5: 
        logInfo('✅ 드론 이륙 완료.');
        logInfo('📍 "이착륙장" ➡️ "격납고" 로 이동 중입니다.'); break;
      case 6: logInfo('✅ 다음 명령을 대기합니다.'); break;
    }
  }

  if (data.topic === 'landing_status' && data.type === 'Int' && data.data !== lastLandingStatus) {
    lastLandingStatus = data.data;
    switch (data.data) {
      case 1: logInfo('📍 "격납고" ➡️ "이착륙장" 으로 이동 중입니다'); break;
      case 2: logInfo('🛬 드론 추종 : 착륙을 시작합니다.');break;
      case 3: 
        logInfo('✅ 드론 착륙 완료.');          
        logInfo('📍 "이착륙장" ➡️ "승하차장" 으로 이동 중입니다.'); break;
      case 4:
        logInfo('📍 승객 하차 중...'); break;        
      case 5:
        logInfo('✅ 승객 하차 완료.');          
        logInfo('🔋 기체 배터리 충분.');
        logInfo('📍 "승하차장" ➡️ "격납고" 로 이동 중입니다.');
        break;
      case 6:
        logInfo('✅ 승객 하차 완료.');          
        logInfo('⚡ 기체 배터리 부족.');
        logInfo('📍 "승하차장" ➡️ "충전소" 로 이동 중입니다.');
        break;
      case 7:
        logInfo('🔋 기체 배터리 충전 시작.');
        let sec = 1;
              const chargingTimer = setInterval(() => {
                if (sec <= 5) {
                  logInfo(`🔋 충전중... (${sec}초 경과)`);
                  sec++;
                } 
                if (sec > 5) {
                  clearInterval(chargingTimer);
                }
              }, 1000);
        break;
      case 8:
        setTimeout(() => {
          logInfo('🔋 기체 배터리 충전 완료.');
          logInfo('📍 "충전소" ➡️ "격납고" 로 이동 중입니다.');
        }, 1000);
        break;
      case 9:
        logInfo('✅ 다음 명령을 대기합니다.');
        break;
    }
  }
}

window.addEventListener('DOMContentLoaded', () => {
  connectWebSocket();
});

const socket = io('http://localhost:5000');

socket.on('sensor_update', data => {
  const sensor = data.sensor;
  if (!sensor) return;

  document.getElementById('bat').textContent    = sensor.bat ?? '-';
  document.getElementById('roll').textContent   = sensor.roll ?? '-';
  document.getElementById('pitch').textContent  = sensor.pitch ?? '-';
  document.getElementById('height').textContent = sensor.height ?? '-';

  if (landing_height == true){
    document.getElementById('drone-height').textContent = sensor.height;
  }

  if (ws && ws.readyState === WebSocket.OPEN) {
    const payload = {
      topic: 'sensor',
      type:  'info',
      bat:   sensor.bat,
      height: sensor.height
    };
    ws.send(JSON.stringify(payload));
  }
});


const BTN = {
  standby:   document.getElementById('btn-standby'),  
  follow:    document.getElementById('btn-follow'),    
  emergency: document.getElementById('btn-emergency'),
};

const hasBtns = BTN.standby && BTN.follow && BTN.emergency;

function setBtnDisabled(el, disabled) {
  if (!el) return;
  if (disabled) el.setAttribute('disabled', 'true');
  else el.removeAttribute('disabled');
}

const TOPIC = {
  TAKEOFF: 'take_off_scenario', 
  LANDING: 'landing_scenario',  
};
const ACTION = { TAKEOFF: 'TAKEOFF', LANDING: 'LANDING' };
let standbyAction = ACTION.TAKEOFF;

const FOLLOW = {IN: 'IN', OUT: 'OUT'};
let followAction = FOLLOW.IN

const EMERGENCY = {EMERGENCY: 'EMERGENCY', RESTART: 'RESTART' };
let emergencyAction = EMERGENCY.EMERGENCY;

function setStandbyAction(action) {
  standbyAction = action;
  if (!BTN.standby) return;
  BTN.standby.textContent = (action === ACTION.TAKEOFF) ? '이륙 승인' : '착륙 승인';
}

function setFollowAction(action){
  followAction = action;
  if (!BTN.follow) return;
  BTN.follow.textContent = (action === FOLLOW.IN) ? '승객 탑승 완료' : '승객 하차 완료';
}

function setEmergencyAction(action) {
  emergencyAction = action;
  if (!BTN.emergency) return;
  BTN.emergency.textContent = (action === EMERGENCY.EMERGENCY) ? '비상 정지' : '운행 재개';
}

function applyDefaultButtons() {
  setStandbyAction(ACTION.TAKEOFF);
  setBtnDisabled(BTN.standby, false);
  setBtnDisabled(BTN.follow,  true);
  setBtnDisabled(BTN.emergency, false);
}

function updateButtonsFromStatus() {
  if (!hasBtns) return;

  let standbyDisabled = false;
  let followDisabled  = true;
  let emergencyDisabled = false;

  if (lastTakeOffStatus != null) {
    switch (lastTakeOffStatus) {
      case 1:
        standbyDisabled = true;
        followDisabled = true;
        break;
      case 2: 
        standbyDisabled = true;
        followDisabled = false;
        break;
      case 3:
        standbyDisabled = true;
        followDisabled = true;
        break;
      case 4:
        standbyDisabled = true;
        followDisabled = true;
        break;
      case 5:
        standbyDisabled = true;
        followDisabled = true;
        break;        
      case 6:
        setStandbyAction(ACTION.LANDING);
        setFollowAction(FOLLOW.OUT);
        standbyDisabled = false;
        followDisabled  = true;
        break;
      default: 
        break;
    }
  }

  if (lastLandingStatus != null) {
    switch (lastLandingStatus) {
      case 1:
        standbyDisabled = true;
        followDisabled = true;
        break;     
      case 2:
        standbyDisabled = true;
        followDisabled = true;
        break; 
      case 3:
        standbyDisabled = true;
        followDisabled = true;
        break; 
      case 4:
        standbyDisabled = true;
        followDisabled = false;
        break; 
      case 5:
        standbyDisabled = true;
        followDisabled = true;
        break; 
      case 6:
        standbyDisabled = true;
        followDisabled = true;
        break;
      case 7:
        standbyDisabled = true;
        followDisabled = true;
        break;
      case 8:
        standbyDisabled = true;
        followDisabled = true;
        break;  
      case 9:
        setStandbyAction(ACTION.TAKEOFF);
        setFollowAction(FOLLOW.IN);
        standbyDisabled = false;
        followDisabled  = true;
        break;
      default:
        break;
    }
  }

  setBtnDisabled(BTN.standby,  standbyDisabled);
  setBtnDisabled(BTN.follow,   followDisabled);
  setBtnDisabled(BTN.emergency, emergencyDisabled);
}

if (hasBtns) applyDefaultButtons();

(function hookStatusToButtons() {
  setInterval(updateButtonsFromStatus, 300);
})();


function publishViaWS(payload) {
  if (!ws || ws.readyState !== WebSocket.OPEN) {
    logInfo('⚠️ WebSocket 미연결: 명령 전송 실패');
    return;
  }
  ws.send(JSON.stringify(payload));
}

if (hasBtns) {
  BTN.standby.addEventListener('click', () => {
    if (BTN.standby.hasAttribute('disabled')) return;

    if (standbyAction === ACTION.TAKEOFF) {
      logInfo('✅ 이륙 요청 승인');
      publishViaWS({topic : TOPIC.TAKEOFF, type : 'command', data : true});
      setBtnDisabled(BTN.standby, true);
    } 
    else {
      logInfo('✅ 착륙 요청 승인');
      publishViaWS({topic : TOPIC.LANDING, type : 'command', data : true});
      setBtnDisabled(BTN.standby, true);
    }
  });

  BTN.follow.addEventListener('click', () => {
    publishViaWS({ topic : 'start_check', type : 'command', data : true});
    setBtnDisabled(BTN.follow, true);
    publishViaWS({ topic : 'start_check', type : 'command', data : false});
  });

  BTN.emergency.addEventListener('click', () => {
    if (emergencyAction === EMERGENCY.EMERGENCY){
      logInfo('🛑 비상 정지 요청');
      publishViaWS({ topic : 'emergency_stop', type: 'command', data: true});
      setEmergencyAction(EMERGENCY.RESTART);
    }
    else{
      logInfo('🟢 운행 재개 요청');
      publishViaWS({ topic : 'emergency_stop', type: 'command', data: false});
      setEmergencyAction(EMERGENCY.EMERGENCY);
    }
  });
}
