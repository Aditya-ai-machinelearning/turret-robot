import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from flask import Flask, render_template_string
from flask_socketio import SocketIO
import threading
import base64
import cv2
import json

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins='*', async_mode='threading')

robot_state = {
    'target_status': 'searching',
    'fire_status': 'ready',
    'pan_angle': 135.0,
    'tilt_angle': 135.0,
    'error_x': 0.0,
    'error_y': 0.0,
    'bldc_status': 'off',
    'imu_status': 'simulation',
    'log': [],
    'shot_count': 0,
    'fire_enabled': True,
    'target_class': 'bottle',
    'detection_box': None,
    'confidence': 0.0
}

error_history = {'x': [], 'y': []}
MAX_HISTORY = 50

HTML = '''
<!DOCTYPE html>
<html>
<head>
<title>Turret Robot Dashboard</title>
<script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.7.5/socket.io.min.js"></script>
<script src="https://cdnjs.cloudflare.com/ajax/libs/Chart.js/4.4.0/chart.umd.min.js"></script>
<style>
*{box-sizing:border-box;margin:0;padding:0;font-family:system-ui,sans-serif}
body{background:#0f0f0f;color:#e0e0e0;padding:12px}
.topbar{display:flex;align-items:center;justify-content:space-between;padding:10px 16px;background:#1a1a1a;border-radius:10px;margin-bottom:12px;border:1px solid #2a2a2a}
.title{font-size:15px;font-weight:500;color:#fff}
.badge{font-size:11px;padding:3px 10px;border-radius:20px;font-weight:500}
.badge-green{background:#14532d;color:#4ade80}
.badge-red{background:#450a0a;color:#f87171}
.badge-amber{background:#451a03;color:#fb923c}
.grid4{display:grid;grid-template-columns:repeat(4,1fr);gap:10px;margin-bottom:12px}
.grid5{display:grid;grid-template-columns:repeat(5,1fr);gap:10px;margin-bottom:12px}
.metric{background:#1a1a1a;border-radius:8px;padding:12px 14px;border:1px solid #2a2a2a}
.metric-label{font-size:11px;color:#888;margin-bottom:4px}
.metric-value{font-size:22px;font-weight:500}
.green{color:#4ade80}.red{color:#f87171}.amber{color:#fb923c}.blue{color:#60a5fa}.white{color:#fff}
.grid-main{display:grid;grid-template-columns:2fr 1fr;gap:10px;margin-bottom:12px}
.card{background:#1a1a1a;border-radius:10px;padding:14px;border:1px solid #2a2a2a}
.card-title{font-size:12px;color:#888;margin-bottom:10px;font-weight:500;text-transform:uppercase;letter-spacing:.5px}
.cam-wrap{position:relative;background:#000;border-radius:6px;overflow:hidden}
#cam{width:100%;display:block;border-radius:6px}
#cam-canvas{position:absolute;top:0;left:0;width:100%;height:100%;pointer-events:none}
.cam-info{position:absolute;bottom:6px;left:8px;font-size:10px;color:#fff;opacity:.8;background:rgba(0,0,0,.5);padding:2px 6px;border-radius:4px}
.status-row{display:flex;flex-direction:column;gap:6px}
.status-item{display:flex;align-items:center;justify-content:space-between;padding:6px 10px;background:#111;border-radius:6px}
.status-label{font-size:12px;color:#888}
.status-val{font-size:12px;font-weight:500}
.grid-bottom{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-bottom:12px}
.dpad{display:grid;grid-template-columns:repeat(3,44px);gap:5px;margin:0 auto 10px;width:fit-content}
.dpad-btn{width:44px;height:44px;background:#222;border:1px solid #333;border-radius:8px;cursor:pointer;font-size:16px;color:#fff;display:flex;align-items:center;justify-content:center;transition:background .1s;user-select:none}
.dpad-btn:hover{background:#333}
.dpad-btn:active{background:#4ade80;color:#000;transform:scale(0.95)}
.dpad-empty{width:44px;height:44px}
.speed-row{display:flex;align-items:center;gap:8px;margin-bottom:8px}
.speed-label{font-size:12px;color:#888;white-space:nowrap}
input[type=range]{flex:1;accent-color:#4ade80}
.angle-row{display:grid;grid-template-columns:repeat(4,1fr);gap:6px;margin-bottom:10px}
.angle-card{background:#111;border-radius:6px;padding:8px;text-align:center}
.angle-label{font-size:10px;color:#888;margin-bottom:2px}
.angle-val{font-size:15px;font-weight:500;color:#fff}
.log-box{background:#111;border-radius:6px;padding:8px;font-size:11px;font-family:monospace;line-height:1.9;height:80px;overflow:hidden}
.log-green{color:#4ade80}.log-red{color:#f87171}.log-amber{color:#fb923c}.log-blue{color:#60a5fa}
.safety-btn{width:100%;padding:10px;border-radius:8px;border:none;cursor:pointer;font-size:13px;font-weight:500;transition:all .2s;margin-bottom:8px}
.safety-on{background:#14532d;color:#4ade80;border:1px solid #166534}
.safety-off{background:#450a0a;color:#f87171;border:1px solid #7f1d1d}
.target-row{display:flex;gap:6px;margin-bottom:8px;flex-wrap:wrap}
.target-btn{padding:5px 12px;border-radius:20px;border:1px solid #333;background:#222;color:#888;cursor:pointer;font-size:12px;transition:all .15s}
.target-btn.active{background:#1d4ed8;color:#60a5fa;border-color:#2563eb}
.target-btn:hover{background:#333}
.shot-big{font-size:36px;font-weight:500;color:#f87171;text-align:center;padding:8px 0}
.graph-wrap{background:#111;border-radius:6px;padding:8px;margin-top:8px}
</style>
</head>
<body>

<div class="topbar">
  <span class="title">Turret robot — live dashboard</span>
  <div style="display:flex;gap:8px;align-items:center">
    <span class="badge badge-green">connected</span>
    <span class="badge badge-amber">simulation mode</span>
    <span class="badge" id="fire-badge" style="background:#14532d;color:#4ade80">fire enabled</span>
  </div>
</div>

<div class="grid5">
  <div class="metric">
    <div class="metric-label">Target status</div>
    <div class="metric-value" id="target-status">SEARCHING</div>
  </div>
  <div class="metric">
    <div class="metric-label">Fire status</div>
    <div class="metric-value" id="fire-status">READY</div>
  </div>
  <div class="metric">
    <div class="metric-label">Pan angle</div>
    <div class="metric-value white" id="pan-angle">135.0°</div>
  </div>
  <div class="metric">
    <div class="metric-label">Tilt angle</div>
    <div class="metric-value white" id="tilt-angle">135.0°</div>
  </div>
  <div class="metric">
    <div class="metric-label">Shots fired</div>
    <div class="metric-value red" id="shot-count">0</div>
  </div>
</div>

<div class="grid-main">
  <div class="card">
    <div class="card-title">Camera feed — /image_raw</div>
    <div class="cam-wrap">
      <img id="cam" src="" alt="Waiting for camera..."/>
      <canvas id="cam-canvas"></canvas>
      <span class="cam-info" id="cam-info">waiting...</span>
    </div>
  </div>
  <div class="card">
    <div class="card-title">System status</div>
    <div class="status-row">
      <div class="status-item">
        <span class="status-label">Camera</span>
        <span class="status-val green">online</span>
      </div>
      <div class="status-item">
        <span class="status-label">Detection</span>
        <span class="status-val green">running</span>
      </div>
      <div class="status-item">
        <span class="status-label">Target</span>
        <span class="status-val" id="s-target">searching</span>
      </div>
      <div class="status-item">
        <span class="status-label">BLDC motors</span>
        <span class="status-val" id="s-bldc">off</span>
      </div>
      <div class="status-item">
        <span class="status-label">Fire</span>
        <span class="status-val green" id="s-fire">ready</span>
      </div>
      <div class="status-item">
        <span class="status-label">IMU</span>
        <span class="status-val amber">simulation</span>
      </div>
    </div>
  </div>
</div>

<div class="grid-bottom">
  <div class="card">
    <div class="card-title">Car steering</div>
    <div class="dpad">
      <div class="dpad-empty"></div>
      <div class="dpad-btn" id="btn-fwd">▲</div>
      <div class="dpad-empty"></div>
      <div class="dpad-btn" id="btn-left">◀</div>
      <div class="dpad-btn" id="btn-stop">■</div>
      <div class="dpad-btn" id="btn-right">▶</div>
      <div class="dpad-empty"></div>
      <div class="dpad-btn" id="btn-back">▼</div>
      <div class="dpad-empty"></div>
    </div>
    <div class="speed-row">
      <span class="speed-label">Speed</span>
      <input type="range" min="0" max="100" value="50" id="speed-slider">
      <span style="font-size:12px;color:#888;min-width:32px" id="speed-val">50%</span>
    </div>

    <div class="card-title" style="margin-top:8px">Fire safety</div>
    <button class="safety-btn safety-on" id="safety-btn"
      onclick="toggleSafety()">FIRE ENABLED — click to disable</button>

    <div class="card-title">Track target</div>
    <div class="target-row" id="target-row">
      <button class="target-btn active" onclick="setTarget('bottle')">Bottle</button>
      <button class="target-btn" onclick="setTarget('cup')">Cup</button>
      <button class="target-btn" onclick="setTarget('person')">Person</button>
      <button class="target-btn" onclick="setTarget('sports ball')">Ball</button>
      <button class="target-btn" onclick="setTarget('cell phone')">Phone</button>
    </div>
  </div>

  <div class="card">
    <div class="card-title">Angles + shot counter</div>
    <div style="display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-bottom:10px">
      <div>
        <div class="angle-row">
          <div class="angle-card">
            <div class="angle-label">Pan</div>
            <div class="angle-val" id="a-pan">135°</div>
          </div>
          <div class="angle-card">
            <div class="angle-label">Tilt</div>
            <div class="angle-val" id="a-tilt">135°</div>
          </div>
          <div class="angle-card">
            <div class="angle-label">Error X</div>
            <div class="angle-val" id="a-ex">0.0</div>
          </div>
          <div class="angle-card">
            <div class="angle-label">Error Y</div>
            <div class="angle-val" id="a-ey">0.0</div>
          </div>
        </div>
        <div class="log-box" id="log-box"></div>
      </div>
      <div style="text-align:center">
        <div style="font-size:11px;color:#888;margin-bottom:4px">Total shots</div>
        <div class="shot-big" id="shot-big">0</div>
        <div style="font-size:11px;color:#888;margin-top:8px">Tracking confidence</div>
        <div style="font-size:20px;font-weight:500;color:#fb923c;margin-top:4px" id="conf-val">0%</div>
      </div>
    </div>
    <div class="graph-wrap">
      <div style="font-size:11px;color:#888;margin-bottom:6px">Live error graph</div>
      <canvas id="error-chart" height="80"></canvas>
    </div>
  </div>
</div>

<script>
const socket = io();
let speed = 0.5;
let fireEnabled = true;
let currentTarget = 'bottle';
let shotCount = 0;

const ctx = document.getElementById('error-chart').getContext('2d');
const errorChart = new Chart(ctx, {
  type: 'line',
  data: {
    labels: Array(50).fill(''),
    datasets: [
      {label:'Error X', data:Array(50).fill(0),
       borderColor:'#f87171', borderWidth:1.5,
       pointRadius:0, tension:0.3, fill:false},
      {label:'Error Y', data:Array(50).fill(0),
       borderColor:'#60a5fa', borderWidth:1.5,
       pointRadius:0, tension:0.3, fill:false}
    ]
  },
  options: {
    animation:false,
    plugins:{legend:{labels:{color:'#888',font:{size:10}}}},
    scales:{
      x:{display:false},
      y:{grid:{color:'#222'},ticks:{color:'#888',font:{size:10}},
         min:-320, max:320}
    }
  }
});

socket.on('state', data => {
  const el = id => document.getElementById(id);
  const ts = data.target_status;
  el('target-status').textContent = ts.toUpperCase();
  el('target-status').className = 'metric-value ' +
    (ts==='locked'?'green':ts==='acquiring'?'amber':'red');

  const fs = data.fire_status;
  el('fire-status').textContent = fs.toUpperCase();
  el('fire-status').className = 'metric-value ' +
    (fs==='fired'?'red':fs==='cooldown'?'amber':'green');

  el('pan-angle').textContent = data.pan_angle.toFixed(1) + '°';
  el('tilt-angle').textContent = data.tilt_angle.toFixed(1) + '°';
  el('a-pan').textContent = data.pan_angle.toFixed(1) + '°';
  el('a-tilt').textContent = data.tilt_angle.toFixed(1) + '°';
  el('a-ex').textContent = data.error_x.toFixed(1);
  el('a-ey').textContent = data.error_y.toFixed(1);

  const st = el('s-target');
  st.textContent = ts;
  st.className = 'status-val '+(ts==='locked'?'green':ts==='acquiring'?'amber':'red');

  const sb = el('s-bldc');
  sb.textContent = data.bldc_status;
  sb.className = 'status-val '+(data.bldc_status==='spinning'?'green':'red');

  const sf = el('s-fire');
  sf.textContent = fs;
  sf.className = 'status-val '+(fs==='ready'?'green':fs==='fired'?'red':'amber');

  shotCount = data.shot_count;
  el('shot-count').textContent = shotCount;
  el('shot-big').textContent = shotCount;
  el('conf-val').textContent = (data.confidence * 100).toFixed(0) + '%';

  errorChart.data.datasets[0].data.push(data.error_x);
  errorChart.data.datasets[0].data.shift();
  errorChart.data.datasets[1].data.push(data.error_y);
  errorChart.data.datasets[1].data.shift();
  errorChart.update();
});

socket.on('frame', data => {
  const img = document.getElementById('cam');
  img.src = 'data:image/jpeg;base64,' + data.img;
  document.getElementById('cam-info').textContent =
    `640x480 · ${data.fps}fps · target: ${currentTarget}`;
});

socket.on('log', data => {
  const box = document.getElementById('log-box');
  const cls = data.type==='fire'?'log-red':data.type==='lock'?'log-green':
              data.type==='warn'?'log-amber':'log-blue';
  box.innerHTML = `<span class="${cls}">[${data.tag}]</span> ${data.msg}<br>` + box.innerHTML;
  const lines = box.innerHTML.split('<br>');
  if(lines.length > 5) box.innerHTML = lines.slice(0,5).join('<br>');
});

function toggleSafety() {
  fireEnabled = !fireEnabled;
  const btn = document.getElementById('safety-btn');
  const badge = document.getElementById('fire-badge');
  if(fireEnabled) {
    btn.textContent = 'FIRE ENABLED — click to disable';
    btn.className = 'safety-btn safety-on';
    badge.style.background = '#14532d';
    badge.style.color = '#4ade80';
    badge.textContent = 'fire enabled';
  } else {
    btn.textContent = 'FIRE DISABLED — click to enable';
    btn.className = 'safety-btn safety-off';
    badge.style.background = '#450a0a';
    badge.style.color = '#f87171';
    badge.textContent = 'fire disabled';
  }
  socket.emit('fire_enable', {enabled: fireEnabled});
}

function setTarget(cls) {
  currentTarget = cls;
  document.querySelectorAll('.target-btn').forEach(b => b.classList.remove('active'));
  event.target.classList.add('active');
  socket.emit('set_target', {target: cls});
}

document.getElementById('speed-slider').addEventListener('input', e => {
  speed = e.target.value / 100;
  document.getElementById('speed-val').textContent = e.target.value + '%';
});

function sendDrive(x, y, z) {
  socket.emit('drive', {x, y, z});
}

document.getElementById('btn-fwd').addEventListener('click', () => sendDrive(speed,0,0));
document.getElementById('btn-back').addEventListener('click', () => sendDrive(-speed,0,0));
document.getElementById('btn-left').addEventListener('click', () => sendDrive(0,0,speed));
document.getElementById('btn-right').addEventListener('click', () => sendDrive(0,0,-speed));
document.getElementById('btn-stop').addEventListener('click', () => sendDrive(0,0,0));

document.addEventListener('keydown', e => {
  const k = e.key.toLowerCase();
  if(k==='w') sendDrive(speed,0,0);
  else if(k==='s') sendDrive(-speed,0,0);
  else if(k==='a') sendDrive(0,0,speed);
  else if(k==='d') sendDrive(0,0,-speed);
  else if(k===' ') { e.preventDefault(); sendDrive(0,0,0); }
});
</script>
</body>
</html>
'''

class DashboardNode(Node):

    def __init__(self):
        super().__init__('dashboard_node')
        self.bridge = CvBridge()
        self.last_frame_time = 0
        self.fps = 0
        self.frame_count = 0
        import time
        self.fps_timer = time.time()

        self.create_subscription(
            String, '/turret_status', self.on_status, 10)
        self.create_subscription(
            Float32MultiArray, '/servo_cmd', self.on_servo, 10)
        self.create_subscription(
            Float32MultiArray, '/target_error', self.on_error, 10)
        self.create_subscription(
            Bool, '/locked', self.on_locked, 10)
        self.create_subscription(
            Bool, '/target_detected', self.on_detected, 10)
        self.create_subscription(
            Image, '/image_raw', self.on_image, 10)
        self.create_subscription(
            String, '/detections', self.on_detections, 10)

        from geometry_msgs.msg import Twist
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_pub = self.create_publisher(String, '/set_target', 10)
        self.fire_enable_pub = self.create_publisher(Bool, '/fire_enable', 10)

        self.current_box = None
        self.current_conf = 0.0

        self.get_logger().info(
            'Dashboard node started! Open http://localhost:5000')

    def on_detections(self, msg):
        import json
        try:
            detections = json.loads(msg.data)
            target = robot_state['target_class']
            targets = [d for d in detections if d['class'] == target]
            if targets:
                best = max(targets, key=lambda d: d['conf'])
                robot_state['detection_box'] = best
                robot_state['confidence'] = best['conf']
                self.current_conf = best['conf']
            else:
                robot_state['detection_box'] = None
                robot_state['confidence'] = 0.0
        except:
            pass

    def on_status(self, msg):
        s = msg.data.lower()
        if 'locked' in s:
            robot_state['target_status'] = 'locked'
            self.emit_log('LOCKED', 'target locked on!', 'lock')
        elif 'acquiring' in s:
            robot_state['target_status'] = 'acquiring'
        else:
            robot_state['target_status'] = 'tracking'
        self.emit_state()

    def on_servo(self, msg):
        robot_state['pan_angle'] = round(msg.data[0], 1)
        robot_state['tilt_angle'] = round(msg.data[1], 1)
        self.emit_state()

    def on_error(self, msg):
        robot_state['error_x'] = round(msg.data[0], 1)
        robot_state['error_y'] = round(msg.data[1], 1)

    def on_locked(self, msg):
        if msg.data and robot_state.get('fire_enabled', True):
            robot_state['fire_status'] = 'fired'
            robot_state['shot_count'] += 1
            self.emit_log('FIRE', f"shot #{robot_state['shot_count']} fired!", 'fire')
        else:
            robot_state['fire_status'] = 'ready'
        self.emit_state()

    def on_detected(self, msg):
        if msg.data:
            robot_state['bldc_status'] = 'spinning'
            if robot_state['target_status'] == 'searching':
                robot_state['target_status'] = 'tracking'
        else:
            robot_state['bldc_status'] = 'off'
            robot_state['target_status'] = 'searching'
        self.emit_state()

    def on_image(self, msg):
        import time
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # draw detection box on dashboard feed
            box = robot_state.get('detection_box')
            if box:
                x1 = int(box['x1'])
                y1 = int(box['y1'])
                x2 = int(box['x2'])
                y2 = int(box['y2'])
                conf = box['conf']
                cls = box['class']
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                # green box
                cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,0), 2)
                # label
                cv2.putText(frame, f'{cls} {conf:.0%}',
                    (x1, y1-8), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0,255,0), 2)
                # center dot
                cv2.circle(frame, (cx,cy), 5, (0,255,0), -1)

            # draw crosshair
            h, w = frame.shape[:2]
            cx_f, cy_f = w//2, h//2
            cv2.line(frame,(cx_f-20,cy_f),(cx_f+20,cy_f),(0,0,255),2)
            cv2.line(frame,(cx_f,cy_f-20),(cx_f,cy_f+20),(0,0,255),2)

            # calculate fps
            self.frame_count += 1
            now = time.time()
            if now - self.fps_timer >= 1.0:
                self.fps = self.frame_count
                self.frame_count = 0
                self.fps_timer = now

            _, buf = cv2.imencode('.jpg', frame,
                [cv2.IMWRITE_JPEG_QUALITY, 65])
            b64 = base64.b64encode(buf).decode('utf-8')
            socketio.emit('frame', {'img': b64, 'fps': self.fps})
        except Exception as e:
            pass

    def emit_state(self):
        socketio.emit('state', robot_state)

    def emit_log(self, tag, msg, type):
        socketio.emit('log', {'tag': tag, 'msg': msg, 'type': type})
        robot_state['log'].insert(0, f'[{tag}] {msg}')
        robot_state['log'] = robot_state['log'][:10]

@app.route('/')
def index():
    return render_template_string(HTML)

@socketio.on('drive')
def on_drive(data):
    from geometry_msgs.msg import Twist
    msg = Twist()
    msg.linear.x = float(data['x'])
    msg.linear.y = float(data['y'])
    msg.angular.z = float(data['z'])
    node_ref[0].cmd_pub.publish(msg)

@socketio.on('fire_enable')
def on_fire_enable(data):
    robot_state['fire_enabled'] = data['enabled']
    msg = Bool()
    msg.data = data['enabled']
    node_ref[0].fire_enable_pub.publish(msg)
    status = 'enabled' if data['enabled'] else 'DISABLED'
    node_ref[0].emit_log('SAFETY', f'fire {status}', 'warn')

@socketio.on('set_target')
def on_set_target(data):
    robot_state['target_class'] = data['target']
    msg = String()
    msg.data = data['target']
    node_ref[0].target_pub.publish(msg)
    node_ref[0].emit_log('TARGET', f"tracking: {data['target']}", 'blue')

node_ref = [None]

def main():
    rclpy.init()
    node = DashboardNode()
    node_ref[0] = node

    flask_thread = threading.Thread(
        target=lambda: socketio.run(
            app, host='0.0.0.0', port=5000,
            allow_unsafe_werkzeug=True))
    flask_thread.daemon = True
    flask_thread.start()

    rclpy.spin(node)
    rclpy.shutdown()