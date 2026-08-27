import sqlite3, bisect, math, re
import numpy as np
import cv2
import rclpy  # noqa - ensures ament env
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage, Image, LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log

BAG = '/home/jetson-04/leo_bags/explore_20260814_run3/explore_20260814_run3_0.db3'
OUT = '/home/jetson-04/leo_maps/explore_20260814_labeled.mp4'
db = sqlite3.connect(BAG)
tid = {name: i for i, name in db.execute('SELECT id, name FROM topics')}

def load(topic, msgtype):
    ts, ms = [], []
    for t, d in db.execute('SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp', (tid[topic],)):
        ts.append(t/1e9); ms.append(d)
    return ts, ms, msgtype

def nearest(store, t):
    ts, ms, mt = store
    if not ts: return None
    i = bisect.bisect_left(ts, t)
    if i == 0: j = 0
    elif i >= len(ts): j = len(ts)-1
    else: j = i if ts[i]-t < t-ts[i-1] else i-1
    if abs(ts[j]-t) > 1.5: return None
    return deserialize_message(ms[j], mt)

# The colour topic is jpeg now (record_rover_bag.sh records
# /debug/color_5hz/compressed -- the raw one is 6x the bytes for the same
# picture). Older bags carry the raw Image; accept either.
_COLOR = [('/bag/color/compressed', CompressedImage),
          ('/debug/color_5hz/compressed', CompressedImage),
          ('/debug/color_5hz', Image)]          # oldest name last
for _t, _ty in _COLOR:
    if _t in tid:
        frames = load(_t, _ty)
        break
else:
    raise SystemExit(f'bag has none of {[t for t, _ in _COLOR]}')


def decode(raw, msgtype):
    m = deserialize_message(raw, msgtype)
    if msgtype is CompressedImage:
        return cv2.imdecode(np.frombuffer(m.data, dtype=np.uint8),
                            cv2.IMREAD_COLOR)
    rgb = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, 3)
    return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
fused = load('/scan_collision_fused', LaserScan)
camscan = load('/camera/scan_collision', LaserScan)
req = load('/cmd_vel_request', Twist)
out = load('/cmd_vel', Twist)
odom = load('/wheel_odom', Odometry)

# explorer status lines from rosout
exp_ts, exp_lines = [], []
gate_ts, gate_lines = [], []
for t, d in db.execute('SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp', (tid['/rosout'],)):
    m = deserialize_message(d, Log)
    if m.name == 'safe_room_explorer':
        exp_ts.append(t/1e9); exp_lines.append(m.msg)
    elif m.name in ('safety_command_gate','collision_monitor') and ('closed' in m.msg or 'stop' in m.msg.lower()):
        gate_ts.append(t/1e9); gate_lines.append(m.name+': '+m.msg)

def last_before(ts_list, lines, t, maxage):
    i = bisect.bisect_right(ts_list, t) - 1
    if i < 0 or t - ts_list[i] > maxage: return None
    return lines[i]

W, H = 1120, 560
CAM_W, CAM_H = 640, 480
writer = cv2.VideoWriter(OUT, cv2.VideoWriter_fourcc(*'mp4v'), 5, (W, H))
t_run0 = req[0][0] if req[0] else frames[0][0]
path_len, last_p = 0.0, None
n = len(frames[0])
for k in range(n):
    t = frames[0][k]
    bgr = decode(frames[1][k], frames[2])
    canvas = np.zeros((H, W, 3), dtype=np.uint8)
    canvas[40:40+CAM_H, 0:CAM_W] = cv2.resize(bgr, (CAM_W, CAM_H))

    om = nearest(odom, t)
    if om is not None:
        p = (om.pose.pose.position.x, om.pose.pose.position.y)
        if last_p is not None:
            path_len += math.hypot(p[0]-last_p[0], p[1]-last_p[1])
        last_p = p
    rq = nearest(req, t); ot = nearest(out, t)
    rv = abs(rq.linear.x)+abs(rq.angular.z) if rq else 0.0
    ov = abs(ot.linear.x)+abs(ot.angular.z) if ot else 0.0
    if rq is None or rv < 0.01:
        state, color = 'IDLE / NO REQUEST', (160,160,160)
    elif ov < 0.01:
        state, color = 'OBSTACLE: CM HOLDING (request blocked)', (0,0,255)
    elif ov < 0.7*rv:
        state, color = 'OBSTACLE NEAR: CM SLOWDOWN', (0,165,255)
    else:
        state, color = 'DRIVING (request passed)', (0,200,0)
    cv2.rectangle(canvas, (0,0), (W,36), color, -1)
    cv2.putText(canvas, state, (8,26), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,0), 2)
    cv2.putText(canvas, 't=%.1fs' % (t-t_run0), (W-130,26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 2)

    # lidar panel
    px0, py0, R = CAM_W+240, 40+240, 240
    scale = R/3.5
    cv2.rectangle(canvas, (CAM_W,40), (W,40+CAM_H), (25,25,25), -1)
    cv2.circle(canvas, (px0,py0), int(0.35*scale), (0,0,180), 1)
    for rr in (1,2,3):
        cv2.circle(canvas, (px0,py0), int(rr*scale), (60,60,60), 1)
    fs = nearest(fused, t)
    if fs is not None:
        a = fs.angle_min
        for r in fs.ranges:
            if 0.05 < r < 3.5 and math.isfinite(r):
                x, y = r*math.cos(a), r*math.sin(a)
                cv2.circle(canvas, (px0+int(-y*scale), py0-int(x*scale)), 2, (255,255,255), -1)
            a += fs.angle_increment
    cs = nearest(camscan, t)
    cmin = float('inf')
    if cs is not None:
        a = cs.angle_min
        for r in cs.ranges:
            if 0.05 < r < 3.5 and math.isfinite(r):
                cmin = min(cmin, r)
                x, y = r*math.cos(a), r*math.sin(a)
                cv2.circle(canvas, (px0+int(-y*scale), py0-int(x*scale)), 2, (0,220,220), -1)
            a += cs.angle_increment
    tri = np.array([[px0, py0-8],[px0-6, py0+6],[px0+6, py0+6]])
    cv2.fillPoly(canvas, [tri], (0,200,0))
    cv2.putText(canvas, 'LIDAR fused (white) / camera (yellow)', (CAM_W+10, 40+CAM_H-10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200,200,200), 1)

    # bottom info
    y = 40+CAM_H
    cv2.rectangle(canvas, (0,y), (W,H), (35,35,35), -1)
    info = last_before(exp_ts, exp_lines, t, 3.0)
    mode = ''
    if info:
        mm = re.search(r'mode=(\S+).*battery=([\d.]+)', info)
        if mm: mode = 'explorer mode=%s  battery=%sV' % (mm.group(1), mm.group(2))
        elif 'stopping' in info: mode = 'explorer: ' + info[:80]
    rqs = 'req vx=%.2f wz=%.2f' % (rq.linear.x, rq.angular.z) if rq else 'req --'
    ots = 'CM out vx=%.2f wz=%.2f' % (ot.linear.x, ot.angular.z) if ot else 'CM out --'
    camflag = ('cam min=%.2fm' % cmin) if cmin < 3.5 else 'cam clear'
    cv2.putText(canvas, '%s | %s | path=%.2fm | %s' % (rqs, ots, path_len, camflag), (8, y+22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255,255,255), 1)
    if mode:
        cv2.putText(canvas, mode, (8, y+42), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180,255,180), 1)
    g = last_before(gate_ts, gate_lines, t, 2.0)
    if g:
        cv2.putText(canvas, g[:110], (8, y+62), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0,120,255), 1)
    writer.write(canvas)
    if k % 200 == 0:
        print('frame %d/%d' % (k, n), flush=True)
writer.release()
print('DONE', OUT, flush=True)
