import rclpy, time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

rclpy.init()
n = Node('fw_stability_monitor')
counts = {'odom': 0, 'batt': 0}
last_batt = [None]
n.create_subscription(Odometry, '/rob_2/firmware/wheel_odom', lambda m: counts.__setitem__('odom', counts['odom']+1), qos_profile_sensor_data)
def bcb(m):
    counts['batt'] += 1
    last_batt[0] = m.data
n.create_subscription(Float32, '/rob_2/firmware/battery_averaged', bcb, qos_profile_sensor_data)

def net():
    for line in open('/proc/net/dev'):
        if 'enP8p1s0' in line:
            f = line.replace(':',' ').split()
            return int(f[1]), int(f[9])
    return 0, 0

start = time.time()
r0, t0 = net()
print('bin_end_s odom_hz batt_hz batt_V tx_KBs rx_KBs', flush=True)
while time.time() - start < 300:
    b0 = time.time()
    counts['odom'] = 0; counts['batt'] = 0
    while time.time() - b0 < 30:
        rclpy.spin_once(n, timeout_sec=0.2)
    r1, t1 = net()
    dt = time.time() - b0
    bv = last_batt[0] if last_batt[0] is not None else -1
    print(f'{int(time.time()-start)} {counts["odom"]/dt:.1f} {counts["batt"]/dt:.1f} {bv:.2f} {(t1-t0)/dt/1024:.0f} {(r1-r0)/dt/1024:.0f}', flush=True)
    r0, t0 = r1, t1
print('DONE', flush=True)
