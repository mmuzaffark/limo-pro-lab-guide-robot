#!/usr/bin/env python3
# guide_manager_foxy.py : guide behaviour on the robot. Commands on /guide/command (std_msgs/String):
#   go PLACE | home | cancel | return on | return off | list | status | quit
#   quit = drive home (2 attempts), publish state 'shutdown', then stop all robot software
#   (the limo-guide service, or the guide_start.sh process group). The Jetson stays on.
# Status JSON on /guide/status (repeated every 5 s with "hb": true). Battery safety: below low_volt
# for low_hold_s seconds it refuses new trips and drives home.
import json, math, os, signal, subprocess, sys, time, yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
try:
    from limo_msgs.msg import LimoStatus
except ImportError:
    LimoStatus = None


class Guide(Node):
    def __init__(self, places_file):
        super().__init__('guide_manager')
        self.places = yaml.safe_load(open(places_file))['places']
        self.declare_parameter('wait_s', 10.0)
        self.declare_parameter('low_volt', 11.0)
        self.declare_parameter('low_hold_s', 20.0)
        self.ret = True; self.goal = None; self.timer = None; self.seq = 0
        self.target = 'home'; self.volts = []; self.low_sent = False; self.last = {}
        self.quitting = False; self.home_tries = 0; self.done = False
        self.ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.pub = self.create_publisher(String, '/guide/status', 10)
        self.create_subscription(String, '/guide/command', self.on_cmd, 10)
        if LimoStatus is not None:
            self.create_subscription(LimoStatus, '/limo_status', self.on_battery, 10)
        self.create_timer(5.0, self.heartbeat)
        self.create_timer(2.0, self.check_battery)
        self.status('idle', 'home', 'ready: ' + ', '.join(self.places))

    # ---------- battery ----------
    def on_battery(self, m):
        t = time.time(); self.volts.append((t, m.battery_voltage))
        self.volts = [(a, v) for (a, v) in self.volts if t - a < 60]

    def volt(self):
        return self.volts[-1][1] if self.volts else None

    def battery_low(self):
        hold = float(self.get_parameter('low_hold_s').value); t = time.time()
        recent = [v for (a, v) in self.volts if t - a <= hold]
        span = (t - min(a for (a, v) in self.volts if t - a <= hold)) if recent else 0
        return len(recent) >= 5 and span >= hold * 0.8 and max(recent) < float(self.get_parameter('low_volt').value)

    def check_battery(self):
        if self.quitting: return  # already on the way home
        if not self.battery_low():
            self.low_sent = False; return
        if self.low_sent: return
        self.low_sent = True
        if self.target != 'home' or self.timer is not None:
            self.status('battery', self.target, 'battery low (%.2f V), returning home' % self.volt())
            self.send('home')
        else:
            self.status('battery', 'home', 'battery low (%.2f V): please charge me' % self.volt())

    # ---------- status ----------
    def status(self, state, target, msg='', log=True):
        v = self.volt()
        self.last = {'state': state, 'target': target, 'msg': msg, 'battery': round(v, 2) if v else None}
        s = String(); s.data = json.dumps(self.last); self.pub.publish(s)
        if log: self.get_logger().info(s.data)

    def heartbeat(self):
        if self.last:
            d = dict(self.last); d['hb'] = True; v = self.volt(); d['battery'] = round(v, 2) if v else None
            s = String(); s.data = json.dumps(d); self.pub.publish(s)

    # ---------- commands ----------
    def on_cmd(self, m):
        w = m.data.strip().lower().split()
        if not w:
            return
        if self.quitting and w[0] != 'status':
            self.status('quitting', 'home', 'shutting down: command ignored'); return
        if w[0] == 'quit':
            self.quitting = True; self.home_tries = 0
            self.status('quitting', 'home', 'returning home, then stopping the robot software')
            self.send('home'); return
        if w[0] == 'go' and len(w) > 1:
            name = '_'.join(w[1:])
            if name not in self.places:
                self.status('error', name, 'unknown place')
            elif self.battery_low():
                self.status('error', name, 'battery low (%.2f V): please charge me' % self.volt())
            else:
                self.send(name)
        elif w[0] in ('home', 'cancel', 'stop'):
            self.send('home')
        elif w[0] == 'return' and len(w) > 1:
            self.ret = (w[1] == 'on')
            self.status('info', None, 'return home ' + ('on' if self.ret else 'off'))
        elif w[0] == 'list':
            self.status('info', None, ', '.join(self.places))
        elif w[0] == 'status' and self.last:
            self.status(self.last['state'], self.last['target'], self.last['msg'], log=False)

    def clear_timer(self):
        if self.timer is not None:
            self.timer.cancel(); self.destroy_timer(self.timer); self.timer = None

    def send(self, name):
        self.clear_timer()
        self.seq += 1; s = self.seq; self.target = name
        if self.goal is not None:
            self.goal.cancel_goal_async(); self.goal = None
        if not self.ac.wait_for_server(timeout_sec=2.0):
            self.status('error', name, 'Nav2 not running')
            if self.quitting: self.finish(False)
            return
        x, y, yaw = self.places[name]
        g = NavigateToPose.Goal(); g.pose = PoseStamped()
        g.pose.header.frame_id = 'map'; g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x = float(x); g.pose.pose.position.y = float(y)
        g.pose.pose.orientation.z = math.sin(yaw / 2.0); g.pose.pose.orientation.w = math.cos(yaw / 2.0)
        if not self.quitting: self.status('going', name)
        self.ac.send_goal_async(g).add_done_callback(lambda f: self.on_accept(f, name, s))

    def on_accept(self, f, name, s):
        h = f.result()
        if s != self.seq:
            if h.accepted:
                h.cancel_goal_async()
            return
        if not h.accepted:
            self.status('failed', name, 'goal rejected')
            if self.quitting: self.home_failed()
            return
        self.goal = h
        h.get_result_async().add_done_callback(lambda r: self.on_result(r, name, s))

    def on_result(self, r, name, s):
        if s != self.seq:
            return
        self.goal = None
        st = r.result().status
        if self.quitting:
            if st == GoalStatus.STATUS_SUCCEEDED: self.finish(True)
            else: self.home_failed()
            return
        if st == GoalStatus.STATUS_SUCCEEDED:
            if name == 'home':
                self.status('idle', 'home', 'at home' + (': battery low, please charge me' if self.battery_low() else ''))
            elif self.ret:
                w = float(self.get_parameter('wait_s').value)
                self.status('arrived', name, 'returning home in %d s' % w)
                self.timer = self.create_timer(w, lambda: self.send('home'))
            else:
                self.status('arrived', name, 'staying here')
        else:
            self.status('failed', name, 'navigation status %d' % st)
            if name != 'home':
                self.send('home')

    # ---------- quit ----------
    def home_failed(self):
        self.home_tries += 1
        if self.home_tries < 2:
            self.status('quitting', 'home', 'home not reached, trying again'); self.send('home')
        else:
            self.finish(False)

    def finish(self, at_home):
        self.status('shutdown', 'home', ('at home' if at_home else 'NOT at home: carry the robot to the home mark') +
                    ': stopping the robot software')
        self.timer = self.create_timer(2.0, self.stop_all)  # time for the GUI to receive the last status

    def stop_all(self):
        self.clear_timer()
        if os.environ.get('INVOCATION_ID'):  # started by systemd: a plain exit would be restarted
            subprocess.Popen(['sudo', '-n', 'systemctl', 'stop', 'limo-guide'], start_new_session=True)
        else:
            try:
                ppid = os.getppid()
                if 'guide_start' in open('/proc/%d/cmdline' % ppid).read():
                    os.killpg(os.getpgid(ppid), signal.SIGINT)  # = Ctrl+C in the guide_start.sh window
            except (OSError, ProcessLookupError):
                pass
        self.done = True


def main():
    rclpy.init()
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    node = Guide(args[0] if args else '/home/agilex/guide_ws/places_lab.yaml')
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    try:
        node.destroy_node(); rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
