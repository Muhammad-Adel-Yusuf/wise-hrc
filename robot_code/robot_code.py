# -*- coding: utf-8 -*-
"""
ROBOT CODE: moves the real UFactory Lite 6 according to actions.md (no input needed).

    python robot_code/robot_code.py

What it does
  1. opens the SDK relay of the Jetson gateway for this PC (saved login, see below)
     and connects to the arm with XArmAPI('<jetson ip>');
  2. reads actions.md (written by the MATLAB POMDP program): it holds the task
     number, e.g. "0" -> tasks/task0.txt;
  3. reads the task file and moves the arm through pos0, pos1, ... at its speed;
  4. loop:true  -> repeats the positions until actions.md changes to a new task;
     loop:false -> goes through the positions once, then waits;
  5. keeps watching actions.md forever: a new task stops the current motion and
     starts the new task. Ctrl+C stops the arm and ends the program.
  ANY KEY stops the robot at once and pauses (ENTER: follow actions.md again).
  SPEED_CAP below limits every action (slow 5 deg/s for the first tests).
  Live with MATLAB: start this program first, then main.py, then liveHRC (it writes actions.md after G).

Task file = ACTION file (tasks/task<N>.txt, N = the POMDP action number; points in tasks/robot_points.txt)
    speed=slow                          slow | normal (= medium) | fast | a number in deg/s
    path=pos5,pos6,pos7                 points visited in order
    entry=pos9,pos10                    visited first, only when the robot is not already on the path
    wait=pos12:30                       pause 30 s at pos12 on every pass
    loop=true                           repeat the path until a new action arrives
    stop=true                           (instead of a path) stop where the robot is and hold
  The older format with the points in the file (pos0:[j1..j6] lines, loop:false) still works.
Test the actions one by one with:  python robot_code/test_actions.py

Once per computer, before the first run (the password is never stored here):
    python robot_code/lite6.py login 10.13.37.101
"""

import os
import re
import sys
import time
import traceback

# ======================= settings you can change =========================
HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT = os.path.dirname(HERE)                       # pilot_physical_full_code
ACTIONS_FILE = os.path.join(PROJECT, 'actions.md')
TASKS_DIR = os.path.join(PROJECT, 'tasks')
TASK_FILE = 'task{}.txt'                              # task number -> file name
POINTS_FILE = os.path.join(TASKS_DIR, 'robot_points.txt')   # named points of the action files
NEAR_DEG = 15.0                  # "already on the path": every joint within this of a path point (skips 'entry')
ENTRY_TOL = 3.0                  # entry path resumed from where the robot is: on a segment / at a point within this [deg]
EXIT_VIA = [('pos2', 'pos1', 8.0),   # storage exit: robot within 8 deg (every joint) of the line pre-grasp - grasp of a
            ('pos18', 'pos17', 8.0)]  # box (tool inside it; box 2: pos1 - pos2, box 1: pos17 - pos18) -> first straight
                                      # up to that pre-grasp, whatever the action, so it never leaves a box sideways (not
                                      # when the action itself starts at that pre-grasp or grasp)

SPEEDS = {'slow': 5, 'medium': 15, 'normal': 23, 'fast': 30}   # normal 23 (was 15, 2026-10-09)   # joint speed [deg/s]; the gateway caps 60 deg/s
                                                       # slow 5 deg/s: tool <= ~4 cm/s at 0.45 m reach, so it travels
                                                       # < ~3.5 cm in the ~0.85 s reaction chain (prohibited zone 5 cm)
ACCELERATION = 200                                     # joint acceleration [deg/s^2]
MAX_SPEED = 60                                         # never command more [deg/s]

POLL = 0.1                       # how often actions.md and the arm are checked [s]
REACHED_TOL = 0.5                # a position counts as reached within this [deg]
RUN_TASK_AT_START = False        # True: start with the task already in actions.md (False: an old number left in
                                 # the file from the last session is NOT run - only a NEW number written by MATLAB)
SPEED_CAP = 30.0                 # every action at most this fast [deg/s] (30 = fast: retreat; normal 15, slow 5). None: file speeds
KEY_STOP = True                  # Windows: ANY KEY stops the robot and pauses; ENTER follows actions.md again
RESUME_WINDOW_S = 600            # at start: an action written less than this ago may be executed (ENTER)
INTERRUPT_ON_NEW_TASK = True     # True: a new task stops the motion at once;
                                 # False: the current move to a position is finished first
RELAY_RENEW_BEFORE = 300         # renew the relay access this long before it expires [s]
DRY_RUN = False                  # True: no robot, only print what would be done
SEGMENT_S = 0.4                  # speed factor < 1 (solo, fatigue / distance): the move is done in pieces of this
                                 # duration, each at the factor read just before it -> the speed follows MATLAB
MIN_SCALE = 0.05                 # factor below this: the robot holds (MATLAB normally sends a protective stop)
MIN_SPEED = 0.5                  # never command a piece slower than this [deg/s]
BLEND = True                     # pieces are blended (MoveArcJoint, radius <= 30 % of the piece, <= BLEND_MAX): no stop
BLEND_MAX = 3.0                  # between pieces -> constant speed along the path; refused once -> unblended from then on
ALWAYS_PIECES = True             # every move in pieces of SEGMENT_S (queued one ahead): if the connection to the arm
                                 # is lost, the arm runs on for at most about one piece instead of the whole move
# =========================================================================

sys.path.insert(0, HERE)


LOG_FILE = os.path.join(HERE, 'robot_code_log.txt')   # every message of the console, with the date


def log(msg):
    print('[%s] %s' % (time.strftime('%H:%M:%S'), msg), flush=True)
    try:
        with open(LOG_FILE, 'a', encoding='utf-8') as f:
            f.write('[%s] %s\n' % (time.strftime('%Y-%m-%d %H:%M:%S'), msg))
    except OSError:
        pass


# --------------------------------------------------------------------------- files

def read_action():
    """Task number in actions.md, or None if the file is missing, empty or being written."""
    try:
        with open(ACTIONS_FILE, 'r', encoding='utf-8-sig') as f:
            text = f.read()
    except OSError:
        return None
    m = re.search(r'-?\d+', text)
    return int(m.group()) if m else None


def read_speed_scale():
    """Speed factor 0..1 from the line 'speed_scale=..' of actions.md (MATLAB, solo fatigue / distance); 1 if none."""
    try:
        with open(ACTIONS_FILE, 'r', encoding='utf-8-sig') as f:
            text = f.read()
    except OSError:
        return 1.0
    m = re.search(r'speed_scale\s*=\s*([0-9.]+)', text)
    try:
        return max(0.0, min(1.0, float(m.group(1)))) if m else 1.0
    except ValueError:
        return 1.0


def read_task(number):
    """dict(speed [deg/s], positions [[6 angles]], loop bool, file) or raises ValueError."""
    return read_task_file(os.path.join(TASKS_DIR, TASK_FILE.format(number)))


def read_points(path=None):
    """name -> 6 joint angles [deg] from robot_points.txt (name:[j1..j6] lines)."""
    path = path or POINTS_FILE
    try:
        with open(path, 'r', encoding='utf-8-sig') as f:
            lines = f.read().splitlines()
    except OSError:
        raise ValueError('points file not found: %s' % path)
    pts = {}
    for n, line in enumerate(lines, 1):
        line = line.split('#')[0].strip()
        m = re.match(r'(\w+)\s*:\s*\[(.*)\]\s*$', line)
        if not m:
            continue
        nums = re.findall(r'-?\d+(?:\.\d+)?', m.group(2))
        if len(nums) < 6:
            raise ValueError('%s line %d: %s needs 6 joint angles' % (path, n, m.group(1)))
        pts[m.group(1).lower()] = [float(x) for x in nums[:6]]
    return pts


def read_task_file(path):
    """Same as read_task, for any file. A position has 6 joint angles [deg], optionally followed by the tool
    x, y, z [mm] recorded with it (kept in 'xyz', not used for the motion). Action files (path= / entry= / wait= /
    stop=) take their points from robot_points.txt."""
    try:
        with open(path, 'r', encoding='utf-8-sig') as f:
            lines = f.read().splitlines()
    except OSError:
        raise ValueError('task file not found: %s' % path)
    speed, loop, positions, xyz = SPEEDS['slow'], False, [], {}
    path_names, entry_names, waits, stop, title = None, [], {}, False, ''
    for line in lines:                                    # first comment line = the action's name
        if line.startswith('#'):
            title = line.lstrip('# ').strip()
            break
    for n, line in enumerate(lines, 1):
        line = line.split('#')[0].strip()
        if not line:
            continue
        m = re.match(r'(\w+)\s*[=:]\s*(.*)$', line)
        if not m:
            raise ValueError('%s line %d: expected "name=value" or "name:value": %s' % (path, n, line))
        key, value = m.group(1).lower(), m.group(2).strip()
        if key == 'speed':
            if value.lower() in SPEEDS:
                speed = SPEEDS[value.lower()]
            else:
                try:
                    speed = float(value)
                except ValueError:
                    raise ValueError('%s line %d: speed must be %s or a number' % (path, n, '/'.join(SPEEDS)))
        elif key == 'loop':
            if value.lower() not in ('true', 'false', '1', '0'):
                raise ValueError('%s line %d: loop must be true or false' % (path, n))
            loop = value.lower() in ('true', '1')
        elif key in ('path', 'entry'):
            names = [x.strip().lower() for x in value.split(',') if x.strip()]
            if key == 'path':
                path_names = names
            else:
                entry_names = names
        elif key == 'wait':
            for item in value.split(','):
                name, _, sec = item.partition(':')
                try:
                    waits[name.strip().lower()] = float(sec)
                except ValueError:
                    raise ValueError('%s line %d: wait must be point:seconds, e.g. pos12:30' % (path, n))
        elif key == 'stop':
            stop = value.lower() in ('true', '1')
        elif key == 'action':
            title = value
        elif re.fullmatch(r'pos\d+', key):
            nums = re.findall(r'-?\d+(?:\.\d+)?', value)
            if len(nums) not in (6, 9):
                raise ValueError('%s line %d: %s needs 6 joint angles (+ optional x, y, z), got %d numbers'
                                 % (path, n, key, len(nums)))
            positions.append((int(key[3:]), [float(x) for x in nums[:6]]))
            if len(nums) == 9:
                xyz[int(key[3:])] = [float(x) for x in nums[6:]]
        else:
            raise ValueError('%s line %d: unknown entry "%s"' % (path, n, key))
    if not 0 < speed <= MAX_SPEED:
        raise ValueError('%s: speed %g deg/s is outside 0..%g' % (path, speed, MAX_SPEED))
    base = {'speed': speed, 'loop': loop, 'file': path, 'title': title, 'stop': stop, 'waits': waits,
            'entry': [], 'entry_names': []}
    if stop:
        return dict(base, positions=[], names=[], xyz=[])
    if path_names is not None:                            # action file: points from robot_points.txt
        pts = read_points()
        missing = [x for x in path_names + entry_names if x not in pts]
        if missing:
            raise ValueError('%s: unknown points %s (not in %s)' % (path, ', '.join(missing), POINTS_FILE))
        if not path_names:
            raise ValueError('%s: empty path' % path)
        return dict(base, positions=[pts[x] for x in path_names], names=path_names, xyz=[None] * len(path_names),
                    entry=[pts[x] for x in entry_names], entry_names=entry_names)
    if not positions:
        raise ValueError('%s: no positions (pos0:[...] or path=...)' % path)
    positions.sort(key=lambda p: p[0])
    return dict(base, positions=[p[1] for p in positions], names=['pos%d' % p[0] for p in positions],
                xyz=[xyz.get(p[0]) for p in positions])


# --------------------------------------------------------------------------- gateway relay

class Relay(object):
    """SDK access for this PC through the Jetson gateway, renewed before it expires."""

    def __init__(self):
        import lite6
        self.lite6 = lite6
        try:
            self.gw = lite6.connect(quiet=True)
        except lite6.Lite6Error as e:
            sys.exit('%s\nLog in once on this computer:  python robot_code/lite6.py login 10.13.37.101' % e)
        self.host = self.gw.host
        self.until = 0
        self.renew()

    def renew(self):
        try:
            r = self.gw.relay_allow()
        except self.lite6.Lite6Error as e:
            sys.exit('the gateway refused SDK relay access: %s\n'
                     '(check `python robot_code/lite6.py status`: the relay must be enabled and not halted)' % e)
        self.until = r.get('until') or time.time() + 600
        log('relay open for this PC (%s) until %s, ports %s' % (
            r.get('allowed_ip'), time.strftime('%H:%M:%S', time.localtime(self.until)), r.get('ports')))

    def keep_alive(self):
        if time.time() > self.until - RELAY_RENEW_BEFORE:
            self.renew()

    def close(self):
        try:
            self.gw.relay_revoke()
            log('relay access removed')
        except Exception:
            pass


# --------------------------------------------------------------------------- robot

class TaskChanged(Exception):
    pass


class ConnectionLost(Exception):
    """The connection to the arm (through the gateway relay) is gone: the SDK answers 'not connected'."""


def _connected_reply(r):
    """True when an SDK read reply is (code, data) with a usable code (0 ok, 1 / 2 controller error / warning flag)."""
    return isinstance(r, tuple) and len(r) == 2 and r[0] in (0, 1, 2)


class Robot(object):
    def __init__(self, host):
        self.host = host
        try:
            self._connect()
        except ConnectionLost as e:
            sys.exit('could not connect to the arm through %s: %s' % (host, e))
        log('arm ready, joints %s' % self.angles())

    def _connect(self):
        from xarm.wrapper import XArmAPI
        log('connecting to the arm at %s ...' % self.host)
        self.arm = XArmAPI(self.host, is_radian=False)
        if not self.arm.connected:
            raise ConnectionLost('no connection')
        r = self.arm.get_err_warn_code()
        if not _connected_reply(r):
            raise ConnectionLost('no answer from the controller (%r)' % (r,))
        err, warn = r[1][0], r[1][1]
        if err or warn:
            log('controller error %s / warning %s at start: clearing it' % (err, warn))
            self.arm.clean_warn()
            self.arm.clean_error()
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)
        self.arm.set_state(0)
        time.sleep(0.5)
        self.check()

    def reconnect(self):
        """After a ConnectionLost: try again every 2 s until the arm answers; the arm is stopped and made ready."""
        n = 0
        while True:
            n += 1
            try:
                try:
                    self.arm.disconnect()
                except Exception:
                    pass
                self._connect()
                self.stop()
                log('connection to the arm restored (attempt %d), joints %s' % (n, self.angles()))
                return
            except ConnectionLost as e:
                if n == 1 or n % 10 == 0:
                    log('  still no connection to the arm (%s) - retrying every 2 s (Ctrl+C ends)' % e)
                time.sleep(2.0)

    def angles(self):
        r = self.arm.get_servo_angle()
        if not _connected_reply(r):
            if not self.arm.connected or (isinstance(r, tuple) and r[0] == 3):
                raise ConnectionLost('get_servo_angle answered %r' % (r,))
            return None
        return [round(x, 1) for x in r[1][:6]]

    def state(self):
        """Controller state (1 = moving); ConnectionLost when the arm does not answer."""
        r = self.arm.get_state()
        if not _connected_reply(r) or not isinstance(r[1], int):
            raise ConnectionLost('get_state answered %r' % (r,))
        return r[1]

    def check(self):
        """Exit on a controller error (collision, limit, E-stop ...): a person must look first.
        ConnectionLost when the arm does not answer (network / relay)."""
        r = self.arm.get_err_warn_code()
        if not _connected_reply(r) or not isinstance(r[1], (list, tuple)) or len(r[1]) < 2:
            raise ConnectionLost('get_err_warn_code answered %r' % (r,))
        err, warn = r[1][0], r[1][1]
        if err:
            self.stop()
            raise SystemExit('ROBOT ERROR %s: the arm stopped. Check the robot and the area, '
                             'then start this program again.' % err)
        if warn:
            log('controller warning %s' % warn)

    def servo(self, target, speed, blend=None):
        """set_servo_angle (no wait). An unexpected exception inside the SDK (seen 2026-10-09 after a protective
        stop: 'too many values to unpack') is logged with its traceback; the arm is made ready again (mode 0,
        state 0 - like after a stop) and the command is sent once more. A second failure ends the task."""
        if blend is not None and BLEND and getattr(self, 'blend_ok', True):
            try:
                code = self.arm.set_servo_angle(angle=target, speed=speed, mvacc=ACCELERATION, wait=False,
                                                radius=float(blend))
            except Exception as e:
                code = 'exception %s' % e
            if code in (0, 1, 2):
                return code
            if code == 3 or not self.arm.connected:
                raise ConnectionLost('set_servo_angle answered %r' % (code,))
            self.blend_ok = False
            log('blended move refused (%s): moving WITHOUT blending from now on' % (code,))
        try:
            code = self.arm.set_servo_angle(angle=target, speed=speed, mvacc=ACCELERATION, wait=False)
        except Exception:
            log('SDK exception in set_servo_angle - making the arm ready and sending the command again:\n%s'
                % traceback.format_exc())
            self.check()                                  # ConnectionLost if the arm does not answer
            self.arm.set_mode(0)
            self.arm.set_state(0)
            time.sleep(0.5)
            code = self.arm.set_servo_angle(angle=target, speed=speed, mvacc=ACCELERATION, wait=False)
        if code == 3 or not self.arm.connected:
            raise ConnectionLost('set_servo_angle answered %r' % (code,))
        return code

    def stop(self):
        """Stop the current motion and make the arm ready for the next command."""
        try:
            self.arm.set_state(4)
            time.sleep(0.3)
            self.arm.set_mode(0)
            self.arm.set_state(0)
            time.sleep(0.3)
        except Exception as e:                            # no connection: nothing reaches the arm
            log('stop: %s' % e)

    def move(self, target, speed, changed, scale_fn=None):
        """Joint move to target; checks changed() every POLL s and raises TaskChanged. scale_fn: speed factor 0..1
        (read every POLL s); below 1 the move continues in pieces at the reduced speed (move_scaled)."""
        if ALWAYS_PIECES or (scale_fn is not None and scale_fn() < 0.999):
            return self.move_scaled(target, speed, changed, scale_fn or (lambda: 1.0))
        start = self.angles() or target
        dist = max(abs(a - b) for a, b in zip(start, target))
        timeout = 15 + 3 * dist / speed
        code = self.servo(target, speed)
        if code == 10:
            raise SystemExit('the gateway guard refused the motion command (code 10)')
        if code not in (0, 1, 2):
            self.check()
            raise SystemExit('set_servo_angle failed with code %s' % code)
        t0, idle_since = time.time(), None
        while True:
            time.sleep(POLL)
            if INTERRUPT_ON_NEW_TASK and changed():
                self.stop()
                raise TaskChanged()
            self.check()
            if scale_fn is not None and scale_fn() < 0.999:     # MATLAB slows the robot: continue in pieces
                self.stop()
                return self.move_scaled(target, speed, changed, scale_fn)
            now = self.angles()
            state = self.state()
            if now and max(abs(a - b) for a, b in zip(now, target)) < REACHED_TOL and state != 1:
                return
            if state != 1:                         # not moving any more but not at the target
                idle_since = idle_since or time.time()
                if time.time() - idle_since > 2:
                    log('warning: the arm stopped at %s, short of %s' % (now, target))
                    return
            else:
                idle_since = None
            if time.time() - t0 > timeout:
                self.stop()
                raise SystemExit('the move to %s did not finish in %.0f s' % (target, timeout))

    def move_scaled(self, target, speed, changed, scale_fn):
        """Joint move to target at speed x scale_fn(): pieces of SEGMENT_S, each commanded at the factor read just
        before it (the next piece is queued shortly before the current one ends, so the motion stays smooth).
        Factor < MIN_SCALE: the robot stops and holds until the factor is back. Raises TaskChanged."""
        held, logged = False, None
        while True:
            if INTERRUPT_ON_NEW_TASK and changed():
                self.stop()
                raise TaskChanged()
            self.check()
            k = scale_fn()
            now = self.angles()
            if not now:
                time.sleep(POLL)
                continue
            rem = [b - a for a, b in zip(now, target)]
            dmax = max(abs(x) for x in rem)
            if dmax < REACHED_TOL:
                if self.state() != 1:                     # at the target and standing still
                    return
                time.sleep(POLL)
                continue
            if k < MIN_SCALE:
                if not held:
                    self.stop()
                    held = True
                    log('  speed factor %.2f: the robot holds' % k)
                time.sleep(POLL)
                continue
            held = False
            v = max(MIN_SPEED, speed * k)
            if logged is None or abs(k - logged) >= 0.05:
                log('  speed factor %.2f -> %.1f deg/s' % (k, v))
                logged = k
            f = min(1.0, v * SEGMENT_S / dmax)
            sub = [a + f * r for a, r in zip(now, rem)]
            code = self.servo(sub, v, blend=None if f >= 1.0 else min(BLEND_MAX, 0.3 * f * dmax))
            if code == 10:
                raise SystemExit('the gateway guard refused the motion command (code 10)')
            if code not in (0, 1, 2):
                self.check()
                raise SystemExit('set_servo_angle failed with code %s' % code)
            t0 = time.time()
            while True:                               # until this piece is almost done
                time.sleep(POLL)
                if INTERRUPT_ON_NEW_TASK and changed():
                    self.stop()
                    raise TaskChanged()
                self.check()
                if scale_fn() < MIN_SCALE:
                    break
                q = self.angles()
                state = self.state()
                left = max(abs(a - b) for a, b in zip(q, sub)) if q else 0.0
                if state != 1 or left <= max(REACHED_TOL, v * 1.5 * POLL):
                    break
                if time.time() - t0 > 3 * SEGMENT_S + 5:
                    self.stop()
                    raise SystemExit('a reduced-speed piece of the move to %s did not finish' % target)

    def wait(self, seconds, changed):
        """Stand still for seconds; checks changed() every POLL s and raises TaskChanged."""
        t0 = time.time()
        while time.time() - t0 < seconds:
            time.sleep(POLL)
            if changed():
                raise TaskChanged()
            self.check()

    def near(self, positions, tol=NEAR_DEG):
        """True when every joint is within tol of one of the positions."""
        now = self.angles()
        return bool(now) and any(max(abs(a - b) for a, b in zip(now, p)) <= tol for p in positions)

    def close(self):
        try:
            self.arm.set_state(4)
            self.arm.disconnect()
        except Exception:
            pass


class DryRobot(object):
    """DRY_RUN: no arm, the moves are only printed and take the time they would take."""

    def __init__(self, host):
        self.pos = [0.0] * 6

    def move(self, target, speed, changed, scale_fn=None):
        k = scale_fn() if scale_fn is not None else 1.0
        if k < 0.999:
            log('  (dry run) speed factor %.2f' % k)
        t = max(abs(a - b) for a, b in zip(self.pos, target)) / max(MIN_SPEED, speed * max(k, MIN_SCALE))
        t0 = time.time()
        while time.time() - t0 < t:
            time.sleep(POLL)
            if INTERRUPT_ON_NEW_TASK and changed():
                raise TaskChanged()
        self.pos = list(target)

    def stop(self):
        pass

    def angles(self):
        return list(self.pos)

    def wait(self, seconds, changed):
        t0 = time.time()
        while time.time() - t0 < seconds:
            time.sleep(POLL)
            if changed():
                raise TaskChanged()

    def near(self, positions, tol=NEAR_DEG):
        return any(max(abs(a - b) for a, b in zip(self.pos, p)) <= tol for p in positions)

    def close(self):
        pass


def near_path(now, positions, loop, tol=NEAR_DEG):
    """True when the joints 'now' are within tol (every joint) of the path: of a path point OR of any point on the
    joint-space moves between consecutive path points (and back from the last to the first when the path loops).
    A robot stopped halfway between pos12 and pos13 is on the path, so the entry points are not run again."""
    if not now:
        return False
    pts = list(positions)
    segs = list(zip(pts, pts[1:])) + ([(pts[-1], pts[0])] if loop and len(pts) > 1 else [])
    segs = segs or [(pts[0], pts[0])]
    for a, b in segs:
        for k in range(21):
            f = k / 20.0
            if max(abs(n - (x + f * (y - x))) for n, x, y in zip(now, a, b)) <= tol:
                return True
    return False


def wrist_flip(q, near=None):
    """The same tool pose with the wrist turned over: (J4 +/- 180, -J5, J6 +/- 180), J4 / J6 within +/-360 deg; of the
    candidates the one whose wrist is closest to 'near' (default: to q itself)."""
    near = near or q
    cands = [[q[0], q[1], q[2], q[3] + s4, -q[4], q[5] + s6] for s4 in (-180, 180) for s6 in (-180, 180)
             if abs(q[3] + s4) <= 360 and abs(q[5] + s6) <= 360]
    return min(cands, key=lambda c: max(abs(x - y) for x, y in zip(c[3:], near[3:])))


def wrist_hints(names, positions, min_saving=90.0):
    """Consecutive points whose wrist (J4..J6) turns much less with the other wrist configuration of the second point
    (same tool pose): [(from, to, turn now, turn flipped, flipped joints)]. Helps when points are recorded by hand."""
    out = []
    for (na, a), (nb, b) in zip(zip(names, positions), list(zip(names, positions))[1:]):
        now = max(abs(y - x) for x, y in zip(a[3:], b[3:]))
        fb = wrist_flip(b, near=a)
        flipped = max(abs(y - x) for x, y in zip(a[3:], fb[3:]))
        if now - flipped >= min_saving:
            out.append((na, nb, now, flipped, [round(v, 1) for v in fb]))
    return out


def entry_moves(now, task):
    """[(name, joints)] of the entry points still to visit: none when the robot is already on the path; otherwise
    from the entry point the robot is at or moving to (a robot stopped at pos10 of pos9 -> pos10 -> pos11 goes on
    to pos11 instead of going back to pos9)."""
    if not task['entry'] or (now and near_path(now, task['positions'], task['loop'])):
        return []
    start = 0
    if now:
        for k in range(1, len(task['entry'])):
            if near_path(now, [task['entry'][k - 1], task['entry'][k]], False, ENTRY_TOL):
                start = k + 1 if near_path(now, [task['entry'][k]], False, ENTRY_TOL) else k
        start = min(start, len(task['entry']))
    return list(zip(task['entry_names'], task['entry']))[start:]


def path_start(now, task):
    """Index of the first path point still to visit of a NON-looping path: a robot already on the path (e.g. a
    retreat stopped 2 cm before its end) goes on from there instead of starting again at the first point."""
    if task['loop'] or not now or not task['positions']:
        return 0
    pts, start = task['positions'], 0
    for k in range(1, len(pts)):
        if near_path(now, [pts[k - 1], pts[k]], False, ENTRY_TOL):
            start = k + 1 if near_path(now, [pts[k]], False, ENTRY_TOL) else k
    return start


def exit_moves(now, task):
    """[(name, joints)] the robot must do before this task: the EXIT_VIA rule (robot inside the storage box)."""
    if not now or task['stop']:
        return []
    first = (task['entry_names'] if task['entry'] and not near_path(now, task['positions'], task['loop'])
             else task['names'])[:1]
    pts = read_points()
    for inside, via, tol in EXIT_VIA:
        if inside in pts and via in pts and first and first[0] not in (inside, via) \
                and near_path(now, [pts[via], pts[inside]], False, tol):
            return [(via, pts[via])]
    return []


def run_task(robot, task, changed, label, speed_cap=None, keep_alive=None, scale_fn=None):
    """Execute one action file. stop -> stop and hold; else entry (only when not already on the path), then the
    path with its waits, repeated while loop is true. Raises TaskChanged when changed() becomes true."""
    if task['stop']:
        robot.stop()
        log('%s: STOP - the robot holds its position' % label)
        return
    speed = min(task['speed'], speed_cap) if speed_cap else task['speed']
    for name, pos in exit_moves(robot.angles(), task):   # in the storage box: straight up first
        log('  %s storage exit -> %s %s' % (label, name, pos))
        robot.move(pos, speed, changed, scale_fn)
        if changed():
            raise TaskChanged()
    if task['entry']:
        for name, pos in entry_moves(robot.angles(), task):
            log('  %s entry -> %s %s' % (label, name, pos))
            robot.move(pos, speed, changed, scale_fn)
            if changed():
                raise TaskChanged()
    cycle = 0
    first = path_start(robot.angles(), task)             # non-looping path: go on from where the robot is
    if first:
        log('  %s: already on the path, continuing at %s' % (label, task['names'][first] if first < len(task['names'])
                                                            else 'its end (nothing left to do)'))
    while True:
        cycle += 1
        for i, (name, pos) in enumerate(zip(task['names'], task['positions'])):
            if cycle == 1 and i < first:
                continue
            log('  %s cycle %d -> %s %s at %g deg/s' % (label, cycle, name, pos, speed))
            robot.move(pos, speed, changed, scale_fn)
            if changed():                                 # also when INTERRUPT_ON_NEW_TASK is False
                raise TaskChanged()
            if name in task['waits']:
                log('  %s waits %.0f s at %s' % (label, task['waits'][name], name))
                robot.wait(task['waits'][name], changed)
        if not task['loop']:
            log('%s done' % label)
            return
        if keep_alive:
            keep_alive()


# --------------------------------------------------------------------------- main loop

try:
    import msvcrt                                         # Windows keyboard without Enter
except ImportError:                                       # pragma: no cover
    msvcrt = None


def key_hit():
    """True if a key was pressed (all waiting keys are consumed)."""
    if not KEY_STOP or msvcrt is None:
        return False
    hit = False
    while msvcrt.kbhit():
        msvcrt.getwch()
        hit = True
    return hit


def pause_until_enter(robot):
    """Key stop: the robot is stopped and holds; ENTER resumes following actions.md."""
    robot.stop()
    log('*** STOPPED by key - the robot holds. Press ENTER to follow actions.md again (Ctrl+C ends) ***')
    while True:
        ch = msvcrt.getwch()
        if ch in ('\r', '\n'):
            log('resumed: executing the action now in actions.md, then following it')
            return
        if ch == '\x03':
            raise KeyboardInterrupt


def main():
    log('actions file: %s' % ACTIONS_FILE)
    log('speed cap: %s | key stop: %s' % ('%g deg/s' % SPEED_CAP if SPEED_CAP else 'none (file speeds)',
                                          'ANY KEY' if KEY_STOP and msvcrt else 'off'))
    log('tasks folder: %s' % TASKS_DIR)
    relay = None if DRY_RUN else Relay()
    robot = DryRobot(None) if DRY_RUN else Robot(relay.host)
    current = read_action()
    pending = current if RUN_TASK_AT_START else None
    log('actions.md = %s; waiting for changes (Ctrl+C to stop)' % current)
    try:
        age = time.time() - os.path.getmtime(ACTIONS_FILE)
    except OSError:
        age = None
    if pending is None and current is not None and age is not None and age < RESUME_WINDOW_S and KEY_STOP and msvcrt:
        log('actions.md = %s was written %.0f s ago: a session seems to be running.' % (current, age))
        log('   ENTER = execute action %s NOW      any other key = wait for the next NEW action' % current)
        log('   (no answer needed: as soon as MATLAB writes a new action, it is followed)')
        mtime0 = os.path.getmtime(ACTIONS_FILE)
        while True:                                   # never blocks: a new action from MATLAB ends the question
            if msvcrt.kbhit():
                ch = msvcrt.getwch()
                if ch in ('\r', '\n'):
                    pending = current
                    log('executing the current action %s' % current)
                else:
                    log('waiting for the next new action from MATLAB')
                break
            try:
                new_mtime = os.path.getmtime(ACTIONS_FILE)
            except OSError:
                new_mtime = mtime0
            if new_mtime != mtime0:
                a = read_action()
                if a is not None:
                    pending = a
                    log('MATLAB wrote a new action (%s): following it' % a)
                    break
            time.sleep(POLL)

    stopped_by_key = [False]

    def changed():
        """True when actions.md holds a different task number than the one running, or a key was pressed."""
        if key_hit():
            stopped_by_key[0] = True
            return True
        a = read_action()
        return a is not None and a != current

    try:
        while True:
            if relay:
                relay.keep_alive()
            if pending is None:                      # idle: wait for a new task number
                time.sleep(POLL)
                if changed():
                    if stopped_by_key[0]:
                        stopped_by_key[0] = False
                        pause_until_enter(robot)
                        pending = read_action()          # ENTER: execute MATLAB's current decision
                        continue
                    pending = read_action()
                continue
            current, pending = pending, None
            try:
                task = read_task(current)
            except ValueError as e:
                log('task %s NOT run: %s' % (current, e))
                continue
            log('task %s (%s): %s' % (current, task['title'], 'STOP' if task['stop'] else
                                       '%d points, %g deg/s, loop %s' % (len(task['positions']), task['speed'], task['loop'])))
            try:
                run_task(robot, task, changed, 'task %s' % current, speed_cap=SPEED_CAP, scale_fn=read_speed_scale,
                         keep_alive=relay.keep_alive if relay else None)
                log('task %s finished; waiting for a new task' % current)
            except TaskChanged:
                if stopped_by_key[0]:
                    stopped_by_key[0] = False
                    pause_until_enter(robot)
                    pending = read_action()              # ENTER: execute MATLAB's current decision
                    continue
                pending = read_action()
                log('actions.md changed: task %s -> %s' % (current, pending))
            except ConnectionLost as e:
                log('*** CONNECTION TO THE ARM LOST during task %s (%s): the arm stops by itself within ~0.5 s '
                    '(moves are sent in short pieces). Reconnecting ...' % (current, e))
                robot.reconnect()
                pending = read_action()                  # then follow MATLAB's current decision
            except ValueError as e:
                log('task %s stopped: %s\n%s' % (current, e, traceback.format_exc()))
    except KeyboardInterrupt:
        log('Ctrl+C: stopping the arm')
    finally:
        robot.close()
        if relay:
            relay.close()


if __name__ == '__main__':
    main()
