# -*- coding: utf-8 -*-
"""
TEST THE ROBOT ACTIONS one by one (the action files tasks/task0.txt ... task11.txt).

    python robot_code/test_actions.py                 every action at SLOW speed (5 deg/s) - start with this
    python robot_code/test_actions.py --speed file    each action at the speed written in its file

  1. type the action number (0 ... 11) and press ENTER: the plan is shown (points, tool positions, speed)
  2. press ENTER again to move the robot (any other key cancels)
  3. ANY KEY STOPS THE ROBOT IMMEDIATELY while it moves or waits
     (looping actions 1, 2, 5, 6 run until you press a key)
  Other commands:  p = robot position now,  m = list of actions,  q = quit (the robot is stopped)

Also available at any time: the STOP button on the gateway web page, or  python robot_code/lite6.py estop
The same files are used by robot_code.py when MATLAB writes the action number into actions.md.
"""
import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import robot_code as rc                                    # noqa: E402

ACTIONS = range(12)
try:
    import msvcrt                                          # Windows: keys without Enter -> stop at once
except ImportError:                                        # pragma: no cover
    msvcrt = None


def key_pressed():
    """True if a key was pressed since the last call (all waiting keys are consumed)."""
    if msvcrt is None:
        return False
    hit = False
    while msvcrt.kbhit():
        msvcrt.getwch()
        hit = True
    return hit


def read_line(prompt):
    """A line from the keyboard (Windows: character by character so that nothing is left for key_pressed)."""
    if msvcrt is None:
        return input(prompt)
    sys.stdout.write(prompt)
    sys.stdout.flush()
    buf = ''
    while True:
        ch = msvcrt.getwch()
        if ch in ('\r', '\n'):
            sys.stdout.write('\n')
            return buf
        if ch == '\x03':
            raise KeyboardInterrupt
        if ch == '\b':
            if buf:
                buf = buf[:-1]
                sys.stdout.write('\b \b')
        elif ch.isprintable():
            buf += ch
            sys.stdout.write(ch)
        sys.stdout.flush()


def wait_key():
    """The next key (Windows) - ENTER returns '\\r'."""
    if msvcrt is None:
        return '\r' if input() == '' else 'x'
    return msvcrt.getwch()


def tool_xyz(robot, q):
    try:
        code, pose = robot.arm.get_forward_kinematics(q, input_is_radian=False, return_is_radian=False)
        return '[%4.0f %4.0f %4.0f]' % tuple(pose[:3]) if code == 0 else '?'
    except Exception:                                      # noqa: BLE001
        return '?'


def show_menu():
    print('\nACTIONS (tasks/task<N>.txt):')
    for n in ACTIONS:
        try:
            t = rc.read_task(n)
            what = 'STOP and hold' if t['stop'] else '%s%s%s' % (
                ('entry ' + ','.join(t['entry_names']) + ' | ') if t['entry'] else '', ' -> '.join(t['names']),
                '  (repeats)' if t['loop'] else '')
            print('  %2d  %-42s %s' % (n, t['title'].split(':', 1)[-1].strip()[:42], what))
        except ValueError as e:
            print('  %2d  ERROR: %s' % (n, e))
    print('  p = position now   m = this list   q = quit')


def show_plan(robot, n, task, speed):
    print('\nACTION %d - %s' % (n, task['title']))
    if task['stop']:
        print('  the robot stops where it is and holds (no motion)')
        return
    for name, q in rc.exit_moves(robot.angles(), task):
        print('  storage exit FIRST (the robot is in the box): %-6s tool %s mm' % (name, tool_xyz(robot, q)))
    entry = rc.entry_moves(robot.angles(), task) if task['entry'] else []
    if task['entry']:
        print('  entry points %s' % ('USED (the robot is not on the path yet):' if entry else
                                     'skipped (the robot is already on the path)'))
        for name, q in entry:
            print('     %-6s tool %s mm' % (name, tool_xyz(robot, q)))
    print('  path%s:' % (' (repeats until you press a key)' if task['loop'] else ''))
    first = rc.path_start(robot.angles(), task)
    for i, (name, q) in enumerate(zip(task['names'], task['positions'])):
        if i < first:
            print('     %-6s (already passed: the robot is further along this path)' % name)
            continue
        print('     %-6s tool %s mm%s' % (name, tool_xyz(robot, q),
                                         '   then waits %.0f s' % task['waits'][name] if name in task['waits'] else ''))
    print('  speed %g deg/s%s' % (speed, '' if speed == task['speed'] else '  (TEST: file says %g)' % task['speed']))
    seq_n = task['names'] + (task['names'][:1] if task['loop'] else [])
    seq_q = task['positions'] + (task['positions'][:1] if task['loop'] else [])
    for a, b, now, flipped, fq in rc.wrist_hints(seq_n, seq_q):
        print('  NOTE %s -> %s turns the wrist %.0f deg; the same tool pose with the wrist turned over needs only %.0f deg:'
              ' %s = %s in tasks/robot_points.txt' % (a, b, now, flipped, b, fq))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--speed', default='slow',
                    help="slow (default, 5 deg/s for every action), a number in deg/s, or 'file' (speed of each file)")
    args = ap.parse_args()
    cap = None if args.speed == 'file' else (rc.SPEEDS[args.speed] if args.speed in rc.SPEEDS else float(args.speed))
    if msvcrt is None:
        print('WARNING: key-stop needs Windows (msvcrt); use the gateway STOP button')
    print(__doc__)
    relay = rc.Relay()
    robot = rc.Robot(relay.host)
    show_menu()
    try:
        while True:
            relay.keep_alive()
            cmd = read_line('\naction number + ENTER (p / m / q): ').strip().lower()
            if cmd in ('q', 'quit', 'exit'):
                break
            if cmd == 'm':
                show_menu()
                continue
            if cmd == 'p':
                q = robot.angles()
                print('  joints %s  tool %s mm' % (q, tool_xyz(robot, q) if q else '?'))
                continue
            if not cmd.isdigit() or int(cmd) not in ACTIONS:
                print('  type a number 0 ... 11')
                continue
            n = int(cmd)
            try:
                task = rc.read_task(n)
            except ValueError as e:
                print('  cannot use task%d.txt: %s' % (n, e))
                continue
            speed = min(task['speed'], cap) if (cap and not task['stop']) else task['speed']
            show_plan(robot, n, task, speed)
            print('\n  Area clear?  ENTER = start   any other key = cancel   (while it runs: ANY KEY = STOP)')
            if wait_key() not in ('\r', '\n'):
                print('  cancelled - the robot did not move')
                continue
            key_pressed()                                  # forget earlier keys
            t0 = time.time()
            try:
                rc.run_task(robot, task, key_pressed, 'action %d' % n, speed_cap=cap, keep_alive=relay.keep_alive)
                print('  action %d finished in %.0f s' % (n, time.time() - t0))
            except rc.TaskChanged:
                robot.stop()
                print('  *** STOPPED by key after %.0f s - joints %s ***' % (time.time() - t0, robot.angles()))
            except rc.ConnectionLost as e:
                print('  *** CONNECTION TO THE ARM LOST (%s): the arm stops by itself within ~0.5 s. Reconnecting ...' % e)
                robot.reconnect()
    except KeyboardInterrupt:
        print('\nCtrl+C')
    finally:
        print('stopping the robot and closing')
        robot.close()
        relay.close()


if __name__ == '__main__':
    main()
