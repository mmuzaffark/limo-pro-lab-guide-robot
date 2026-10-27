#!/usr/bin/env python3
# lab_gui_win.py [ROBOT_IP] [--test-voice] : lab guide GUI on Windows (no ROS on the laptop).
# Talks to the robot's rosbridge (TCP 9090) with roslibpy: sends std_msgs/String on /guide/command,
# shows /guide/status (JSON from guide_manager_foxy.py). Voice: 5 s mic -> faster-whisper -> place name.
# Speech out: Windows SAPI via PowerShell. Places: %USERPROFILE%\lab\places_lab.yaml (copied by start_gui.ps1).
# Demo layout (guide 5.11): four big buttons Home, Auditorium, Library, Cafeteria (names = keys in places_lab.yaml).
import json, os, queue, subprocess, sys, threading, time, tkinter as tk
from tkinter import messagebox
import yaml
from rapidfuzz import process, fuzz

LAB = os.path.join(os.path.expanduser('~'), 'lab')
PORT = 9090
MSG_TYPE = 'std_msgs/msg/String'
args = [a for a in sys.argv[1:] if not a.startswith('--')]
TEST_VOICE = '--test-voice' in sys.argv


def robot_ip():
    f = os.path.join(LAB, 'robot_ip.txt')
    if args:
        open(f, 'w').write(args[0]); return args[0]
    try:
        return open(f).read().strip()
    except OSError:
        return '10.25.32.136'


def load_places():
    try:
        p = (yaml.safe_load(open(os.path.join(LAB, 'places_lab.yaml'))) or {}).get('places') or {}
        return list(p) or ['home']
    except OSError:
        return ['home']


ROBOT_PLACES = load_places()
# the four demo buttons: (place key on the robot, button text, colour as in the 5.11 figure)
DEMO = [('home', 'Home', '#1f4e9e'), ('auditorium', 'Auditorium', '#7b2fa0'),
        ('library', 'Library', '#1e8449'), ('cafeteria', 'Cafeteria', '#c0590a')]
PLACES = ROBOT_PLACES + [k for k, _, _ in DEMO if k not in ROBOT_PLACES]  # voice also knows the demo names
SPOKEN = [p.replace('_', ' ') for p in PLACES]
_model = None


def transcribe(audio):
    global _model
    if _model is None:
        from faster_whisper import WhisperModel
        _model = WhisperModel('base.en', device='cpu', compute_type='int8')
    segs, _ = _model.transcribe(audio, language='en')
    return ' '.join(s.text for s in segs).strip()


def record(sec=5, rate=16000):
    import sounddevice as sd
    a = sd.rec(int(sec * rate), samplerate=rate, channels=1, dtype='float32'); sd.wait()
    return a[:, 0]


def to_command(text):
    t = text.lower()
    if any(w in t for w in ('cancel', 'stop')):
        return 'cancel'
    if 'home' in t or 'come back' in t:
        return 'home'
    best = process.extractOne(t, SPOKEN, scorer=fuzz.partial_ratio)
    if best and best[1] >= 70:
        return 'go ' + PLACES[best[2]]
    return None


def say(text):
    t = text.replace("'", "''")
    subprocess.Popen(['powershell', '-NoProfile', '-Command',
                      "Add-Type -AssemblyName System.Speech; "
                      "(New-Object System.Speech.Synthesis.SpeechSynthesizer).Speak('%s')" % t],
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                     creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))


def test_voice():
    print('places:', ', '.join(PLACES))
    say('Voice test. Say a place name after the beep.'); time.sleep(3)
    print('recording 5 s ...', flush=True)
    text = transcribe(record())
    print('heard   :', repr(text))
    print('command :', to_command(text))


class Link:
    """rosbridge connection; callbacks put events on a queue that the Tk loop reads."""
    def __init__(self, ip, q):
        import roslibpy
        self.q = q; self.ip = ip
        self.ros = roslibpy.Ros(host=ip, port=PORT)
        self.cmd = roslibpy.Topic(self.ros, '/guide/command', MSG_TYPE)
        self.st = roslibpy.Topic(self.ros, '/guide/status', MSG_TYPE)
        self.st.subscribe(lambda m: self.q.put(('status', m.get('data', ''))))
        self.Message = roslibpy.Message
        threading.Thread(target=self._connect, daemon=True).start()

    def _connect(self):
        try:
            self.ros.run(timeout=10)
        except Exception:
            pass  # roslibpy keeps retrying in the background; the GUI shows the state

    def connected(self):
        return bool(self.ros.is_connected)

    def send(self, cmd):
        if not self.connected():
            return False
        self.cmd.publish(self.Message({'data': cmd})); return True


def main():
    if TEST_VOICE:
        test_voice(); return
    ip = robot_ip(); q = queue.Queue()
    root = tk.Tk(); root.title('Lab guide robot')
    info = tk.StringVar(value='Connecting to robot %s:%d ...' % (ip, PORT))
    link_txt = tk.StringVar(value='link: connecting')
    link = Link(ip, q)
    state = {'was': None, 'asked': False, 'quit': False}

    def on_status(raw):
        try:
            d = json.loads(raw)
        except ValueError:
            d = {'state': '?', 'msg': raw}
        key = d.get('target') or ''
        tgt = key.replace('_', ' ')
        busy = d.get('state') in ('going', 'battery', 'quitting')
        for k, b in buttons.items():  # the place the robot is heading for stays highlighted
            b.config(relief='sunken' if busy and k == key else 'raised')
        v = d.get('battery')
        info.set('%s %s  %s%s' % (d.get('state'), tgt, d.get('msg') or '',
                                  '   [battery %.2f V]' % v if v else ''))
        if d.get('hb'):
            return
        s = d.get('state')
        if s == 'shutdown':
            say('Goodbye. ' + ('I am home.' if (d.get('msg') or '').startswith('at home')
                               else 'I could not reach home. Please carry me to the home mark.'))
            info.set('Robot software stopping: ' + (d.get('msg') or '') + '. This window closes in 5 s.')
            if not state.get('closing'): state['closing'] = True; root.after(5000, root.destroy)
            return
        if s == 'arrived':
            say('We have arrived at the ' + tgt)
        elif s == 'failed':
            say('Sorry, I could not reach the ' + tgt)
        elif s == 'battery':
            say('My battery is low. I am going back home.')
        elif s == 'error' and 'battery' in (d.get('msg') or ''):
            say('Sorry, my battery is too low. Please charge me.')

    def poll():
        while not q.empty():
            kind, data = q.get()
            if kind == 'status': on_status(data)
            elif kind == 'heard':
                entry.delete(0, 'end'); entry.insert(0, data); send(to_command(data))
            elif kind == 'info': info.set(data)
        c = link.connected()
        if state['quit'] and state['was'] and not c and not state.get('closing'):  # robot software already gone
            state['closing'] = True; root.after(3000, root.destroy)
        if c != state['was']:
            state['was'] = c
            link_txt.set('link: OK (%s)' % ip if c else 'link: NOT connected (%s:%d)' % (ip, PORT))
            lbl.config(fg='#080' if c else '#c00')
            if c and not state['asked']:
                state['asked'] = True; link.send('status')
        root.after(200, poll)

    def send(cmd):
        if cmd is None:
            info.set('Not understood. Try a place name.'); say('Sorry, which place?'); return
        if not link.send(cmd):
            info.set('Not connected to the robot: is rosbridge running?'); return
        info.set('Sent: ' + cmd)
        if cmd.startswith('go '):
            say('Please follow me to the ' + cmd[3:].replace('_', ' '))

    grid = tk.Frame(root); grid.pack(padx=10, pady=8)
    buttons = {}
    for i, (key, text, col) in enumerate(DEMO):
        cmd = 'home' if key == 'home' else 'go ' + key
        label = text if key == 'home' or key in ROBOT_PLACES else text + '\n(not saved on robot)'
        b = tk.Button(grid, text=label, width=20, height=3, bg=col, fg='white', activebackground=col,
                      font=('Segoe UI', 14, 'bold'), command=lambda c=cmd: send(c))
        b.grid(row=i // 2, column=i % 2, padx=6, pady=6); buttons[key] = b
    row = tk.Frame(root); row.pack(padx=10, pady=4, fill='x')
    entry = tk.Entry(row, width=40); entry.pack(side='left', expand=True, fill='x')
    entry.bind('<Return>', lambda e: send(to_command(entry.get())))
    tk.Button(row, text='Go', command=lambda: send(to_command(entry.get()))).pack(side='left', padx=4)

    def mic():
        info.set('Listening for 5 s...')
        def work():
            try:
                q.put(('heard', transcribe(record())))
            except Exception as ex:
                q.put(('info', 'Mic error: %s' % ex))
        threading.Thread(target=work, daemon=True).start()

    ret = tk.BooleanVar(value=True)
    ctl = tk.Frame(root); ctl.pack(padx=10, pady=6)
    tk.Button(ctl, text='Mic (5 s)', width=12, command=mic).pack(side='left', padx=4)
    tk.Checkbutton(ctl, text='Return home after arriving', variable=ret,
                   command=lambda: link.send('return ' + ('on' if ret.get() else 'off'))).pack(side='left', padx=4)
    tk.Button(ctl, text='CANCEL', width=10, bg='#d33', fg='white', command=lambda: send('cancel')).pack(side='left', padx=4)

    def quit_all():
        if not link.connected():
            root.destroy(); return  # nothing runs on the robot side to stop
        if not messagebox.askyesno('Quit', 'The robot drives home, then all robot software stops '
                                   '(Nav2, AMCL, base, lidar, bridges).\nThe robot computer stays on.\n\nQuit now?'):
            return
        state['quit'] = True
        for b in buttons.values(): b.config(state='disabled')
        link.send('quit'); info.set('Returning home, then stopping the robot software ...')
        say('Going home. Then I will shut down.')

    tk.Button(ctl, text='QUIT', width=10, bg='#333', fg='white', command=quit_all).pack(side='left', padx=4)
    tk.Label(root, textvariable=info, anchor='w', width=70).pack(padx=10, pady=(8, 0))
    lbl = tk.Label(root, textvariable=link_txt, anchor='w', width=70); lbl.pack(padx=10, pady=(0, 8))
    root.after(200, poll)
    root.mainloop()
    try:
        link.ros.terminate()
    except Exception:
        pass


if __name__ == '__main__':
    main()
