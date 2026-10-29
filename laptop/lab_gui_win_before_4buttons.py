#!/usr/bin/env python3
# lab_gui_win.py [ROBOT_IP] [--test-voice] : lab guide GUI on Windows (no ROS on the laptop).
# Talks to the robot's rosbridge (TCP 9090) with roslibpy: sends std_msgs/String on /guide/command,
# shows /guide/status (JSON from guide_manager_foxy.py). Voice: 5 s mic -> faster-whisper -> place name.
# Speech out: Windows SAPI via PowerShell. Places: %USERPROFILE%\lab\places_lab.yaml (copied by start_gui.ps1).
import json, os, queue, subprocess, sys, threading, time, tkinter as tk
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


PLACES = load_places()
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
    state = {'was': None, 'asked': False}

    def on_status(raw):
        try:
            d = json.loads(raw)
        except ValueError:
            d = {'state': '?', 'msg': raw}
        tgt = (d.get('target') or '').replace('_', ' ')
        v = d.get('battery')
        info.set('%s %s  %s%s' % (d.get('state'), tgt, d.get('msg') or '',
                                  '   [battery %.2f V]' % v if v else ''))
        if d.get('hb'):
            return
        s = d.get('state')
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
    for i, p in enumerate([p for p in PLACES if p != 'home']):
        tk.Button(grid, text=p.replace('_', ' ').title(), width=18, height=2,
                  command=lambda p=p: send('go ' + p)).grid(row=i // 3, column=i % 3, padx=4, pady=4)
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
    tk.Button(ctl, text='Home', width=10, command=lambda: send('home')).pack(side='left', padx=4)
    tk.Button(ctl, text='CANCEL', width=10, bg='#d33', fg='white', command=lambda: send('cancel')).pack(side='left', padx=4)
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
