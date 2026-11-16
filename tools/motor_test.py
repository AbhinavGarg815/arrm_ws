#!/usr/bin/env python3
"""Drive the motor-test firmware on the Mega (DS51150 x3) and the ESP32 (ST3215 x3).

Every motion command runs inside a session that streams a heartbeat to the
board. If this process dies, hangs, is Ctrl-C'd or loses the USB link, the
firmware's dead-man relaxes the motor within 0.4 s. On every exit path -
normal, exception, Ctrl-C - the session also sends an explicit stop.

    # no motion
    motor_test.py mega-status
    motor_test.py esp-scan

    # one joint, relative move and back, then stop
    motor_test.py mega-joint 1 5       # DS51150 joint 1 (base) +5 deg, then back
    motor_test.py esp-joint 2 5        # ST3215 id 2 +5 deg, then back

    # emergency stop, from any terminal, even while a test is running
    motor_test.py estop

Run as a member of 'dialout' (a fresh login after `usermod -aG dialout`).
"""

import argparse
import re
import sys
import threading
import time

import serial

MEGA_PORT = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_85339313233351213111-if00"
# The ESP32 board, by device identity rather than by USB socket: a by-path name
# stopped existing when the board was moved, and the stale name then resolved to
# the MEGA, so these tools asked the Arduino ST3215 questions. The CP2102's
# serial is the generic "0001", so this is unique only while it is the only
# CP2102 attached - if a second appears, write a udev rule for /dev/arm-wrist.
ESP_PORT = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"


def xor_checksum(text):
    v = 0
    for ch in text.encode():
        v ^= ch
    return v


def strip_checksum(line):
    """Return the payload if the checksum is absent or correct, else None."""
    if "*" not in line:
        return line
    body, _, want = line.rpartition("*")
    try:
        return body if xor_checksum(body) == int(want, 16) else None
    except ValueError:
        return None


class Board:
    """A serial link with a background reader and a heartbeat."""

    def __init__(self, port, baud, name, reset_on_open):
        self.name = name
        self.ser = serial.Serial()
        self.ser.port, self.ser.baudrate, self.ser.timeout = port, baud, 0.05
        if not reset_on_open:
            # ESP32 auto-reset circuit: with DTR and RTS both deasserted the
            # chip keeps running. (The Mega resets on open regardless - its
            # firmware is safe at boot, so that is acceptable.)
            self.ser.dtr = False
            self.ser.rts = False
        self.ser.open()
        self._lock = threading.Lock()
        self._lines = []
        self._cv = threading.Condition()
        self._alive = True
        self._beat = False
        threading.Thread(target=self._reader, daemon=True).start()
        threading.Thread(target=self._heartbeat, daemon=True).start()

    def _reader(self):
        buf = b""
        while self._alive:
            try:
                chunk = self.ser.read(256)
            except serial.SerialException:
                break
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode(errors="replace").strip()
                if not line:
                    continue
                with self._cv:
                    self._lines.append(line)
                    self._cv.notify_all()
                if line.startswith(("!!", "##")) and not line.startswith("## target") \
                        and not line.startswith("## goal"):
                    print(f"  [{self.name}] {line}")

    def _heartbeat(self):
        while self._alive:
            if self._beat:
                self.send("h", quiet=True)
            time.sleep(0.1)

    def send(self, cmd, quiet=False, checksum=True):
        """Send one line. Every line carries a checksum by default.

        The Mega's USB-serial bridge corrupts ~0.6% of host->device lines while
        it is transmitting, by dropping and inserting bytes rather than just
        flipping them. Both firmwares accept an unchecksummed line so a human
        can type into a serial monitor, so a mangled one would still be acted
        on - and mangling can truncate "G 1 2 3*7F" into a valid "G 1 2".
        """
        if checksum:
            cmd = cmd + "*%02X" % xor_checksum(cmd)
        with self._lock:
            self.ser.write((cmd + "\n").encode())
            self.ser.flush()
        if not quiet:
            print(f"  [{self.name}] > {cmd}")

    def wait_for(self, pattern, timeout, start=0):
        """Return the first line at or after index `start` matching `pattern`."""
        rx = re.compile(pattern)
        deadline = time.time() + timeout
        with self._cv:
            while True:
                for line in self._lines[start:]:
                    if rx.search(line):
                        return line
                left = deadline - time.time()
                if left <= 0:
                    raise TimeoutError(f"{self.name}: no '{pattern}' within {timeout} s")
                self._cv.wait(left)

    def mark(self):
        with self._cv:
            return len(self._lines)

    def lines_since(self, start):
        with self._cv:
            return list(self._lines[start:])

    def heartbeat(self, on):
        self._beat = on

    def stop(self):
        for _ in range(3):
            try:
                self.send("x", quiet=True)
            except serial.SerialException:
                pass
            time.sleep(0.02)

    def close(self):
        self.stop()
        self._beat = False
        self._alive = False
        time.sleep(0.1)
        self.ser.close()


# ------------------------------------------------------------------ Mega --
def open_mega():
    b = Board(MEGA_PORT, 250000, "mega", reset_on_open=True)
    b.wait_for(r"^## ready", timeout=8)          # opening the port reboots it
    return b


def mega_angles(b):
    m = b.mark()
    b.send("?", quiet=True)
    b.wait_for(r"^## active=", timeout=2, start=m)
    rows = {}
    for line in b.lines_since(m):
        if line.startswith("S,"):
            _, n, name, present, exc, deg, excraw, active, pin = line.split(",")
            rows[int(n)] = dict(name=name, present=present == "1", exc_ok=exc == "1",
                                deg=float(deg), exc_raw=int(excraw), pin=int(pin))
    return rows


def cmd_mega_status(_):
    b = open_mega()
    try:
        for n, r in sorted(mega_angles(b).items()):
            print(f"  joint {n} {r['name']:9s} ADS {'found' if r['present'] else 'MISSING':7s} "
                  f"excitation {'OK ' if r['exc_ok'] else 'BAD'} ({r['exc_raw']:6d})  "
                  f"angle {r['deg']:+8.2f} deg  (driven from D{r['pin']})")
    finally:
        b.close()


def settle_mega(b, joint, seconds):
    """Wait for motion to finish, watching for a fault the whole time."""
    end = time.time() + seconds
    m = b.mark()
    while time.time() < end:
        for line in b.lines_since(m):
            if line.startswith("!! FAULT"):
                raise RuntimeError(line)
        time.sleep(0.05)
    return mega_angles(b)[joint]["deg"]


def cmd_mega_joint(args):
    n, deg = args.joint, args.deg
    if not 1 <= n <= 3:
        sys.exit("joint must be 1, 2 or 3")
    if abs(deg) > 20:
        sys.exit("keep test moves within +/-20 deg (the firmware fences at 20 anyway)")
    b = open_mega()
    try:
        if args.pin is not None:
            # Which PWM pin drives this pot channel's servo. The firmware's
            # default table pairs 0x48/D9, 0x49/D10, 0x4A/D11, and on this arm
            # that is wrong: D9 drives the base, whose pot is on 0x4A.
            m = b.mark()
            b.send(f"w {n} {args.pin}")
            b.wait_for(r"^## map|^!!", timeout=2, start=m)
        m = b.mark()
        b.send(f"v {args.speed}")
        b.wait_for(r"^## speed|^!!", timeout=2, start=m)
        before = mega_angles(b)[n]
        print(f"  joint {n} {before['name']} is driven from D{before['pin']}")
        print(f"  joint {n} {before['name']} starts at {before['deg']:+.2f} deg")
        b.heartbeat(True)
        m = b.mark()
        b.send(f"e {n}")
        b.wait_for(r"^## ENABLED|^!!", timeout=2, start=m)
        if any(l.startswith("!!") for l in b.lines_since(m)):
            raise RuntimeError("enable refused")
        time.sleep(0.5)
        held = settle_mega(b, n, 0.3)
        b.send(f"m {deg}")
        out = settle_mega(b, n, abs(deg) / args.speed + 2.0)
        if args.stay:
            # Leave it where it went, for recovering a joint driven OUTSIDE its
            # soft limit, where the closed-loop firmware is no use: it drives
            # harder into the stop and then faults on the pot leaving its rail.
            print(f"\n  RESULT joint {n} {before['name']}: commanded {deg:+.1f} deg, "
                  f"LEFT IN PLACE")
            print(f"    start {before['deg']:+.2f}  held {held:+.2f}  now {out:+.2f}"
                  f"  (moved {out - held:+.2f} deg)")
            return
        b.send(f"m {-deg}")
        back = settle_mega(b, n, abs(deg) / args.speed + 2.0)
        print(f"\n  RESULT joint {n} {before['name']}: commanded {deg:+.1f} deg")
        print(f"    start {before['deg']:+.2f}  held {held:+.2f}  after move {out:+.2f}  "
              f"back {back:+.2f}")
        print(f"    measured change {out - held:+.2f} deg "
              f"(pot moved {'the SAME way' if (out - held) * deg > 0 else 'the OPPOSITE way'}"
              f" as commanded), returned within {abs(back - held):.2f} deg")
    finally:
        b.heartbeat(False)
        b.close()


# ----------------------------------------------------------------- ESP32 --
def open_esp():
    b = Board(ESP_PORT, 115200, "esp32", reset_on_open=False)
    time.sleep(0.3)
    return b


def esp_scan(b):
    m = b.mark()
    b.send("scan")
    b.wait_for(r"^## scan done", timeout=10, start=m)
    rows = {}
    for line in b.lines_since(m):
        if line.startswith("S,"):
            _, i, pos, load, volt, temp, tq, lo, hi, fails, verdict = line.split(",")
            rows[int(i)] = dict(pos=int(pos), load=int(load), volt=float(volt), temp=int(temp),
                                torque=int(tq), lo=int(lo), hi=int(hi), fails=int(fails),
                                verdict=verdict)
    return rows


def cmd_esp_scan(_):
    b = open_esp()
    try:
        rows = esp_scan(b)
        if not rows:
            print("  no ST3215 answered on ids 1..20 - servo power off, or bus not connected")
        for i, r in sorted(rows.items()):
            print(f"  id {i:2d}  pos {r['pos']:4d} ({r['pos'] * 360 / 4096:6.1f} deg)  "
                  f"load {r['load']:+5d}  {r['volt']:4.1f} V  {r['temp']:2d} C  "
                  f"torque {'ON ' if r['torque'] else 'off'}  limits {r['lo']}..{r['hi']}  "
                  f"read-fails {r['fails']}  {r['verdict']}")
    finally:
        b.close()


def cmd_esp_joint(args):
    sid, deg = args.id, args.deg
    if abs(deg) > 45:
        sys.exit("keep ST3215 test moves within +/-45 deg (the firmware fences at 45 anyway)")
    b = open_esp()
    try:
        rows = esp_scan(b)
        if sid not in rows:
            raise RuntimeError(f"id {sid} did not answer the scan")
        b.heartbeat(True)
        m = b.mark()
        b.send(f"hold {sid}")
        b.wait_for(r"^## HOLDING|^!!", timeout=2, start=m)
        if any(l.startswith("!!") for l in b.lines_since(m)):
            raise RuntimeError("hold refused")
        b.send("t1")

        def pos_now(wait):
            end, last, peak, m2 = time.time() + wait, None, 0, b.mark()
            while time.time() < end:
                for line in b.lines_since(m2):
                    if line.startswith("!! FAULT"):
                        raise RuntimeError(line)
                    if line.startswith("T,"):
                        _, _, _, goal, pos, load = line.split(",")
                        last, peak = int(pos), max(peak, abs(int(load)))
                time.sleep(0.05)
            return last, peak

        held, _ = pos_now(0.5)
        b.send(f"m {deg}")
        out, peak1 = pos_now(abs(deg) / 10.0 + 1.5)
        k = 360.0 / 4096.0
        if args.stay:
            # Leave the joint where it went, for visual calibration: park it,
            # look, then re-zero. The servo relaxes when this process exits, so
            # the joint then holds on friction alone.
            print(f"\n  RESULT ST3215 id {sid}: commanded {deg:+.1f} deg, LEFT IN PLACE")
            print(f"    held {held} ({held * k:.1f} deg)  now {out} ({out * k:.1f} deg)  "
                  f"measured change {(out - held) * k:+.2f} deg, peak load {peak1}/1000")
            return
        b.send(f"m {-deg}")
        back, peak2 = pos_now(abs(deg) / 10.0 + 1.5)
        print(f"\n  RESULT ST3215 id {sid}: commanded {deg:+.1f} deg")
        print(f"    held {held} ({held * k:.1f} deg)  after move {out} ({out * k:.1f})  back {back}")
        print(f"    measured change {(out - held) * k:+.2f} deg, returned within "
              f"{abs(back - held) * k:.2f} deg, peak load {max(peak1, peak2)}/1000")
    finally:
        b.heartbeat(False)
        b.close()


def cmd_esp_centre(args):
    """Re-index one ST3215's zero onto where it is standing right now.

    This writes the servo's EEPROM and it is the servo, not this repo, that
    remembers - so get the joint into the pose you want to call zero FIRST
    (park it with `esp-joint <id> <deg> --stay` and look at it), then run this.
    Afterwards the joint reads 2048 there, and actuators.yaml wants
    motor_counts_at_zero: 2048 for it.
    """
    b = open_esp()
    try:
        rows = esp_scan(b)
        if args.id not in rows:
            raise RuntimeError(f"id {args.id} did not answer the scan")
        before = rows[args.id]["pos"]
        k = 360.0 / 4096.0
        print(f"  id {args.id} is standing at {before} counts ({before * k:.2f} deg)")
        print(f"  re-zeroing HERE: this position becomes 2048")
        m = b.mark()
        b.send(f"centre {args.id}")
        line = b.wait_for(r"^## centred|^!!", timeout=5, start=m)
        if line.startswith("!!"):
            raise RuntimeError(line)
        for l in b.lines_since(m):
            if l.startswith("!!"):
                raise RuntimeError(l)
    finally:
        b.heartbeat(False)
        b.close()


# --- arm3: the three-joint closed-loop firmware (DS51150_test env `arm3`).
# Same board and baud as motor_test, different protocol: E/D enable, G/g
# targets, F <hz> status stream, one "F,..." line carrying all three joints.
ARM3_FAULTS = {0: "none", 1: "tracking", 2: "wiper", 3: "excitation",
               4: "dead-man", 5: "refused"}


def open_arm3():
    b = Board(MEGA_PORT, 250000, "arm3", reset_on_open=True)
    b.wait_for(r"^## ready", timeout=8)
    return b


def arm3_frames(b, start):
    """Parse the status stream since `start` into dicts."""
    out = []
    for raw in b.lines_since(start):
        if not raw.startswith("F,"):
            continue
        line = strip_checksum(raw)
        if line is None:              # corrupted in transit - drop it
            continue
        f = line.split(",")
        if len(f) != 17:
            continue
        try:
            out.append(dict(ms=int(f[1]), state=int(f[2]), fault=int(f[3]), fjoint=int(f[4]),
                            meas=[float(x) for x in f[5:8]],
                            set=[float(x) for x in f[8:11]],
                            target=[float(x) for x in f[11:14]],
                            us=[int(x) for x in f[14:17]]))
        except ValueError:
            continue
    return out


def arm3_watch(b, seconds, start=None):
    """Collect frames for `seconds`, raising on any fault."""
    m = b.mark() if start is None else start
    end = time.time() + seconds
    while time.time() < end:
        for line in b.lines_since(m):
            if line.startswith("!! FAULT"):
                raise RuntimeError(line)
        time.sleep(0.05)
    return arm3_frames(b, m)


def cmd_arm3_status(_):
    b = open_arm3()
    try:
        m = b.mark()
        b.send("?")
        b.wait_for(r"^## enabled=", timeout=3, start=m)
        for line in b.lines_since(m):
            if line.startswith("S,"):
                f = line.split(",")
                print(f"  joint {f[1]} {f[2]:9s} {f[3]:4s} pot {f[4]:5s} "
                      f"ads {'ok' if f[5]=='1' else 'MISSING':7s} "
                      f"exc {'ok' if f[6]=='1' else 'BAD':3s} "
                      f"wiper {'ok' if f[7]=='1' else 'BAD':3s} "
                      f"meas {float(f[8]):+8.2f}  set {float(f[9]):+8.2f}  "
                      f"target {float(f[10]):+8.2f}  {f[11]} us  {f[12]} {f[13]} {f[14]}")
    finally:
        b.close()


def cmd_arm3_hold(args):
    """Enable and just hold. The first closed-loop test: nothing is commanded
    to move, so any motion at all is the loop misbehaving."""
    b = open_arm3()
    try:
        b.send("F 50")
        b.wait_for(r"^## status", timeout=2)
        b.heartbeat(True)
        for spec in getattr(args, "gains", []):
            joint, _, nums = spec.partition(":")
            vals = [v.strip() for v in nums.split(",") if v.strip()]
            if not joint.strip().isdigit() or len(vals) not in (3, 4):
                raise SystemExit(
                    f"--gains wants N:KP,KI,KD[,KV], got {spec!r}")
            line = "k " + joint.strip() + " " + " ".join(vals)
            mg = b.mark()
            b.send(line)
            # The firmware acks this one, so a typo or an out-of-range value is
            # caught HERE rather than being silently ignored and then blamed on
            # whatever the arm does next.
            b.wait_for(r"^##|^!!", timeout=2, start=mg)
            for ln in b.lines_since(mg):
                if ln.startswith("!!"):
                    raise RuntimeError(f"firmware rejected '{line}': {ln}")
            print(f"  gains -> {line}   (RAM only; power cycle restores arm3.cpp)")
        m = b.mark()
        b.send("E")
        b.wait_for(r"^## ENABLED|^!!", timeout=3, start=m)
        if any(l.startswith("!!") for l in b.lines_since(m)):
            raise RuntimeError("enable refused")
        frames = arm3_watch(b, args.seconds)
        if not frames:
            raise RuntimeError("no status frames - firmware not streaming?")
        first = frames[0]["meas"]
        print(f"\n  held for {args.seconds:.0f} s, {len(frames)} frames")
        for i in range(3):
            meas = [f["meas"][i] for f in frames]
            err = [f["set"][i] - f["meas"][i] for f in frames]
            print(f"    joint {i+1}: start {first[i]:+7.2f}  end {meas[-1]:+7.2f}  "
                  f"drift {meas[-1]-first[i]:+6.2f}  spread {max(meas)-min(meas):5.2f}  "
                  f"|err| max {max(abs(e) for e in err):5.2f} deg")
    finally:
        b.heartbeat(False)
        b.close()


def cmd_arm3_move(args):
    n, deg = args.joint, args.deg
    b = open_arm3()
    try:
        b.send(f"v {args.speed}")
        b.wait_for(r"^## vlim", timeout=2)
        b.send("F 50")
        b.wait_for(r"^## status", timeout=2)
        b.heartbeat(True)
        m = b.mark()
        b.send("E")
        b.wait_for(r"^## ENABLED|^!!", timeout=3, start=m)
        if any(l.startswith("!!") for l in b.lines_since(m)):
            raise RuntimeError("enable refused")
        base = arm3_watch(b, 0.5)[-1]["meas"][n - 1]
        wait = abs(deg) / args.speed + 2.5
        b.send(f"g {n} {base + deg:.2f}")
        out = arm3_watch(b, wait)[-1]["meas"][n - 1]
        b.send(f"g {n} {base:.2f}")
        back = arm3_watch(b, wait)[-1]["meas"][n - 1]
        print(f"\n  RESULT joint {n}: commanded {deg:+.1f} deg at {args.speed} deg/s")
        print(f"    start {base:+.2f}  after {out:+.2f} (moved {out-base:+.2f})  "
              f"back {back:+.2f} (within {abs(back-base):.2f})")
    finally:
        b.heartbeat(False)
        b.close()


def cmd_arm3_stream(args):
    """Stream targets for all three joints at 50 Hz - the path ros2_control
    will use. A slow sine on every joint at once, so the test exercises
    simultaneous motion, the unacked streaming command, and the status echo
    that stands in for an ack."""
    import math
    b = open_arm3()
    try:
        b.send(f"v {args.speed}")
        b.wait_for(r"^## vlim", timeout=2)
        b.send("F 50")
        b.wait_for(r"^## status", timeout=2)
        # Optionally also run the binary stream, purely as transmit load, so
        # this tool matches what ros2_control puts on the link (`B 100`). TX
        # interferes with RX on this bridge, and an XOR8 passes roughly 1 in 256
        # corrupted lines, so the rate of wrong-but-accepted targets scales with
        # transmit load. `F 50` stays on so the tool can parse its telemetry.
        if args.binary_load:
            b.send(f"B {int(args.binary_load)}")
        b.heartbeat(True)
        m = b.mark()
        b.send("E")
        b.wait_for(r"^## ENABLED|^!!", timeout=3, start=m)
        if any(l.startswith("!!") for l in b.lines_since(m)):
            raise RuntimeError("enable refused")
        start = arm3_watch(b, 0.5)[-1]["meas"]
        print(f"  starting from {[round(x,2) for x in start]}")

        m = b.mark()
        t0 = time.time()
        period = args.seconds
        while True:
            t = time.time() - t0
            if t > period:
                break
            phase = 2 * math.pi * t / period
            tgt = [start[i] + args.amplitude * math.sin(phase) for i in range(3)]
            b.send(f"G {tgt[0]:.2f} {tgt[1]:.2f} {tgt[2]:.2f}", quiet=True, checksum=True)
            for line in b.lines_since(m):
                if line.startswith("!! FAULT"):
                    raise RuntimeError(line)
            time.sleep(1.0 / args.rate)
        for i in range(3):
            b.send(f"g {i+1} {start[i]:.2f}")
        frames = arm3_watch(b, 3.0, start=m)

        print(f"\n  {len(frames)} status frames over {period:.0f} s")
        for i in range(3):
            err = [f["set"][i] - f["meas"][i] for f in frames]
            cmd = [f["target"][i] for f in frames]
            got = [f["meas"][i] for f in frames]
            print(f"    joint {i+1}: commanded range {min(cmd):+7.2f}..{max(cmd):+7.2f}  "
                  f"followed {min(got):+7.2f}..{max(got):+7.2f}  "
                  f"|tracking err| max {max(abs(e) for e in err):5.2f} deg")
        end = frames[-1]["meas"]
        print(f"    returned to {[round(x,2) for x in end]} "
              f"(started {[round(x,2) for x in start]})")
        good = sum(1 for l in b.lines_since(m) if l.startswith("F,") and strip_checksum(l))
        total = sum(1 for l in b.lines_since(m) if l.startswith("F,"))
        print(f"    status frames: {good}/{total} passed checksum")
        b.send("?")                      # firmware's own rx_ok / rx_bad tally
        time.sleep(0.5)
    finally:
        b.heartbeat(False)
        b.close()


def cmd_arm3_monitor(args):
    """Watch all three pots live, without ever enabling a servo.

    For chasing an intermittent connection: run this, then flex the board,
    press the suspect joint, and twist the cable. A dropout shows up instantly
    as a jump or an out-of-range angle. Nothing can move - the arm is never
    enabled, so this is safe to run while poking at the electronics."""
    b = open_arm3()
    try:
        b.send("F 20")
        b.wait_for(r"^## status", timeout=2)
        print(f"  watching for {args.seconds:.0f} s - flex the board and cables now")
        print(f"  {'time':>6s}  {'base':>9s} {'shoulder':>9s} {'elbow':>9s}   glitches")
        m = b.mark()
        t0 = time.time()
        last = None
        glitches = [0, 0, 0]
        lo = [None, None, None]
        hi = [None, None, None]
        nxt = 0.0
        while time.time() - t0 < args.seconds:
            frames = arm3_frames(b, m)
            if frames:
                m = b.mark()
                for f in frames:
                    for i in range(3):
                        v = f["meas"][i]
                        # Outside the servo's own travel, or a jump no arm can
                        # make between frames, means the sensing dropped out.
                        if abs(v) > 140 or (last and abs(v - last[i]) > 5.0):
                            glitches[i] += 1
                            continue        # a dropout is not travel
                        lo[i] = v if lo[i] is None else min(lo[i], v)
                        hi[i] = v if hi[i] is None else max(hi[i], v)
                    last = f["meas"]
            if time.time() - t0 >= nxt:
                nxt += 1.0
                if last:
                    print(f"  {time.time()-t0:5.0f}s  {last[0]:+9.2f} {last[1]:+9.2f} "
                          f"{last[2]:+9.2f}   {glitches}")
            time.sleep(0.05)
        print(f"\n  glitches: base {glitches[0]}, shoulder {glitches[1]}, elbow {glitches[2]}")
        print("  zero while you were poking it = the joint is sound;"
              " any count at all = still intermittent")
        # The same stream measures TRAVEL, if what you were doing was moving each
        # joint by hand to its stops rather than flexing cables. Readings flagged
        # as glitches are excluded - a sensing dropout is not travel.
        print("\n  range seen (ignore if you were cable-poking, not moving joints):")
        for i, name in enumerate(("base", "shoulder", "elbow")):
            if lo[i] is None:
                print(f"    {name:9s} no clean reading")
            else:
                print(f"    {name:9s} {lo[i]:+8.2f} .. {hi[i]:+8.2f} deg"
                      f"   = {hi[i]-lo[i]:6.2f} deg of travel")
    finally:
        b.close()


def cmd_arm3_probe(args):
    """Find a joint's real limit by driving it until it stops following.

    Hand-moving these joints is not an option: a DS51150 is geared for
    150 kg-cm and is not back-drivable. Instead the joint is walked outward in
    small steps under power while the tracking error is watched. While the joint
    is free, measured follows setpoint within a few tenths of a degree; at a
    hard stop the setpoint keeps going and the measurement does not. That onset
    is the limit, and stopping at the onset rather than at a stall means the
    answer arrives with its margin already in it.

        motor_test.py arm3-probe 3  120     # elbow, toward +120
        motor_test.py arm3-probe 3 -120     # elbow, toward -120
    """
    n = args.joint
    if n < 1 or n > 3:
        sys.exit("joint must be 1, 2 or 3")
    b = open_arm3()
    try:
        b.send(f"v {args.speed}")
        b.wait_for(r"^## vlim", timeout=2)
        b.send("F 50")
        b.wait_for(r"^## status", timeout=2)
        b.heartbeat(True)
        m = b.mark()
        b.send("E")
        b.wait_for(r"^## ENABLED|^!!", timeout=3, start=m)
        if any(l.startswith("!!") for l in b.lines_since(m)):
            raise RuntimeError("enable refused")

        def state(wait):
            # Take the mark BEFORE waiting. Marking inside the read loop asks for
            # "frames that arrived since this instant", which is always none.
            mk = b.mark()
            end, last = time.time() + wait, None
            while time.time() < end:
                fr = arm3_frames(b, mk)
                if fr:
                    mk = b.mark()
                    last = fr[-1]
                time.sleep(0.05)
            return last

        f = state(0.6)
        if not f:
            raise RuntimeError("no status frames")
        here = f["meas"][n - 1]
        step = abs(args.step) * (1 if args.target > here else -1)
        print(f"  joint {n} at {here:+.2f} deg, walking toward {args.target:+.1f} "
              f"in {abs(step):.1f} deg steps")
        print(f"  stopping when it falls {args.err:.1f} deg behind the setpoint\n")

        best = here
        target = here
        while (step > 0 and target < args.target) or (step < 0 and target > args.target):
            target += step
            if (step > 0 and target > args.target) or (step < 0 and target < args.target):
                target = args.target
            b.send(f"g {n} {target:.2f}")
            f = state(abs(step) / max(args.speed, 1.0) + 0.6)
            if not f:
                raise RuntimeError("status stopped")
            if f["fault"]:
                print(f"  FIRMWARE FAULT {ARM3_FAULTS.get(f['fault'], f['fault'])} "
                      f"- stopping")
                break
            meas, setp = f["meas"][n - 1], f["set"][n - 1]
            err = setp - meas
            # cmdUs distinguishes "not being driven" from "being driven and
            # not moving", which look identical in the position alone.
            print(f"    target {target:+7.2f}  set {setp:+7.2f}  meas {meas:+7.2f}"
                  f"  behind {err:+6.2f}  cmd {f['us'][n - 1]:4d} us")
            if abs(err) > args.err:
                print(f"\n  STOPPED: {abs(err):.2f} deg behind at target "
                      f"{target:+.2f}. It stopped following here.")
                break
            best = meas
        else:
            print(f"\n  reached {args.target:+.1f} with no resistance - the limit is "
                  f"beyond this, or beyond the firmware's own soft limit")

        print(f"  furthest FREE position: {best:+.2f} deg")
        print(f"  backing off to {here:+.2f}")
        b.send(f"g {n} {here:.2f}")
        state(abs(best - here) / max(args.speed, 1.0) + 1.5)
    finally:
        b.heartbeat(False)
        b.close()


def cmd_center_all(args):
    """Drive every motor to its own mechanical centre and hold it there.

    For reassembly. A servo parked at mid-travel has symmetric travel either
    way, so if each link is bolted on with the arm upright, motor-zero IS
    URDF-zero and no offset has to be calibrated afterwards.

    Centre means the MOTOR's centre, not the URDF zero.

        tools/motor_test.py center-all            # hold for 5 minutes
        tools/motor_test.py center-all 900        # hold for 15

    It holds with torque ON, and both firmwares relax on a 400 ms dead-man, so
    the arm goes limp within half a second of this tool exiting. Run it with the
    links detached, or support them.
    """
    mega = open_arm3()
    esp = None
    try:
        mega.send("v 6")
        mega.wait_for(r"^## vlim", timeout=2)
        mega.send("F 20")
        mega.wait_for(r"^## status", timeout=2)
        mega.heartbeat(True)
        m = mega.mark()
        mega.send("E")
        mega.wait_for(r"^## ENABLED|^!!", timeout=3, start=m)
        if any(l.startswith("!!") for l in mega.lines_since(m)):
            raise RuntimeError("Mega refused to enable - check the pots and power")
        mega.send("G 0 0 0")
        print("  Mega: joints 1-3 -> 0.00 deg (1500 us, mid-travel)")

        esp = open_esp()
        esp.heartbeat(True)
        m = esp.mark()
        esp.send("E")
        esp.wait_for(r"^## ENABLED|^!!", timeout=3, start=m)
        if any(l.startswith("!!") for l in esp.lines_since(m)):
            raise RuntimeError("ESP32 refused to enable - is a servo not answering?")
        esp.send("G 2048 2048 2048")
        print("  ESP32: ids 1-3 -> 2048 counts (mid-travel)")

        print(f"\n  holding for {args.seconds:.0f} s. Mount links NOW, with the arm")
        print(f"  in its upright pose. Ctrl-C relaxes everything.\n")
        t0 = time.time()
        nxt = 0.0
        while time.time() - t0 < args.seconds:
            if time.time() - t0 >= nxt:
                nxt += 5.0
                last = None
                mk = mega.mark()
                end = time.time() + 0.5
                while time.time() < end:
                    fr = arm3_frames(mega, mk)
                    if fr:
                        last = fr[-1]
                    time.sleep(0.05)
                if last:
                    if last["fault"]:
                        raise RuntimeError(
                            f"Mega fault: {ARM3_FAULTS.get(last['fault'], last['fault'])}")
                    print(f"    t+{time.time()-t0:5.0f}s  mega "
                          + "  ".join(f"{v:+7.2f}" for v in last["meas"]))
            time.sleep(0.1)
        print("\n  done - relaxing")
    finally:
        mega.heartbeat(False)
        mega.close()
        if esp is not None:
            esp.heartbeat(False)
            esp.close()


# ----------------------------------------------------------------- estop --
def cmd_estop(_):
    """Stop both boards. Safe to run while another test holds the ports."""
    for name, port, baud, reset in (("mega", MEGA_PORT, 250000, True),
                                    ("esp32", ESP_PORT, 115200, False)):
        try:
            s = serial.Serial()
            s.port, s.baudrate, s.timeout = port, baud, 0.1
            if not reset:
                s.dtr = False
                s.rts = False
            s.open()
            for _ in range(5):
                s.write(b"x\n")
                time.sleep(0.02)
            s.close()
            print(f"  {name}: stop sent")
        except (serial.SerialException, OSError) as e:
            print(f"  {name}: could not open ({e})")
    print("  Software stop only. The physical e-stop is what removes power.")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("mega-status").set_defaults(fn=cmd_mega_status)
    p = sub.add_parser("mega-joint")
    p.add_argument("joint", type=int)
    p.add_argument("deg", type=float)
    p.add_argument("--stay", action="store_true",
                   help="do not move back - leave the joint where it went. For "
                        "recovering a joint driven outside its soft limit")
    p.add_argument("--pin", type=int, choices=(9, 10, 11), default=None,
                   help="PWM pin that drives this joint (default: firmware table)")
    p.add_argument("--speed", type=float, default=10.0,
                   help="deg/s, 1..10 (default 10; the firmware caps it at 10)")
    p.set_defaults(fn=cmd_mega_joint)
    sub.add_parser("esp-scan").set_defaults(fn=cmd_esp_scan)
    p = sub.add_parser("esp-joint")
    p.add_argument("id", type=int)
    p.add_argument("deg", type=float)
    p.add_argument("--stay", action="store_true",
                   help="do not move back - park the joint there for a visual check")
    p.set_defaults(fn=cmd_esp_joint)
    p = sub.add_parser("esp-centre", help="store the joint's present position as its 2048 zero")
    p.set_defaults(fn=cmd_esp_centre)
    p.add_argument("id", type=int)

    p = sub.add_parser("arm3-probe",
                       help="find a joint's real limit by driving it to the onset "
                            "of resistance (never by hand - these are not "
                            "back-drivable)")
    p.set_defaults(fn=cmd_arm3_probe)
    p.add_argument("joint", type=int)
    p.add_argument("target", type=float, help="angle to walk toward, in degrees")
    p.add_argument("--step", type=float, default=1.5)
    p.add_argument("--err", type=float, default=2.0,
                   help="degrees behind the setpoint that counts as a stop")
    p.add_argument("--speed", type=float, default=5.0)

    p = sub.add_parser("center-all",
                       help="park every motor at its mechanical centre and hold, "
                            "for bolting links on during reassembly")
    p.set_defaults(fn=cmd_center_all)
    p.add_argument("seconds", type=float, nargs="?", default=300.0)

    sub.add_parser("arm3-status").set_defaults(fn=cmd_arm3_status)
    p = sub.add_parser("arm3-hold")
    p.add_argument("seconds", type=float, nargs="?", default=5.0)
    # Retune a joint's gains for this run only. The firmware's `k` command
    # writes to RAM, so a power cycle restores the compiled-in values - which is
    # what makes trying a gain on the real arm safe, and why one that works has
    # to be written into arm3.cpp afterwards or it is lost.
    p.add_argument("--gains", metavar="N:KP,KI,KD[,KV]", action="append", default=[],
                   help="set joint N's gains before enabling, e.g. --gains 1:6,3,3.0,1.6 "
                        "(RAM only; a power cycle undoes it)")
    p.set_defaults(fn=cmd_arm3_hold)
    p = sub.add_parser("arm3-move")
    p.add_argument("joint", type=int)
    p.add_argument("deg", type=float)
    p.add_argument("--speed", type=float, default=5.0)
    p.set_defaults(fn=cmd_arm3_move)
    p = sub.add_parser("arm3-stream")
    p.add_argument("--amplitude", type=float, default=5.0)
    p.add_argument("--seconds", type=float, default=12.0)
    p.add_argument("--speed", type=float, default=5.0)
    # Match ros2_control's update_rate (100 Hz) when standing in for it: the
    # bridge corrupts ~0.6% of received lines while transmitting and an XOR8
    # passes roughly 1 in 256, so the rate of wrong-but-accepted setpoints
    # scales with this number.
    p.add_argument("--rate", type=float, default=50.0,
                   help="command rate in Hz (default 50; ros2_control uses 100)")
    p.add_argument("--binary-load", type=float, default=0.0, metavar="HZ",
                   help="also run the BINARY status stream at HZ, as transmit load "
                        "(ros2_control uses 100); 0 = off")
    p.set_defaults(fn=cmd_arm3_stream)
    p = sub.add_parser("arm3-monitor")
    p.add_argument("seconds", type=float, nargs="?", default=30.0)
    p.set_defaults(fn=cmd_arm3_monitor)
    sub.add_parser("estop").set_defaults(fn=cmd_estop)
    args = ap.parse_args()
    try:
        args.fn(args)
    except KeyboardInterrupt:
        print("\n  interrupted - stop sent")
        return 130
    except (RuntimeError, TimeoutError) as e:
        print(f"\n  ABORTED: {e}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
