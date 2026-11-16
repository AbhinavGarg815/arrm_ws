#!/usr/bin/env python3
"""Measure and tune the ST3215 wrist servos at the bench, without ROS.

Why this exists: the wrist hunts only when gravity loads it, and the servos hunt
on their OWN internal loop - while holding, the ESP32 sends them nothing at all.
The only levers are the servos' own registers, and reaching them means owning the
serial port, which ros2_control cannot share. So this tool enables the wrist,
measures how much each joint hunts, applies a change, and measures again.

    tools/wrist_tune.py --measure 10                  # hunt amplitude, no change
    tools/wrist_tune.py --pid 3 32 40 0 --measure 10  # Kp Kd Ki on id 3, then measure
    tools/wrist_tune.py --deadband 3 2 2 --measure 10 # insensitive area, in counts

Writes go to EEPROM and persist across power cycles - including into what
ros2_control sees next time. Record what you changed.
"""
import argparse, math, pathlib, struct, sys, threading, time
import serial

# The ESP32 board, by device identity rather than by USB socket: a by-path name
# stopped existing when the board was moved, and the stale name then resolved to
# the MEGA. The CP2102's serial is the generic "0001", so this is unique only
# while it is the only CP2102 attached.
PORT = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
FRAME = 32          # sync2 ms4 en fault id pos6 load6 target6 volt3 vmin sum
IDS = (1, 2, 3)

CFG = str(pathlib.Path(__file__).resolve().parent.parent /
          "src/arm_description/config")
JOINT_OF = {1: "joint4", 2: "joint5", 3: "joint6"}

def counts_to_deg(counts, act):
    """A servo count as the URDF joint angle, in degrees.

    The same mapping arm_hardware applies, from actuators.yaml:
        q = direction * (counts - counts_at_zero) / counts_per_rad
    Doing it here rather than by hand is the point - joint4's direction is -1,
    so a count that LOOKS positive is a negative joint angle, and reading a
    limit off raw counts gets the sign of the limit wrong.
    """
    q = act["direction"] * (counts - act["motor_counts_at_zero"]) / act["counts_per_rad"]
    return q * 180.0 / math.pi

def limit_from_here(deg, margin=5.0, grain=5.0):
    """A tidy limit from a position measured at the point the harness goes taut.

    Two deliberate reductions, both INWARD:
      margin - the measured point is where the cable is already taut. A limit
               there means every move to the limit tensions the harness, and a
               goal tolerance of ~2 deg means some of them overshoot it. joint3
               keeps 4 deg off its measured stop for the same reason.
      grain  - rounds to a round number, so the stored limit is something a
               human can recognise and repeat, not 87.3194 deg of false
               precision from one hand-positioning.

    Always rounds toward zero, so the result is inside what was measured and
    never outside it.
    """
    if deg >= 0:
        return math.floor((deg - margin) / grain) * grain
    return math.ceil((deg + margin) / grain) * grain

def framed(body):
    v = 0
    for ch in body.encode():
        v ^= ch
    return f"{body}*{v:02X}\n".encode()

class Wrist:
    def __init__(self):
        self.ser = serial.Serial()
        self.ser.port, self.ser.baudrate, self.ser.timeout = PORT, 115200, 0.05
        self.ser.dtr = self.ser.rts = False
        self.ser.open()
        time.sleep(1.5)
        self.buf = bytearray()
        self.frames = []
        self.lines = []
        self.beat = False
        self.alive = True
        threading.Thread(target=self._reader, daemon=True).start()
        threading.Thread(target=self._heart, daemon=True).start()

    def _reader(self):
        while self.alive:
            chunk = self.ser.read(256)
            if not chunk:
                continue
            self.buf += chunk
            # Binary frames and ASCII lines share the link; pick out whichever
            # comes next rather than assuming.
            while True:
                i = self.buf.find(b"\xa5\x5a")
                if i >= 0 and len(self.buf) >= i + FRAME:
                    f = bytes(self.buf[i:i + FRAME])
                    del self.buf[:i + FRAME]
                    x = 0
                    for b in f[2:FRAME - 1]:
                        x ^= b
                    if x == f[FRAME - 1]:
                        pos = struct.unpack_from("<3H", f, 9)
                        # Load, 0-1000 of stall, signed by direction: how a
                        # servo says it is pushing against something.
                        load = struct.unpack_from("<3h", f, 15)
                        volts = (f[27] / 10.0, f[28] / 10.0, f[29] / 10.0)
                        self.frames.append((f[6], f[7], pos, min(volts), volts, load))
                    continue
                j = self.buf.find(b"\n")
                if j >= 0 and (i < 0 or j < i):
                    line = bytes(self.buf[:j]).decode(errors="replace").strip()
                    del self.buf[:j + 1]
                    if line:
                        self.lines.append(line)
                        print("  ", line)
                    continue
                break

    def _heart(self):
        while self.alive:
            if self.beat:
                self.send("h")
            time.sleep(0.1)

    def send(self, body):
        self.ser.write(framed(body))
        self.ser.flush()

    def close(self):
        self.beat = False
        for _ in range(3):
            try:
                self.send("D")
            except serial.SerialException:
                pass
            time.sleep(0.02)
        self.alive = False
        time.sleep(0.15)
        self.ser.close()

def wait_line(w, prefix, timeout):
    end = time.time() + timeout
    seen = len(w.lines)
    while time.time() < end:
        for line in w.lines[seen:]:
            if line.startswith(prefix):
                return line
        time.sleep(0.05)
    return None

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pid", nargs=4, type=int, metavar=("ID", "KP", "KD", "KI"))
    ap.add_argument("--deadband", nargs=3, type=int, metavar=("ID", "CW", "CCW"))
    ap.add_argument("--measure", type=float, default=0.0, metavar="SECONDS")
    ap.add_argument("--probe", nargs=2, type=float, metavar=("ID", "TOWARD_DEG"),
                    help="find ONE wrist joint's real travel: walk it toward an "
                         "angle and stop at the onset of resistance. The ST3215s "
                         "are not back-drivable, so this is the only safe way to "
                         "measure it. e.g. --probe 1 90")
    ap.add_argument("--beyond-limits", action="store_true",
                    help="allow probing past the limit in joint_parameters.yaml. "
                         "These joints are limited by CABLE LENGTH, which the probe "
                         "cannot detect - it finds that limit by breaking the cable. "
                         "Only use this when the harness has been verified longer")
    ap.add_argument("--load", type=float, default=150.0,
                    help="load (0-1000 of stall) that counts as resistance "
                         "(default 150). The firmware's own trip is 500, so this "
                         "stops well before it")
    ap.add_argument("--step", type=float, default=2.0, metavar="DEG",
                    help="probe step size in degrees (default 2)")
    ap.add_argument("--recentre", type=int, nargs="+", metavar="ID",
                    help="re-index these servos' zeros onto where they stand NOW, "
                         "so each reads 2048 there. Writes the servo's EEPROM. Use "
                         "after posing the arm by eye: it keeps zero_counts a clean "
                         "2048 with symmetric travel, instead of recording a "
                         "far-from-centre offset that costs range and parks the "
                         "joint near the 0/4095 seam.")
    ap.add_argument("--limit-here", type=int, metavar="ID",
                    help="record where this joint STANDS NOW as one end of its "
                         "travel. Torque off, so you pose it by hand: roll the "
                         "joint until the harness loop just goes taut, then run "
                         "this. Which end it becomes follows from the sign of the "
                         "angle. Use this INSTEAD of --probe on a cable-limited "
                         "joint: the probe finds a cable's limit by snapping it.")
    ap.add_argument("--range", type=float, default=0.0, metavar="SECONDS",
                    help="stream each wrist servo's position with torque OFF and "
                         "report the min/max seen. For measuring real travel: run "
                         "it, then move each wrist joint BY HAND to both of its "
                         "mechanical stops.")
    ap.add_argument("--watch", type=float, default=0.0, metavar="SECONDS",
                    help="stream each servo's status WITHOUT enabling torque, and "
                         "report every dropout. For finding an intermittent cable: "
                         "run it, then flex the wrist wiring by hand near each "
                         "connector in turn and watch which movement drops a servo.")
    a = ap.parse_args()

    w = Wrist()
    try:
        w.send("Q")
        time.sleep(1.2)

        if a.pid:
            w.send("P {} {} {} {}".format(*a.pid))
            if not wait_line(w, "##", 4):
                sys.exit("PID write not confirmed")
        if a.deadband:
            w.send("W {} {} {}".format(*a.deadband))
            if not wait_line(w, "##", 4):
                sys.exit("deadband write not confirmed")
        if a.pid or a.deadband:
            w.send("Q")
            time.sleep(1.2)

        if a.probe:
            # Walk one joint outward and watch the servo's own load reading.
            # While the joint is free the load stays near zero; at a stop it
            # climbs. Stopping at that onset rather than at a stall means the
            # answer arrives with its margin already in it.
            sid = int(a.probe[0])
            if sid not in IDS:
                sys.exit(f"id must be one of {IDS}")
            idx = IDS.index(sid)

            # Clamp to the configured limit unless explicitly overridden.
            # These joints are limited by the HARNESS, which this probe cannot
            # detect: a cable running out of length offers no resistance to stop
            # at, it just goes taut and breaks. Pointed at +170 deg on id1 it
            # found nothing all the way to +150 and snapped that cable. So a
            # limit in joint_parameters.yaml is treated as real here even when
            # nothing mechanical enforces it. --beyond-limits is for when the
            # harness has been verified to reach further.
            import yaml
            joint = JOINT_OF[sid]
            lim = yaml.safe_load(
                open(f"{CFG}/joint_parameters.yaml"))["joints"][joint]
            lo_deg, hi_deg = lim["lower"] * 57.29578, lim["upper"] * 57.29578
            want = a.probe[1]
            if not a.beyond_limits and not (lo_deg <= want <= hi_deg):
                clamped = max(lo_deg, min(hi_deg, want))
                print(f"  {joint} is limited to {lo_deg:+.1f}..{hi_deg:+.1f} deg in "
                      f"joint_parameters.yaml.")
                print(f"  Asked for {want:+.1f}, clamping to {clamped:+.1f}.")
                print(f"  These joints are limited by CABLE LENGTH, which this probe")
                print(f"  cannot feel - it would find that limit by snapping the")
                print(f"  cable, which is exactly what happened on 2026-11-02.")
                print(f"  Pass --beyond-limits only if you have verified the harness")
                print(f"  reaches further.\n")
                a.probe[1] = clamped
            k = 360.0 / 4096.0
            step_ct = max(1, int(round(a.step / k)))

            w.send("B 50")
            w.beat = True
            time.sleep(0.4)
            w.frames.clear()
            w.send("E")
            end = time.time() + 4.0
            live = None
            while time.time() < end and live is None:
                time.sleep(0.05)
                for f in w.frames:
                    if f[1] != 0:
                        sys.exit(f"wrist faulted on enable, code {f[1]}")
                    if f[0] == 1:
                        live = f
            if live is None:
                sys.exit("no frame reported enabled within 4 s")

            def state(wait):
                """Newest frame after `wait` seconds, so the servo has settled."""
                w.frames.clear()
                time.sleep(wait)
                return w.frames[-1] if w.frames else None

            f = state(0.4)
            if f is None:
                sys.exit("status stopped")
            hold = list(f[2])
            start = hold[idx]
            toward_ct = int(round(a.probe[1] / k)) + 2048
            direction = 1 if toward_ct > start else -1
            print(f"  id {sid} at {start} ct ({start * k - 180.0:+.1f} deg from centre"
                  f" = {(start - 2048) * k:+.1f} deg), walking toward {toward_ct} ct")
            print(f"  stopping when its load exceeds {a.load:.0f}/1000, or it stops "
                  f"following\n")

            best = start
            target = start
            over = 0
            while (toward_ct - target) * direction > 0:
                target += direction * step_ct
                if (target - toward_ct) * direction > 0:
                    target = toward_ct
                target = max(0, min(4095, target))
                g = list(hold)
                g[idx] = target
                w.send(f"G {g[0]} {g[1]} {g[2]}")
                f = state(max(0.35, a.step / 20.0 + 0.3))
                if f is None:
                    sys.exit("status stopped mid-probe")
                if f[1] != 0:
                    print(f"  FIRMWARE FAULT code {f[1]} - stopping")
                    break
                pos, load = f[2][idx], abs(f[5][idx])
                behind = (target - pos) * direction
                print(f"    target {target:4d}  pos {pos:4d} ({(pos-2048)*k:+7.1f} deg)"
                      f"  behind {behind:+4d} ct  load {load:4.0f}")
                if load > a.load:
                    over += 1
                    if over >= 2:
                        print(f"\n  STOPPED: load {load:.0f} exceeds {a.load:.0f} twice "
                              f"running at {(pos-2048)*k:+.1f} deg. It is pushing "
                              f"against something.")
                        break
                else:
                    over = 0
                if behind > 3 * step_ct:
                    print(f"\n  STOPPED: {behind} counts behind its target - it has "
                          f"stopped following at {(pos-2048)*k:+.1f} deg.")
                    break
                if target in (0, 4095):
                    print(f"\n  reached the servo's own count limit ({target}) with no "
                          f"resistance - the mechanism allows at least this much.")
                    best = pos
                    break
                best = pos
            else:
                print(f"\n  reached {toward_ct} ct with no resistance - the limit is "
                      f"beyond where this probe was asked to go.")

            print(f"  furthest FREE position: {best} ct = {(best - 2048) * k:+.2f} deg "
                  f"from centre")
            print(f"  returning to {start} ct")
            g = list(hold)
            g[idx] = start
            w.send(f"G {g[0]} {g[1]} {g[2]}")
            time.sleep(abs(best - start) * k / 20.0 + 1.5)
            return

        if a.limit_here is not None:
            # The counterpart to --probe, for joints whose limit is the
            # harness. A probe cannot find a cable's limit - a bundle going taut
            # offers no resistance, it just breaks - so this inverts the roles:
            # the human finds the limit by hand with torque off, and the tool
            # records it, converts it through actuators.yaml and takes its
            # margin.
            sid = a.limit_here
            if sid not in IDS:
                sys.exit(f"id must be one of {IDS}")
            idx = IDS.index(sid)
            joint = JOINT_OF[sid]

            import yaml
            act = yaml.safe_load(open(f"{CFG}/actuators.yaml"))["joints"][joint]
            lim = yaml.safe_load(open(f"{CFG}/joint_parameters.yaml"))["joints"][joint]

            # Torque OFF, and left off. A servo holding position would fight the
            # hand that is posing it, and would report where its loop is holding
            # rather than where the joint rests.
            w.send("D")
            time.sleep(0.4)
            w.send("B 20")
            time.sleep(0.4)
            w.frames.clear()
            print(f"  {joint} (id {sid}), torque OFF - reading for 2 s, hold still.")
            time.sleep(2.0)
            pos = [f[2][idx] for f in w.frames if f[2][idx] != 0]
            if len(pos) < 5:
                sys.exit(f"id {sid} answered {len(pos)} time(s) in 2 s - it is not "
                         f"on the bus. Check the harness and the pack before "
                         f"recording a limit from it.")

            span = max(pos) - min(pos)
            k = 360.0 / 4096.0
            # Insist it is STATIONARY. A joint still being moved gives a reading
            # from somewhere along the way, and the whole value of this method
            # over the probe is that the human has stopped at the right place.
            if span * k > 1.0:
                sys.exit(f"id {sid} moved {span * k:.1f} deg while reading "
                         f"({span} counts). Let go of it and run this again.")

            counts = round(sum(pos) / len(pos))
            deg = counts_to_deg(counts, act)
            lo_deg, hi_deg = lim["lower"] * 180.0 / math.pi, lim["upper"] * 180.0 / math.pi

            print(f"  {len(pos)} reads, steady within {span} count(s)")
            print(f"  position : {counts} counts")
            print(f"  joint    : {deg:+.2f} deg  (direction {act['direction']:+d}, "
                  f"zero {act['motor_counts_at_zero']})")
            print(f"  current limits in joint_parameters.yaml: "
                  f"{lo_deg:+.1f} .. {hi_deg:+.1f} deg\n")

            end = "upper" if deg >= 0 else "lower"
            value = limit_from_here(deg)
            if (value >= 0) != (deg >= 0) or value == 0.0:
                sys.exit(f"the joint is only {deg:+.2f} deg from zero, so there is "
                         f"no sensible limit to take from here - a limit this "
                         f"close to zero would leave the joint almost no travel. "
                         f"Is it posed where you meant?")

            print(f"  -> this becomes the {end.upper()} limit of {joint}")
            print(f"     measured {deg:+.2f}, less 5 deg margin, rounded toward "
                  f"zero: {value:+.1f} deg")
            print(f"\n  for config/joint_parameters.yaml:")
            print(f"    {joint}:")
            print(f"      {end}: {value * math.pi / 180.0:.6f}   "
                  f"# {value:+.0f} deg [HARNESS] measured by hand, torque off, "
                  f"at the point the loop went taut ({counts} ct)")
            if end == "upper":
                print(f"      lower: {lim['lower']:.6f}   # unchanged - measure "
                      f"this end too by rolling the other way")
            else:
                print(f"      upper: {lim['upper']:.6f}   # unchanged - measure "
                      f"this end too by rolling the other way")
            print(f"\n  NOT written automatically: a limit is the one number "
                  f"that stops the arm hurting itself,")
            print(f"  so it goes in the file by hand, with its provenance "
                  f"beside it.")
            return

        if a.recentre:
            # Torque must be OFF: the offset has to be measured against where the
            # joint physically rests, not against a position a loop is holding.
            w.send("D")
            time.sleep(0.3)
            for sid in a.recentre:
                w.send(f"C {sid}")
                line = wait_line(w, "##", 6)
                if line is None:
                    sys.exit(f"id {sid}: no reply to the re-centre command")
                time.sleep(0.3)
            time.sleep(0.5)
            w.send("Q")
            time.sleep(1.0)
            w.send("?")
            time.sleep(0.8)
            return

        if getattr(a, "range") > 0:
            # Torque stays OFF: the joint is moved by hand, and a servo holding
            # position would fight you and hide its own travel.
            secs = getattr(a, "range")
            w.send("B 20")
            time.sleep(0.4)
            w.frames.clear()
            print(f"  {secs:.0f} s, torque OFF. Move each wrist joint BY HAND to "
                  f"both stops - gently, and one joint at a time.")
            print(f"  {'':8s}{'counts':>18s}{'min':>12s}{'max':>12s}{'travel':>12s}")
            k = 360.0 / 4096.0
            lo = [None] * 3
            hi = [None] * 3
            t0 = time.time()
            nxt = 0.0
            while time.time() - t0 < secs:
                time.sleep(0.1)
                for f in w.frames:
                    for i in range(3):
                        v = f[2][i]
                        if v == 0:          # no reply recorded in that frame
                            continue
                        lo[i] = v if lo[i] is None else min(lo[i], v)
                        hi[i] = v if hi[i] is None else max(hi[i], v)
                w.frames.clear()
                if time.time() - t0 >= nxt:
                    nxt += 2.0
                    for i, sid in enumerate(IDS):
                        if lo[i] is None:
                            continue
                        print(f"  id {sid}  {lo[i]:5d}..{hi[i]:5d} ct"
                              f"  {lo[i]*k:8.1f}°  {hi[i]*k:8.1f}°"
                              f"  {(hi[i]-lo[i])*k:8.1f}°")
                    print()
            print("  FINAL - for config/joint_parameters.yaml:")
            for i, sid in enumerate(IDS):
                if lo[i] is None:
                    print(f"    id {sid}: never answered"); continue
                span = (hi[i] - lo[i]) * k
                print(f"    id {sid}: {lo[i]}..{hi[i]} counts"
                      f"  = {span:.1f}° of travel")
                # A span filling almost the whole 0..4095 is the single-turn
                # encoder having WRAPPED, not two stops found: min/max then
                # straddle the seam rather than the mechanism, so "1..4080 =
                # 358.5 deg" means no stop at all.
                if span > 330.0:
                    print(f"       ^^ that is a WRAP, not two stops. This joint "
                          f"turned past the encoder's 0/4095 seam, so it has NO "
                          f"mechanical stop in at least one direction and these "
                          f"min/max numbers describe the seam, not the mechanism. "
                          f"Its usable limit is set by the single-turn encoder "
                          f"(and by any cable it winds), NOT measured here.")
                elif span < 2.0:
                    print(f"       ^^ only {span:.1f}° - this joint was almost "
                          f"certainly NOT moved during the run, so this is not a "
                          f"travel measurement. Move one joint at a time.")
            return

        if a.watch > 0:
            # Nothing is energised on purpose: hunt a cable fault with the arm
            # limp. Dropouts are counted from the firmware's own per-servo miss
            # counters, because a failed read leaves the last good position and
            # voltage in place, so the telemetry values cannot show a miss.
            import re
            def misses():
                m0 = len(w.lines)
                w.send("?")
                end = time.time() + 2.0
                while time.time() < end:
                    for line in w.lines[m0:]:
                        g = re.search(r"miss=(\d+)/(\d+)/(\d+)", line)
                        if g:
                            return [int(x) for x in g.groups()]
                    time.sleep(0.05)
                return None

            base = misses()
            if base is None:
                sys.exit("no status reply - is the board connected?")
            print(f"  watching for {a.watch:.0f} s. FLEX THE WIRING NOW, one "
                  f"segment at a time - near id3's connector, then id2's, then "
                  f"along each length. Report which movement does it.")
            t0 = time.time()
            last = base
            worst = [0, 0, 0]
            while time.time() - t0 < a.watch:
                time.sleep(0.5)
                cur = misses()
                if cur is None:
                    print(f"  t+{time.time()-t0:5.1f}s  the BOARD stopped answering")
                    continue
                for i in range(3):
                    d = cur[i] - last[i]
                    if d > 0:
                        worst[i] += d
                        print(f"  t+{time.time()-t0:5.1f}s  id {IDS[i]} missed "
                              f"{d} read(s)   <-- the flex you just made did this")
                last = cur
            print(f"\n  total missed reads over {a.watch:.0f} s: "
                  f"id1 {worst[0]}  id2 {worst[1]}  id3 {worst[2]}")
            if not any(worst):
                print("    nothing missed - flex harder, or the fault only shows "
                      "under the motion of a real move")
            return

        if a.measure > 0:
            w.send("B 50")
            w.beat = True
            time.sleep(0.3)
            w.send("E")
            # Judge by the status frame, not the "## ENABLED" text: once binary
            # frames stream, ASCII lines arrive interleaved and a text match is
            # unreliable. The frame carries the flag and fault code outright.
            end = time.time() + 4.0
            while time.time() < end:
                live = [f for f in w.frames if f[0] == 1]
                bad = [f for f in w.frames if f[1] != 0]
                if bad:
                    sys.exit(f"wrist faulted on enable, code {bad[-1][1]}")
                if live:
                    break
                time.sleep(0.05)
            else:
                sys.exit("no frame reported enabled within 4 s")
            w.frames.clear()
            print(f"  holding and measuring for {a.measure:.0f} s ...")
            time.sleep(a.measure)
            fr = [f for f in w.frames if f[0] == 1]
            if len(fr) < 20:
                sys.exit(f"only {len(fr)} frames - nothing to judge")
            k = 360.0 / 4096.0
            print(f"\n  {len(fr)} frames, supply {min(f[3] for f in fr):.1f} V min")
            print(f"  {'servo':6s} {'mean':>9s} {'p-p':>8s} {'counts':>7s}")
            for i, sid in enumerate(IDS):
                p = [f[2][i] for f in fr]
                span = max(p) - min(p)
                print(f"  id {sid:<3d} {sum(p)/len(p)*k:8.2f}° {span*k:7.3f}° {span:6d}")
    finally:
        w.close()

main()
