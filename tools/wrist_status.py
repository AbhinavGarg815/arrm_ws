#!/usr/bin/env python3
"""Ask the ESP32's production firmware for its per-servo report. Read-only.

    tools/wrist_status.py

NEVER run this while the ros2_control stack is up: two readers on one tty split
the bytes between them, so this steals the status frames the hardware interface
is parsing, and opening the port can reset the board. While the stack is up,
joint positions come from /joint_states.

Prints per servo: position, goal, load, travel limits and supply voltage - the
last distinguishing a dead 5 V sensing buck (pots read as wild angles, servos
still answer at ~12 V) from a disconnected pack (nothing answers).
"""
import time, serial

# The ESP32 board, by device identity rather than by USB socket. The CP2102
# reports the generic serial "0001", so this is unique only while it is the only
# CP2102 attached.
PORT = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"

def framed(body):
    v = 0
    for ch in body.encode():
        v ^= ch
    return f"{body}*{v:02X}\n"

s = serial.Serial()
s.port, s.baudrate, s.timeout = PORT, 115200, 0.2
s.dtr = s.rts = False          # keep the auto-reset circuit from rebooting it
s.open()
time.sleep(1.5)
s.reset_input_buffer()
s.write(framed("?").encode())
s.flush()
time.sleep(0.4)
s.write(framed("Q").encode())    # each servo's own Kp/Kd/Ki and deadbands
s.flush()
t0 = time.time()
while time.time() - t0 < 3.0:
    line = s.readline().decode(errors="replace").strip()
    if line:
        print(" ", line)
s.close()
