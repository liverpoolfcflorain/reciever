#!/usr/bin/env python3
"""
Raspberry Pi 3 Drone Receiver for KK2.1.5 Flight Controller
============================================================
Receives UDP control packets from laptop and generates 50Hz servo PWM
signals via pigpio (hardware-timed, accurate to ~1μs).

WIRING (KK2.1.5 → Pi 3):
------------------------
KK2.1.5 Receiver Header (left side, 3 pins per row):
  Edge → GND | VCC(+5V) | SIGNAL ← Inner

Connect ONLY:
  • ONE common GND wire: any KK2.1.5 GND → any Pi GND (e.g. Pin 6)
  • SIGNAL wires:
      Aileron (Roll)  → GPIO 17 (Pin 11)
      Elevator(Pitch) → GPIO 27 (Pin 13)
      Throttle        → GPIO 22 (Pin 15)
      Rudder  (Yaw)   → GPIO 23 (Pin 16)
      AUX             → GPIO 24 (Pin 18)

DO NOT connect KK2.1.5 VCC(+5V) to the Pi — the Pi is self-powered.
The KK2.1.5 is powered by its ESC BEC via the motor outputs.

NoIR Camera:
  • Connect via CSI ribbon cable (port next to HDMI). No GPIO used.
"""

import socket
import json
import subprocess
import sys
import time
import signal
import os
import pigpio

# ─── CONFIGURATION ──────────────────────────────────────────
PINS = {
    'throttle': 22,   # Pin 15
    'roll':     17,   # Pin 11  (Aileron on KK2.1.5)
    'pitch':    27,   # Pin 13  (Elevator on KK2.1.5)
    'yaw':      23,   # Pin 16  (Rudder on KK2.1.5)
    'aux':      24,   # Pin 18
}

PWM_FREQ  = 50          # Hz — standard RC servo/update rate
MIN_US    = 1000        # microseconds — low endpoint
MAX_US    = 2000        # microseconds — high endpoint
MID_US    = 1500        # microseconds — neutral

UDP_PORT      = 5005
VIDEO_PORT    = 5000
LAPTOP_IP     = "192.168.4.2"   # Change if your laptop gets a different IP
VIDEO_WIDTH   = 640
VIDEO_HEIGHT  = 480
VIDEO_FPS     = 30

FAILSAFE_MS   = 500     # Drop throttle if no packet for 500ms

# ─── GLOBALS ────────────────────────────────────────────────
pi = None
sock = None
video_procs = []
last_packet_time = 0
failsafe_active = False
running = True

# ─── SIGNAL HANDLER ───────────────────────────────────────────
def shutdown(signum, frame):
    global running
    print("\n[SHUTDOWN] Stopping receiver...")
    running = False

signal.signal(signal.SIGINT, shutdown)
signal.signal(signal.SIGTERM, shutdown)

# ─── PWM INITIALIZATION ─────────────────────────────────────
def init_pwm():
    global pi
    pi = pigpio.pi()
    if not pi.connected:
        print("[FATAL] pigpio daemon not running. Start it: sudo pigpiod")
        sys.exit(1)

    for name, pin in PINS.items():
        pi.set_mode(pin, pigpio.OUTPUT)
        pi.set_servo_pulsewidth(pin, MID_US)
        print(f"  [OK] {name:8s} → GPIO {pin:2d}  ({MID_US}us)")

# ─── FAILSAFE ───────────────────────────────────────────────
def trigger_failsafe():
    global failsafe_active
    if not failsafe_active:
        print("[FAILSAFE] Signal lost — dropping throttle to minimum!")
        failsafe_active = True
    pi.set_servo_pulsewidth(PINS['throttle'], MIN_US)

# ─── VIDEO STREAM ───────────────────────────────────────────
def start_video_stream():
    """Launch low-latency H.264 UDP stream using libcamera (modern Pi OS).
    If libcamera is not available, falls back to legacy raspivid."""

    # Detect which camera stack is available
    libcamera_avail = os.system("which libcamera-vid > /dev/null 2>&1") == 0

    if libcamera_avail:
        vid_cmd = [
            "libcamera-vid", "-t", "0", "--codec", "h264",
            "--width", str(VIDEO_WIDTH), "--height", str(VIDEO_HEIGHT),
            "--framerate", str(VIDEO_FPS),
            "--inline", "--annotate", "12",   # show timestamp
            "-o", "-"
        ]
    else:
        print("[WARN] libcamera not found, trying legacy raspivid...")
        vid_cmd = [
            "raspivid", "-t", "0", "-w", str(VIDEO_WIDTH),
            "-h", str(VIDEO_HEIGHT), "-fps", str(VIDEO_FPS),
            "-b", "1000000", "-pf", "high", "-o", "-"
        ]

    gst_cmd = [
        "gst-launch-1.0", "fdsrc", "!", "h264parse", "!", "queue", "!",
        "rtph264pay", "config-interval=1", "pt=96", "!", "gdppay", "!",
        "udpsink", f"host={LAPTOP_IP}", f"port={VIDEO_PORT}", "sync=false"
    ]

    try:
        vid_proc = subprocess.Popen(vid_cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        gst_proc = subprocess.Popen(gst_cmd, stdin=vid_proc.stdout, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        video_procs.extend([vid_proc, gst_proc])
        print(f"  [OK] Video stream → {LAPTOP_IP}:{VIDEO_PORT} ({VIDEO_WIDTH}x{VIDEO_HEIGHT}@{VIDEO_FPS}fps)")
    except Exception as e:
        print(f"  [WARN] Could not start video: {e}")

# ─── UDP CONTROL LOOP ───────────────────────────────────────
def control_loop():
    global last_packet_time, failsafe_active

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", UDP_PORT))
    sock.settimeout(0.1)  # 100ms blocking max

    print(f"\n{'='*55}")
    print(f"  Drone Receiver Ready")
    print(f"  Listening UDP port {UDP_PORT}")
    print(f"  Press Ctrl+C to stop")
    print(f"{'='*55}\n")

    last_packet_time = time.time() * 1000

    while running:
        try:
            data, addr = sock.recvfrom(1024)
            now = time.time() * 1000
            last_packet_time = now

            if failsafe_active:
                print("[RECOVER] Signal restored")
                failsafe_active = False

            try:
                cmd = json.loads(data.decode("utf-8"))
            except json.JSONDecodeError:
                continue

            for name, pin in PINS.items():
                val = float(cmd.get(name, 0.5))
                val = max(0.0, min(1.0, val))
                pulse = int(MIN_US + val * (MAX_US - MIN_US))
                pi.set_servo_pulsewidth(pin, pulse)

        except socket.timeout:
            now = time.time() * 1000
            if (now - last_packet_time) > FAILSAFE_MS:
                trigger_failsafe()

        # Small sleep to prevent CPU spin if packets flood in
        time.sleep(0.001)

    sock.close()

# ─── CLEANUP ──────────────────────────────────────────────────
def cleanup():
    print("[CLEANUP] Stopping PWM and video...")
    for p in video_procs:
        try:
            p.terminate()
            p.wait(timeout=2)
        except Exception:
            pass
    if pi:
        for pin in PINS.values():
            pi.set_servo_pulsewidth(pin, 0)  # 0 = off
        pi.stop()
    print("[CLEANUP] Done.")

# ─── MAIN ─────────────────────────────────────────────────────
if __name__ == "__main__":
    print("Initializing PWM outputs...")
    init_pwm()
    start_video_stream()
    try:
        control_loop()
    finally:
        cleanup()
