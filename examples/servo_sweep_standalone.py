#!/usr/bin/env python3
"""
Standalone servo sweep using RPi.GPIO software PWM. No ROS2 required.

Drives a hobby servo on BCM GPIO 21 at 50 Hz and sweeps 0 -> 180 -> 0.
Useful as a bench sanity check before bringing up servo_pwm_service.

Wiring:
  Signal  -> GPIO 21 (physical pin 40)
  GND     -> Pi GND  (physical pin 39)
  Servo + -> EXTERNAL 5-6V supply (NOT Pi 5V)
  Supply GND tied to Pi GND (common ground is mandatory).

Usage:
  sudo python3 servo_sweep_standalone.py                  # default sweep
  sudo python3 servo_sweep_standalone.py --pin 18         # different pin
  sudo python3 servo_sweep_standalone.py --angle 90       # hold one angle
  sudo python3 servo_sweep_standalone.py --cycles 5       # more cycles
"""
import argparse
import signal
import sys
import time

import RPi.GPIO as GPIO

SERVO_FREQ_HZ = 50
FRAME_MS = 1000.0 / SERVO_FREQ_HZ

MIN_PULSE_MS = 0.5
MAX_PULSE_MS = 2.5


def angle_to_duty(angle_deg: float) -> float:
    angle_deg = max(0.0, min(180.0, angle_deg))
    pulse_ms = MIN_PULSE_MS + (MAX_PULSE_MS - MIN_PULSE_MS) * (angle_deg / 180.0)
    return (pulse_ms / FRAME_MS) * 100.0


def sweep(pwm, start, end, step, dwell_s):
    rng = range(start, end + (1 if step > 0 else -1), step)
    for angle in rng:
        pwm.ChangeDutyCycle(angle_to_duty(angle))
        time.sleep(dwell_s)


def main():
    p = argparse.ArgumentParser(description='Standalone RPi.GPIO servo sweep')
    p.add_argument('--pin', type=int, default=21, help='BCM pin number (default 21)')
    p.add_argument('--angle', type=int, default=None, help='Hold at a fixed angle instead of sweeping')
    p.add_argument('--cycles', type=int, default=3, help='Sweep cycles (default 3)')
    p.add_argument('--step', type=int, default=2, help='Angle step in degrees (default 2)')
    p.add_argument('--dwell', type=float, default=0.02, help='Seconds per step (default 0.02)')
    args = p.parse_args()

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(args.pin, GPIO.OUT)

    pwm = GPIO.PWM(args.pin, SERVO_FREQ_HZ)
    pwm.start(0)

    def cleanup(*_):
        print('\nCleaning up...')
        try:
            pwm.ChangeDutyCycle(0)
            time.sleep(0.1)
            pwm.stop()
        finally:
            GPIO.cleanup(args.pin)
        sys.exit(0)

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    print(f'Servo on BCM {args.pin} @ {SERVO_FREQ_HZ} Hz, pulses {MIN_PULSE_MS}-{MAX_PULSE_MS} ms')
    try:
        if args.angle is not None:
            a = max(0, min(180, args.angle))
            print(f'Holding at {a} deg. Ctrl-C to exit.')
            pwm.ChangeDutyCycle(angle_to_duty(a))
            while True:
                time.sleep(1.0)
        else:
            print(f'Full sweep 0..180, {args.cycles} cycles')
            pwm.ChangeDutyCycle(angle_to_duty(0))
            time.sleep(0.5)
            for c in range(args.cycles):
                print(f'Cycle {c + 1}/{args.cycles}: 0 -> 180')
                sweep(pwm, 0, 180, args.step, args.dwell)
                print(f'Cycle {c + 1}/{args.cycles}: 180 -> 0')
                sweep(pwm, 180, 0, -args.step, args.dwell)
            pwm.ChangeDutyCycle(angle_to_duty(90))
            time.sleep(0.5)
            pwm.ChangeDutyCycle(0)
            time.sleep(0.2)
    finally:
        cleanup()


if __name__ == '__main__':
    main()
