#!/usr/bin/env python3
"""
Standalone servo sweep using pigpio. No ROS2 required.

Drives a hobby servo on BCM GPIO 21 via DMA-driven PWM. pigpio generates
the servo pulses in the Pi's hardware DMA controller, so timing is not
affected by CPU load or kernel scheduling -- unlike RPi.GPIO software
PWM, which is a userspace thread and jitters under load.

Useful as a bench sanity check before bringing up servo_pwm_service, and
as an A/B comparison against the RPi.GPIO example (same sweep, visibly
steadier under load).

Prerequisites on the drone:
  sudo apt install -y pigpio python3-pigpio
  sudo systemctl enable --now pigpiod

Wiring:
  Signal  -> GPIO 21 (physical pin 40)
  GND     -> Pi GND  (physical pin 39)
  Servo + -> EXTERNAL 5-6V supply (NOT Pi 5V)
  Supply GND tied to Pi GND (common ground is mandatory).

Usage:
  python3 servo_sweep_standalone.py                  # full 0..180 sweep
  python3 servo_sweep_standalone.py --pin 18         # different BCM pin
  python3 servo_sweep_standalone.py --angle 90       # hold one angle
  python3 servo_sweep_standalone.py --cycles 5       # more cycles
  python3 servo_sweep_standalone.py --host otherhost # remote pigpiod

Note: unlike the RPi.GPIO example, this does NOT need to run as root.
pigpio's privilege lives in the daemon, not the client.
"""
import argparse
import signal
import sys
import time

import pigpio

# Servo pulse width endpoints in microseconds. Typical hobby servos
# accept 500..2500 us for a full 0..180 degree sweep. Narrow the range
# if your servo buzzes or strains at the extremes.
MIN_PULSE_US = 500
MAX_PULSE_US = 2500


def angle_to_pulse_us(angle_deg: float) -> int:
    angle_deg = max(0.0, min(180.0, angle_deg))
    return int(MIN_PULSE_US + (MAX_PULSE_US - MIN_PULSE_US) * (angle_deg / 180.0))


def sweep(pi: pigpio.pi, pin: int, start: int, end: int, step: int, dwell_s: float) -> None:
    rng = range(start, end + (1 if step > 0 else -1), step)
    for angle in rng:
        pi.set_servo_pulsewidth(pin, angle_to_pulse_us(angle))
        time.sleep(dwell_s)


def main() -> int:
    p = argparse.ArgumentParser(description='Standalone pigpio servo sweep')
    p.add_argument('--pin', type=int, default=21, help='BCM pin number (default 21)')
    p.add_argument('--host', type=str, default='localhost', help='pigpiod host (default localhost)')
    p.add_argument('--angle', type=int, default=None, help='Hold at a fixed angle instead of sweeping')
    p.add_argument('--cycles', type=int, default=3, help='Sweep cycles (default 3)')
    p.add_argument('--step', type=int, default=2, help='Angle step in degrees (default 2)')
    p.add_argument('--dwell', type=float, default=0.02, help='Seconds per step (default 0.02)')
    args = p.parse_args()

    pi = pigpio.pi(args.host)
    if not pi.connected:
        print(f'ERROR: could not connect to pigpiod on {args.host}.', file=sys.stderr)
        print('Is pigpiod running? Try: sudo systemctl enable --now pigpiod', file=sys.stderr)
        return 1

    def cleanup(*_):
        print('\nCleaning up...')
        try:
            pi.set_servo_pulsewidth(args.pin, 0)
        finally:
            pi.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    print(f'Servo on BCM {args.pin} via pigpiod @ {args.host}')
    print(f'Pulse range: {MIN_PULSE_US}-{MAX_PULSE_US} us')

    try:
        if args.angle is not None:
            a = max(0, min(180, args.angle))
            pulse = angle_to_pulse_us(a)
            print(f'Holding at {a} deg ({pulse} us). Ctrl-C to exit.')
            pi.set_servo_pulsewidth(args.pin, pulse)
            while True:
                time.sleep(1.0)
        else:
            print(f'Full sweep 0..180, {args.cycles} cycles, step={args.step}, dwell={args.dwell}s')
            pi.set_servo_pulsewidth(args.pin, angle_to_pulse_us(0))
            time.sleep(0.5)
            for c in range(args.cycles):
                print(f'Cycle {c + 1}/{args.cycles}: 0 -> 180')
                sweep(pi, args.pin, 0, 180, args.step, args.dwell)
                print(f'Cycle {c + 1}/{args.cycles}: 180 -> 0')
                sweep(pi, args.pin, 180, 0, -args.step, args.dwell)
            print('Parking at 90 deg, then releasing signal.')
            pi.set_servo_pulsewidth(args.pin, angle_to_pulse_us(90))
            time.sleep(0.5)
            pi.set_servo_pulsewidth(args.pin, 0)
            time.sleep(0.2)
    finally:
        cleanup()

    return 0


if __name__ == '__main__':
    main()
