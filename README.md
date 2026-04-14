# DEXI GPIO

ROS2 package for controlling Raspberry Pi GPIO pins on DEXI drone.

## Features

- GPIO reader node for monitoring input pins
- GPIO writer service for controlling output pins
- Servo PWM service for driving a hobby servo via pigpio (DMA-timed, jitter-free)
- Configurable pin assignments via YAML config
- Example client and subscriber code

## Nodes

### gpio_reader
Publishes GPIO pin states to ROS2 topics.

**Parameters:**
- `pin`: GPIO pin number to read

**Publishers:**
- `gpio_input_{pin}` (std_msgs/Bool): Pin state

### gpio_writer_service
Provides service interface for writing to GPIO output pins.

**Parameters:**
- `gpio_outputs`: Array of GPIO pin numbers to configure as outputs

**Services:**
- `write_gpio_{pin}` (dexi_interfaces/GPIOSend): Set pin state

### servo_pwm_service
Drives a single hobby servo through the **pigpio daemon**, which generates
servo pulses via DMA in the Pi's hardware timer rather than a userspace
thread. Timing is stable under CPU load — unlike `RPi.GPIO` software PWM
which visibly jitters when the rest of the DEXI stack is busy. Advertises
the standard DEXI servo service, so any client that talks to
`dexi_cpp/servo_controller` (PCA9685 on Pi5/CM5) works against this node
unchanged. Intended for platforms that do not have a PCA9685, such as the
ARK CM4.

**Prerequisites (one-time, on the host):**

```bash
sudo apt install -y pigpio python3-pigpio
sudo systemctl enable --now pigpiod
```

`pigpiod` auto-starts on boot after this. Verify with:

```bash
systemctl status pigpiod
python3 -c "import pigpio; p = pigpio.pi(); print('connected:', p.connected); p.stop()"
```

**Parameters:**
- `servo_pin` (int, default `21`): BCM pin number to drive.
- `pigpiod_host` (str, default `localhost`): pigpiod host for remote-control
  scenarios. Usually left at the default.

**Services:**
- `/dexi/servo_control` (`dexi_interfaces/srv/ServoControl`): Set servo angle.
  - `pin` (int): must match `servo_pin`; the node rejects other pins.
  - `angle` (int): 0..180, clamped.
  - `min_pw` (int, µs): minimum pulse width. `0` means use the default (500 µs).
  - `max_pw` (int, µs): maximum pulse width. `0` means use the default (2500 µs).

> ℹ️ pigpio is DMA-timed and does not require root — the client runs as
> the regular `dexi` user and the daemon holds the privilege. Timing is
> stable under load, so this node is suitable for ground-station payload
> triggers *and* reasonably steady holds during flight. For multi-servo
> setups (> 1 servo) or absolute production-grade timing, a PCA9685 over
> I²C or routing through the flight controller's PWM outputs is still
> the preferred path.

## Usage

### Launch GPIO nodes
```bash
ros2 launch dexi_gpio gpio.launch.py
```

### Run example subscriber
```bash
ros2 run python3 examples/gpio_subscriber.py
```

### Run example service client
```bash
ros2 run python3 examples/gpio_service_call.py
```

### Launch the servo PWM service

First, make sure pigpiod is running (one-time, survives reboots):
```bash
sudo apt install -y pigpio python3-pigpio
sudo systemctl enable --now pigpiod
```

Then launch the node:
```bash
ros2 launch dexi_gpio servo_pwm.launch.py
```

The node will advertise `/dexi/servo_control` on BCM pin 21. To use a
different pin, edit `launch/servo_pwm.launch.py` or pass the parameter:

```bash
ros2 run dexi_gpio servo_pwm_service --ros-args -p servo_pin:=20
```

Call the service directly from the CLI:
```bash
ros2 service call /dexi/servo_control dexi_interfaces/srv/ServoControl \
  "{pin: 21, angle: 90, min_pw: 0, max_pw: 0}"
```

Or run one of the included examples:

```bash
# Open then close once -- mirrors the Node-RED flow, ideal for payload drops
python3 examples/servo_open_close.py

# Full 0..180..0 sweep via the ROS2 service
python3 examples/servo_sweep_ros_client.py

# Pure-Python bench sweep with no ROS2 at all (pigpio, no sudo needed)
python3 examples/servo_sweep_standalone.py
```

## Servo Wiring

For any of the servo examples or the `servo_pwm_service` node:

| Servo wire | Connects to | Notes |
|------------|-------------|-------|
| Signal     | BCM GPIO 21 (physical pin 40) | Or whichever pin `servo_pin` is set to |
| GND        | Pi GND (physical pin 39)      | Shared with the servo supply GND |
| V+         | **External 5–6 V supply**     | Do **not** power from Pi 5V; a stalling servo can reboot the Pi |

The servo supply ground **must** be tied to a Pi GND pin — a hobby servo's
signal line is interpreted relative to its own ground, and without a common
ground the signal is meaningless and the servo will appear not to move.

## Calling the servo service from Python

If your integration lives in a Python script or ROS2 node on the companion
computer, call `/dexi/servo_control` directly with `rclpy`. No rosbridge
needed — you're on the same ROS2 graph as the service.

### Prerequisites

1. `servo_pwm_service` running on the companion computer:
   ```bash
   ros2 launch dexi_gpio servo_pwm.launch.py
   ```
2. Your script's environment has sourced the DEXI workspace so
   `dexi_interfaces` is importable:
   ```bash
   source ~/dexi_ws/install/setup.bash
   ```

### Minimal open/close client

This is the entire file. Save as `my_servo.py`, run with `python3 my_servo.py`.

```python
import time
import rclpy
from rclpy.node import Node
from dexi_interfaces.srv import ServoControl

class ServoClient(Node):
    def __init__(self):
        super().__init__('my_servo_client')
        self.client = self.create_client(ServoControl, '/dexi/servo_control')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for /dexi/servo_control...')

    def set_angle(self, angle: int):
        req = ServoControl.Request()
        req.pin = 21       # must match servo_pwm_service's servo_pin param
        req.angle = angle  # 0..180
        req.min_pw = 0     # 0 -> node default (500 us)
        req.max_pw = 0     # 0 -> node default (2500 us)
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main():
    rclpy.init()
    node = ServoClient()
    try:
        node.set_angle(150)   # open
        time.sleep(2.0)
        node.set_angle(30)    # close
        time.sleep(2.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Request fields

- `pin` (int): must match the node's `servo_pin` parameter — the node
  rejects any other pin with `success: false` and a clear message.
- `angle` (int): 0..180, clamped. For an open/close mechanism, pick two
  values with margin from the endpoints, e.g. `30` for closed and `150`
  for open.
- `min_pw` / `max_pw` (int, µs): pulse-width overrides. Send `0` for both
  to use the node defaults (500 / 2500 µs). Override only if your servo
  needs a narrower or wider range.

### Ready-to-run example

A complete one-shot open/close script is provided at
[`examples/servo_open_close.py`](examples/servo_open_close.py) — same
logic as above, with more comments and logging. Run it directly:

```bash
python3 examples/servo_open_close.py
```

For a full 0°..180°..0° sweep through the same service, use
[`examples/servo_sweep_ros_client.py`](examples/servo_sweep_ros_client.py):

```bash
python3 examples/servo_sweep_ros_client.py
```

### Calling from the shell instead

If you just want to fire a single command without writing any Python at
all, `ros2 service call` works:

```bash
# open
ros2 service call /dexi/servo_control dexi_interfaces/srv/ServoControl \
  "{pin: 21, angle: 150, min_pw: 0, max_pw: 0}"

# close
ros2 service call /dexi/servo_control dexi_interfaces/srv/ServoControl \
  "{pin: 21, angle: 30, min_pw: 0, max_pw: 0}"
```

## Calling the servo service from Node-RED

For competitions or ground-station automation, you can drive the servo from
Node-RED without touching the DEXI web GCS. The servo service is reachable
through the same `rosbridge_websocket` that the GCS already uses, so no extra
ROS2 packages or Node-RED palettes are required — just the built-in websocket
nodes.

### Prerequisites

1. `servo_pwm_service` running on the companion computer:
   ```bash
   ros2 launch dexi_gpio servo_pwm.launch.py
   ```
2. `rosbridge_websocket` running on port 9090 (the standard DEXI bringup
   already does this). Confirm with:
   ```bash
   ros2 service list | grep servo_control
   ros2 service type /dexi/servo_control
   # -> dexi_interfaces/srv/ServoControl
   ```

### rosbridge call_service JSON

Every command is a single JSON message sent over the websocket. This is the
raw rosbridge protocol, which `roslibjs` (used by the DEXI web GCS) sends
under the hood:

```json
{
  "op": "call_service",
  "service": "/dexi/servo_control",
  "type": "dexi_interfaces/srv/ServoControl",
  "args": { "pin": 21, "angle": 150, "min_pw": 0, "max_pw": 0 }
}
```

- `pin` must match the `servo_pin` parameter the node was launched with.
- `angle` is 0..180. For an open/close mechanism, pick two values with
  margin from the endpoints, e.g. `30` for closed and `150` for open.
- `min_pw` / `max_pw` of `0` tell the node to use its defaults
  (500 µs / 2500 µs). Override only if your servo needs a narrower or
  wider range.
- The `"type"` field is required on first call so rosbridge can look up
  the service definition. Always include it.

### Minimal Node-RED flow

1. Drag in a **`websocket out`** node. Edit → Type: *Connect to*, URL:
   `ws://<companion-ip>:9090` (or `ws://localhost:9090` if Node-RED runs
   on the companion itself), Send/receive: *payload*.
2. Add an **`inject`** node labeled "Open".
3. Add a **`function`** node wired after the inject with:
   ```javascript
   msg.payload = JSON.stringify({
       op: "call_service",
       service: "/dexi/servo_control",
       type: "dexi_interfaces/srv/ServoControl",
       args: { pin: 21, angle: 150, min_pw: 0, max_pw: 0 }
   });
   return msg;
   ```
4. Wire the function into the websocket out node.
5. Duplicate the inject + function, label the copy "Close", and change
   `angle: 150` to `angle: 30`. Wire it into the same websocket out node.
6. Optional: drag a **`websocket in`** node pointing at the same URL and
   wire it to a **`debug`** node to see `service_response` replies from
   rosbridge after every call.

### Ready-to-import flow

A complete importable flow is provided at
[`examples/node_red_servo_flow.json`](examples/node_red_servo_flow.json).
Import it via **Menu → Import → select file**, then edit the websocket
client config once to set the correct `ws://<host>:9090` URL.

### Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| Node fails to start with "Could not connect to pigpiod" | `pigpiod` daemon is not running | `sudo systemctl enable --now pigpiod` and confirm with `systemctl status pigpiod` |
| `service_response` with `success: false`, "Pin X not configured" | The `pin` you sent does not match the node's `servo_pin` param | Send `pin: 21` (or whatever the node was launched with) |
| rosbridge log: "Unable to look up service type" | The `"type"` field is missing from your JSON | Always include `"type": "dexi_interfaces/srv/ServoControl"` |
| No response at all | Websocket not connected; check the Node-RED node status | Confirm the URL, confirm `rosbridge_websocket` is running on port 9090 |
| Servo buzzes but does not move | No common ground between servo supply and Pi | Tie the servo supply GND to a Pi GND pin |
| Servo drifts or oscillates at rest | Pulse width at the edge of the servo's usable range | Narrow `min_pw`/`max_pw` in the request (try 600/2400 or 700/2300) |

## Configuration

Edit `config/gpio.yaml` to configure which pins are used as inputs/outputs.

## Dependencies

- rclpy
- dexi_interfaces
- std_msgs
- RPi.GPIO (for gpio_reader and gpio_writer_service)
- pigpio + python3-pigpio (for servo_pwm_service; install via apt)

## License

MIT License