# DEXI GPIO

ROS2 package for controlling Raspberry Pi GPIO pins on DEXI drone.

## Features

- GPIO reader node for monitoring input pins
- GPIO writer service for controlling output pins
- Servo PWM service for driving a hobby servo via software PWM (bench/demo use)
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
Drives a single hobby servo via `RPi.GPIO` software PWM. Advertises the
standard DEXI servo service, so any client that talks to `dexi_cpp/servo_controller`
(PCA9685 on Pi5/CM5) works against this node unchanged. Intended for
platforms that do not have a PCA9685, such as the ARK CM4.

**Parameters:**
- `servo_pin` (int, default `21`): BCM pin number to drive.

**Services:**
- `/dexi/servo_control` (`dexi_interfaces/srv/ServoControl`): Set servo angle.
  - `pin` (int): must match `servo_pin`; the node rejects other pins.
  - `angle` (int): 0..180, clamped.
  - `min_pw` (int, µs): minimum pulse width. `0` means use the default (500 µs).
  - `max_pw` (int, µs): maximum pulse width. `0` means use the default (2500 µs).

> ⚠️ Software PWM timing jitters under CPU load. This is fine for bench
> demos and ground-station payload triggers, but not recommended for
> in-flight use where the servo must hold a precise position. For production
> servo control, use `dexi_cpp/servo_controller` with a PCA9685, or route
> the servo through the flight controller's PWM outputs via MAVLink.

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

# Pure-Python bench sweep with no ROS2 at all
sudo python3 examples/servo_sweep_standalone.py
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
| `service_response` with `success: false`, "Pin X not configured" | The `pin` you sent does not match the node's `servo_pin` param | Send `pin: 21` (or whatever the node was launched with) |
| rosbridge log: "Unable to look up service type" | The `"type"` field is missing from your JSON | Always include `"type": "dexi_interfaces/srv/ServoControl"` |
| No response at all | Websocket not connected; check the Node-RED node status | Confirm the URL, confirm `rosbridge_websocket` is running on port 9090 |
| Servo buzzes but does not move | No common ground between servo supply and Pi | Tie the servo supply GND to a Pi GND pin |
| Servo twitches randomly | Software-PWM jitter under CPU load | Expected on busy systems — for flight use switch to PCA9685 or FC PWM |

## Configuration

Edit `config/gpio.yaml` to configure which pins are used as inputs/outputs.

## Dependencies

- rclpy
- dexi_interfaces
- std_msgs
- RPi.GPIO

## License

MIT License