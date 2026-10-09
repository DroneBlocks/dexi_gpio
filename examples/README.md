# Examples

Teaching scripts and flows. Not installed: run them from the source tree.

## Python

| Script | What it does |
|---|---|
| `python/gpio_service_call.py` | Write a DEXI GPIO output high and low through its service |
| `python/gpio_subscriber.py` | Subscribe to a DEXI GPIO pin's state |
| `python/servo_open_close.py` | Open then close a servo through `/dexi/servo_control` |
| `python/servo_sweep_ros_client.py` | Sweep a servo through `/dexi/servo_control` |
| `python/servo_sweep_standalone.py` | Sweep a servo with pigpio, no ROS 2 needed |

Run with `python3 examples/python/<script>` from the package directory, with the DEXI workspace sourced.

## Node-RED

| Flow | What it does |
|---|---|
| `node_red/node_red_servo_flow.json` | Servo control flow; import it from the Node-RED menu |
