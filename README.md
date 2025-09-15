# DEXI GPIO

ROS2 package for controlling Raspberry Pi GPIO pins on DEXI drone.

## Features

- GPIO reader node for monitoring input pins
- GPIO writer service for controlling output pins
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

## Configuration

Edit `config/gpio.yaml` to configure which pins are used as inputs/outputs.

## Dependencies

- rclpy
- dexi_interfaces
- std_msgs
- RPi.GPIO

## License

MIT License