# RC receiver control design

## Current control flow

`keyboard_control_node.py` publishes `geometry_msgs/Twist` to `/cmd_vel`.

`control_node.py` subscribes to `/cmd_vel`, clamps the command, sends steering to the ESP32 over Jetson UART, and sends duty to the VESC over USB.

So the RC transmitter/receiver should be another `/cmd_vel` publisher, not a replacement for `control_node.py`.

```text
RC transmitter -> RC receiver -> Arduino Nano USB -> rc_receiver_control_node.py
                                                   -> /cmd_vel
                                                   -> control_node.py
                                                   -> ESP32 steering + VESC duty
```

## Arduino-to-Jetson serial contract

Example Arduino Nano sketch:

```text
arduino_rc_receiver_reader.ino
```

The Jetson node accepts any of these line formats:

```text
1500,1498,1000,2000
CH1:1500 CH2:1498 CH3:1000 CH4:2000
ch1=1500,ch2=1498,ch3=1000,ch4=2000
```

Each value is a PWM pulse width in microseconds. Normal RC ranges are usually:

```text
low    ~= 1000 us
center ~= 1500 us
high   ~= 2000 us
```

## Channel discovery first

Run in probe mode first. This always publishes zero `/cmd_vel`, so the vehicle should not move from this node.

```bash
python3 rc_receiver_control_node.py --ros-args \
  -p port:=auto \
  -p baud:=115200 \
  -p probe_only:=true
```

Move one stick/switch at a time and watch which `CHx` changes:

```text
PROBE raw: CH1=1500 CH2=1498 CH3=1000 CH4=2000
```

Write down:

```text
steering left/right = CH1
throttle/VESC       = CH3
optional arm switch = CH?
```

## Driving mode

After channel mapping is known:

```bash
python3 rc_receiver_control_node.py --ros-args \
  -p port:=auto \
  -p baud:=115200 \
  -p probe_only:=false \
  -p steer_channel:=1 \
  -p throttle_channel:=3 \
  -p arm_channel:=0
```

If steering or throttle is reversed, flip only that axis:

```bash
-p invert_steer:=true
-p invert_throttle:=true
```

If using an arm switch:

```bash
-p arm_channel:=5 -p arm_threshold_us:=1700
```

## Safety behavior

`rc_receiver_control_node.py` sends zero command when:

- `probe_only` is true
- serial data stops for more than `cmd_timeout_sec` seconds
- the arm switch is configured but not active
- selected steering/throttle channels are missing

`control_node.py` still has its own `/cmd_vel` timeout and output clamps, so there are two safety layers.

## Current mapped behavior

- CH1 controls steering.
- CH1 `1500 us` is center.
- CH1 `1000 us` and `2000 us` are full left/right input.
- CH3 controls VESC throttle.
- CH3 uses `1500 us` as stop.
- CH3 `1000 us` is forward on this transmitter setup.
- CH3 `2000 us` is reverse on this transmitter setup.
- Full CH3 stick is intentionally slow: default `linear_cmd_max=0.25`, which becomes about `+/-0.05` VESC duty through `control_node.py`.
- Throttle is bidirectional and inverted by default for this transmitter setup.

## Suggested startup order

Connect Arduino Nano to the Jetson Nano with USB. The node now defaults to
`port:=auto`, so it first tries `/dev/serial/by-id/*` names like Arduino,
Nano, CH340, or USB-Serial, then falls back to `/dev/ttyUSB*` and
`/dev/ttyACM*`.

If auto picks the wrong device because VESC is also on USB, check:

```bash
ls -l /dev/serial/by-id/
```

Then pass the Arduino path directly:

```bash
python3 rc_receiver_control_node.py --ros-args \
  -p port:=/dev/serial/by-id/YOUR_ARDUINO_NAME \
  -p probe_only:=true
```

For CH340-based Nano clones, `lsusb` should show something like
`QinHeng Electronics CH340 serial converter`, and a matching `/dev/ttyUSB*`
or `/dev/serial/by-id/*` entry should also appear. If `lsusb` shows CH340 but
no tty device appears, the Jetson kernel is missing or not loading the `ch341`
driver.

On this Jetson kernel, this error means CH340 USB cannot be used directly:

```text
modprobe: FATAL: Module ch341 not found in directory /lib/modules/5.15.148-tegra
```

Practical alternatives:

- Use an Arduino board that appears as `/dev/ttyACM*`.
- Use the existing CP2102 USB-to-UART adapter and connect it to the Nano UART pins.
- Install/build a matching `ch341` kernel module for the exact Jetson kernel.

For the CP2102 adapter path, wire:

```text
CP2102 GND -> Nano GND
CP2102 RXD -> Nano TX1
CP2102 TXD -> Nano RX0
```

Then run the ROS node with the CP2102 port:

```bash
python3 rc_receiver_control_node.py --ros-args \
  -p port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
  -p baud:=115200 \
  -p probe_only:=true
```

Terminal 1:

```bash
python3 control_node.py
```

Terminal 2, first with wheels off the ground:

```bash
python3 rc_receiver_control_node.py --ros-args -p probe_only:=true
```

Then, after mapping channels:

```bash
python3 rc_receiver_control_node.py --ros-args \
  -p probe_only:=false \
  -p steer_channel:=1 \
  -p throttle_channel:=3
```
