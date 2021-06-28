# Simple services (on/off, toggle) to control a GPIO pin

```
$ roslaunch pi_gpio pump_relay.launch
SUMMARY
========

PARAMETERS
 * /pump_relay/gpio_pin: 11
 * /pump_relay/timed_delay: 2

[INFO] [1624894043.075258]: [pump_relay] pin 11 (pump_relay) state is LOW
[INFO] [1624894043.077343]: [pump_relay] Ready to control...

$ rosservice list
/pump_relay/off
/pump_relay/on
/pump_relay/toggle
/pump_relay/timed
...

$ rosservice call /pump_relay/on
success: True
message: "pin 11 (pump_relay) state is HIGH"

$ rosservice call /pump_relay/off
success: True
message: "pin 11 (pump_relay) state is LOW"

$ rosservice call /pump_relay/toggle
success: True
message: "pin 11 (pump_relay) state is HIGH"

$ rosservice call /pump_relay/timed
success: True
message: "pin 11 (pump_relay) state is LOW"

```