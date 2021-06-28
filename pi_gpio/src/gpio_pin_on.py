#!/usr/bin/env python

import RPi.GPIO as GPIO

GPIO.setwarnings(False)

# to use Raspberry Pi board pin numbers
GPIO.setmode(GPIO.BOARD)

# set up the GPIO channels - one input and one output
GPIO.setup(11, GPIO.OUT)
GPIO.output(11, GPIO.HIGH)
