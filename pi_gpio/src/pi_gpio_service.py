#!/usr/bin/env python

import rospy
import time
from std_srvs.srv import Trigger, TriggerResponse
import RPi.GPIO as GPIO

NODE_ID="pi_gpio"
GPIO_PIN = 11
PIN_STATE = GPIO.LOW
TIMED_DELAY = 5 # seconds

# ================================================================================
# GPIO Pin Management
# ================================================================================

def pinState():
    global GPIO_PIN
    return GPIO.input(GPIO_PIN)

def pinOn():
    global GPIO_PIN
    GPIO.output(GPIO_PIN, GPIO.HIGH)

def pinOff():
    global GPIO_PIN
    GPIO.output(GPIO_PIN, GPIO.LOW)

def pinStatusMessage():
    return pinStatusMessageFromState(pinState())

def pinStatusMessageFromState(pinState):
    global GPIO_PIN
    status="unknown"
    if(pinState==GPIO.HIGH):
        status="HIGH"
    if(pinState==GPIO.LOW):
        status="LOW"
    return "pin "+str(GPIO_PIN)+" ("+NODE_ID+") state is "+status


# ================================================================================
# ROS Callbacks
# ================================================================================

def log(s):
    global NODE_ID
    #print(s)
    rospy.loginfo('['+NODE_ID+'] '+s)

def on_callback(request):
    pinOn()
    state=pinState()
    m = pinStatusMessageFromState(state)
    log(m)
    return TriggerResponse(
        success=(state==GPIO.HIGH),
        message=m
    )

def off_callback(request):
    pinOff()
    state = pinState()
    m = pinStatusMessageFromState(state)
    log(m)
    return TriggerResponse(
        success=(state==GPIO.LOW),
        message=m
    )

def toggle_callback(request):
    stateBefore = pinState()
    if(stateBefore==GPIO.LOW):
        pinOn()
    else:
        pinOff()
    stateAfter = pinState()

    m = pinStatusMessageFromState(stateAfter)
    log(m)
    return TriggerResponse(
        success=(stateBefore!=stateAfter),
        message=m
    )

def timed_callback(request):
    global TIMED_DELAY
    stateBefore = pinState()
    if(stateBefore!=GPIO.LOW):
        log("pin must be LOW when timed service is triggered")

    pinOn()
    stateOn = pinState()
    time.sleep(TIMED_DELAY)
    pinOff()
    stateOff = pinState()

    m = pinStatusMessageFromState(stateOff)
    log(m)
    return TriggerResponse(
        success=(stateOn==GPIO.HIGH and stateOff==GPIO.LOW),
        message=m
    )

# ================================================================================
# Main
# ================================================================================

def main():
    global NODE_ID,GPIO_PIN,TIMED_DELAY

    GPIO.setwarnings(False)
    # to use Raspberry Pi board pin numbers
    GPIO.setmode(GPIO.BOARD)

    rospy.init_node(NODE_ID, anonymous=True)
    NODE_ID = rospy.get_name()
    NODE_ID = NODE_ID[1:]

    GPIO_PIN = rospy.get_param('/'+NODE_ID+'/gpio_pin', 11)
    TIMED_DELAY = rospy.get_param('/'+NODE_ID+'/timed_delay', 5)

    # set up the GPIO channels - one input and one output
    GPIO.setup(GPIO_PIN, GPIO.OUT)
    pinOff()

    gpio_toggle = rospy.Service('/'+NODE_ID+'/toggle', Trigger, toggle_callback)
    gpio_on = rospy.Service('/'+NODE_ID+'/on', Trigger, on_callback)
    gpio_off = rospy.Service('/'+NODE_ID+'/off', Trigger, off_callback)
    gpio_timed = rospy.Service('/'+NODE_ID+'/timed', Trigger, timed_callback)

    log(pinStatusMessage())
    log("Ready to control...")

    rospy.spin()

if __name__ == "__main__":
    #try:
    main()
    #except rospy.ROSInterruptException:
    #    pass