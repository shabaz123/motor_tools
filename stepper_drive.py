####################################
# stepper drive.py
# rev 1 - October 2022 - shabaz
####################################


from machine import Pin
import time


################# config #####################
# swap DIR_POLARITY between True and False until motor turns in the correct direction
DIR_POLARITY = True
# set MICROSTEP to the servo driver setting
MICROSTEP = 2
# set STEPPER_DEG to suit the stepper motor angle per step
STEPPER_DEG = 1.8
# set GEARDIV to suit any gearing (set to 1 for no gearing).
# For example for 1:2 gearing down, set to 2
GEARDIV = 30

# set PULSE_RISING_EDGE to True if the driver uses pulse rising edges to advance the motor
PULSE_RISING_EDGE = True
PULSE_PIN = 14
DIR_PIN = 15

# delay between steps in msec
STEP_DELAY_INITIAL_MS = 6  # 20


############### other defines ################
ROT360 = (360 / STEPPER_DEG) * GEARDIV * MICROSTEP
ROT1 = ROT360/360
CCW = 0
CW = 1
HALF_STEP_DELAY_INITIAL_MS = int(STEP_DELAY_INITIAL_MS/2)

############### globals ######################
# built-in LED
boardled = Pin(25, Pin.OUT)
pulse = Pin(PULSE_PIN, Pin.OUT)
direction = Pin(DIR_PIN, Pin.OUT)

def init():
    if PULSE_RISING_EDGE:
        pulse.value(0)
    else:
        pulse.value(1)
    if DIR_POLARITY:
        direction.value(0)
    else:
        direction.value(1)

def step(numpulse):
    for i in range(0,numpulse):
        if PULSE_RISING_EDGE:
            pulse.value(1)
        else:
            pulse.value(0)
        time.sleep_ms(HALF_STEP_DELAY_INITIAL_MS)
        if PULSE_RISING_EDGE:
            pulse.value(0)
        else:
            pulse.value(1)
        time.sleep_ms(HALF_STEP_DELAY_INITIAL_MS)

def rot_deg(deg, rotdir=CW):
    if rotdir==CW:
        if DIR_POLARITY:
            direction.value(0)
        else:
            direction.value(1)
    else:
        if DIR_POLARITY:
            direction.value(1)
        else:
            direction.value(0)
    numpulse = deg * ROT1
    #  print(f"executing {numpulse} pulses")
    step(numpulse)
    #  print("done")
    

    

