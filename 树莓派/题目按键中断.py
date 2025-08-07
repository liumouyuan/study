#!/usr/bin/env python3
import RPi.GPIO as GPIO
import signal
import sys
import time

KEY_A = 17
KEY_B = 27
GPIO.setmode(GPIO.BCM)
GPIO.setup([KEY_A, KEY_B], GPIO.IN, pull_up_down=GPIO.PUD_UP)

def pressed_a(ch):
    print('a', flush=True)

def pressed_b(ch):
    print('b', flush=True)

GPIO.add_event_detect(KEY_A, GPIO.FALLING,
                      callback=pressed_a, bouncetime=500)
GPIO.add_event_detect(KEY_B, GPIO.FALLING,
                      callback=pressed_b, bouncetime=500)

counter = 1
try:
    while True:
        counter += 1
        #print(counter)
except KeyboardInterrupt:
    pass
finally:
    GPIO.cleanup()
