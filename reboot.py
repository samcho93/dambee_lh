import RPi.GPIO as GPIO
import os

def force_reset(channel):
    os.system("sudo reboot")

GPIO.setmode(GPIO.BCM)
GPIO.setup(24, GPIO.IN, pull_up_down = GPIO.PUD_UP)

GPIO.add_event_detect(24, GPIO.FALLING)
GPIO.add_event_callback(24, force_reset)

while True:
  pass