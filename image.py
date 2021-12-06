import RPi.GPIO as GPIO

# User Defined Modules
from lib.tft.lib_tft24T import TFT24T
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont
import sys
import spidev

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

DC = 20
RST = 16
LED = 12

TFT = TFT24T(spidev.SpiDev(), GPIO, landscape=False)
TFT.initLCD(DC, RST, LED)

if len(sys.argv) < 2:
    print("Display a image file.\n\nUsage: %s filename.png" % sys.argv[0])
    sys.exit(-1)
    
TFT.load_wallpaper(sys.argv[1])
TFT.display()