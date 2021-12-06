import RPi.GPIO as GPIO
from lib.nfc.pn532 import *
import threading

uid = ""
isBusy = False
isRead = False

pn532 = ""

def nfcThread(pn532):
    global uid
    while True:
        uid = pn532.read_passive_target(timeout=0.5)
        # Try again if no card is available.
        if uid is None:
            continue
        uid = [hex(i) for i in uid]
        print(uid)

def Init():
    global pn532
    
    try:
        pn532 = PN532_SPI(debug=False, reset=None, cs=7)

        ic, ver, rev, support = pn532.get_firmware_version()
        #print('Found PN532 with firmware version: {0}.{1}'.format(ver, rev))

        # Configure PN532 to communicate with MiFare cards
        pn532.SAM_configuration()

        #print('Waiting for RFID/NFC card...')

        #nfcthread = threading.Thread(target=nfcThread, args=(pn532, ), daemon=True)
        #nfcthread.start()
    except Exception as e:        
        print(e)
def Initspi():
    #global pn532
    
    #pn532.reinit()
    pass
def close():
    global pn532
    
    pn532.close()
    
def getUID():
    global pn532, uid
    
    try:
      uid = pn532.read_passive_target(timeout=0.1)
   
      return uid
    except:
     return None
