import RPi.GPIO as GPIO

InputState = {
    'In0' : 0, 'In1' : 0, 'Cover' : 0, 
    'Fault' : 0, 'Charge' : 0, 'PowerFail' : 0,
    'ForceReset' : 0
}
inout = []

def DEBUGPrint(msg, param1="", param2=""):
  print("[INOUT]", msg, param1, param2)

def in0High(channel):
    global inout
    InputState['In0'] = GPIO.input(inout.in0)
    
    if InputState['In0'] == 0: DEBUGPrint("입력1 On")
    else: DEBUGPrint("입력1 Off")

def in1High(channel):
    global inout
    InputState['In1'] = GPIO.input(inout.in1)
    
    if InputState['In1'] == 0: DEBUGPrint("입력2 On")
    else : DEBUGPrint("입력2 Off")
    
def cover_open(channel):
    global inout
    InputState['Cover'] = GPIO.input(inout.hallsen)
    
    if InputState['Cover'] == 1: DEBUGPrint("커버가 열렸습니다.")
    else: DEBUGPrint("커버가 닫혔습니다")

def force_reset(channel):
    global inout
    DEBUGPrint("강제 리셋을 하시겠습니까?")
    
def InputCheck():
    global inout
    InputState['In0'] = GPIO.input(inout.in0)
    InputState['In1'] = GPIO.input(inout.in1)
    InputState['Cover'] = GPIO.input(inout.hallsen)
    InputState['Fault'] = GPIO.input(inout.faultState)
    InputState['Charge'] = GPIO.input(inout.charge)
    InputState['PowerFail'] = GPIO.input(inout.powerFail)
    DEBUGPrint('입력상태 : ', InputState)

def Init(gp):
    global inout

    inout = gp
    GPIO.add_event_detect(inout.in0, GPIO.BOTH, bouncetime=50)
    GPIO.add_event_callback(inout.in0, in0High)

    GPIO.add_event_detect(inout.in1, GPIO.BOTH)
    GPIO.add_event_callback(inout.in1, in1High)

    GPIO.add_event_detect(inout.hallsen, GPIO.BOTH)
    GPIO.add_event_callback(inout.hallsen, cover_open)

    GPIO.add_event_detect(inout.forceReset, GPIO.FALLING)
    GPIO.add_event_callback(inout.forceReset, force_reset)
    