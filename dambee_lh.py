from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont
from time import sleep, time

import os
import serial
import signal
import threading
import spidev
import netifaces
import getmac
import RPi.GPIO as GPIO
from datetime import datetime

# User Defined Modules
from lib.tft.lib_tft24T import TFT24T
from lib.inout.inout import gp
from lib.touch.touch import TS20
import command as pcmd
import mysocket 
#import sound
import Input 
import display
from task import *
from util import *
import nfc
#import webrtc
import log
from SockCommand import NetCommand
import schedule

vkey = {'K1' : 1,     'K2' : 2, 'K3' : 4, 
        'K4' : 8,     'K5' : 16, 'K6' : 32,
        'K7' : 64,     'K8' : 256, 'K9' : 512,
        'K*' : 2048,   'K0' : 1024, 'K#' : 4096,
        'Call' : 'None'}
ckey = {1 : '1',     2 : '2', 4 : '3', 
        8 : '4',     16 : '5', 32 : '6',
        64 : '7',    256 : '8', 512 : '9',
        2048 : 'call',   1024 : '0', 4096 : 'enter',
        'Call' : 'None'}
             
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

DC = 20
RST = 16
LED = 12

exitThread = False
line = []
key = ""
bCover = False
netdiscon = False
aliveerror = False
errorcode = 0
ser = serial.Serial('/dev/ttyS0', 115200)

def DEBUGPrint(msg, param1="", param2=""):
  string = "[MAIN]" + str(msg) + str(param1) + str(param2)
  print(string)
  log.Log(string)

def handler(signum, frame):
     exitThread = True

def readThread(ser):
    global line
    global exitThread
    global bCober, netdiscon

    while not exitThread:
      try:
        for c in ser.read():
          line.append(chr(c))
                
          if c == 10:
            tmp = ''.join(line[:-2])
            parsing_str(tmp)
                
            del line[:]
      except Exception as e:
          DEBUGPrint(e)
          
      sleep(0.001)
            


def bleopen():
  global ser

  signal.signal(signal.SIGINT, handler)

  blethread = threading.Thread(target=readThread, args=(ser,), daemon=True)
  blethread.start()

def keyThread():
  global key
  while True:
    key = input(">> ")

######################
# Start Main Program # 
######################
log.Init(True)
pcmd.read()

# GPIO 입출력
DEBUGPrint("입출력 초기화")
inout = gp()
inout.fled(0)

#For LCD TFT SCREEN:
DEBUGPrint("LCD 초기화")
TFT = TFT24T(spidev.SpiDev(), GPIO, landscape=False)
TFT.initLCD(DC, RST, LED)

draw = TFT.draw()

TFT.backlight(pcmd.option['lcd'])
display.DispInit(TFT, draw)
display.DispWait()

# 옵션 설정
DEBUGPrint("사운드 초기화")

# 네트워크 정보 읽기
DEBUGPrint("네트워크 정보 로드")
netinfo = netifaces.ifaddresses('eth0')
temp = netifaces.gateways()
gateway = temp['default'][2]

tmp = netinfo[2]
pcmd.system['mac'] = getmac.get_mac_address()
pcmd.network['ipaddr'] = tmp[0]['addr']
pcmd.network['gateway'] = gateway[0]
pcmd.network['netmask'] = tmp[0]['netmask']

# BLE 연결
bleopen()
sendInitMessage(ser)

sleep(1)

# TOUCH TS20
ts20 = TS20()
DEBUGPrint("터치초기화")

# NFC 
nfc.Init()
DEBUGPrint("카드리더초기화")

task = TASK()
#task = None
# 소켓서버 & 웹서버 연결
DEBUGPrint("소켓서버 연결")
mysocket.Init(pcmd.Server['IP'], pcmd.Server['PORT'], task)
#mysocket.Init('192.168.0.2', 9000, task)
# 네트워크 설정 저장
pcmd.save()

# 현재 입력 상태 체크
Input.Init(inout)
Input.InputCheck()

cur_time = 0
open_time = 0
disp_time = 0
access_time = 0
net_time = 0
alarm_time = 0

cur_time = int(time())
net_time = cur_time
pcmd.alive_time = cur_time

keythread = threading.Thread(target=keyThread, daemon=True)
keythread.start()

nfcbusy = False
keybusy = False
keycnt = 0

keyinput = []

schedule.readSchedule()
tog = 0


error_disp = 0

while True:
  # Door Open 
  if open_time == 0 and pcmd.fOpen == True:
    display.DispOpen()
    pcmd.fOpen = False
    inout.relay(1)
    inout.fled(1)
    open_time = int(time())
    access_time = open_time

    try:
      os.system("aplay -D plughw:1,0 /home/pi/dambee_lh/music/doorOpen.wav &")
    except:
      print("Sound Card 1 detect error!!!")

    DEBUGPrint("Door Open")
    
  if(open_time != 0 and ((cur_time - open_time) >= pcmd.option["opetime"])):
    open_time = 0
    inout.relay(0)
    inout.fled(0)
    DEBUGPrint("Door Close")
    
  if (access_time != 0 and ((cur_time - access_time) >= 5)):
    access_time = 0
    if task.task == TASK_CALLING:
      display.DispCalling()
    else:
      display.DispWait()
    DEBUGPrint("Display Init...")
    
  # Invalid User Check
  if pcmd.fFail == True and disp_time == 0:
    display.DispFail()
    pcmd.fFail = False
    disp_time = int(time())
    
  # Retry Disp
  if pcmd.fRetry == True and disp_time == 0:
    display.DispRetry()
    try:
      os.system("aplay -D plughw:1,0 /home/pi/dambee_lh/music/retry.wav &")
    except:
      DEBUGPrint("Sound Card 1 detect error!!!")     
    pcmd.fRetry = False
    disp_time = int(time())
  
  # Call Fail Disp
  if pcmd.fCallFail == True and disp_time == 0:
    display.DispCallFail()
    pcmd.fCallFail = False
    disp_time = int(time())
  
  if (disp_time !=0 and (cur_time - disp_time) >= 3):
    display.DispWait()
    disp_time = 0
    
  #if netdiscon == False:
  # Touch Key
  if task.task == TASK_IDLE and open_time == 0 and nfcbusy == False and netdiscon == False:
    touch = ts20.getTouch()
    
    if touch != 0 and keybusy == False:
      #keycnt = keycnt + 1
      #if keycnt == 5:
        keybusy = True
        DEBUGPrint(GetKey(vkey, touch))
        if touch == vkey['K*']:
          disp_time = 0
              
          task.task = TASK_REQUEST_CALL 
        elif touch == vkey['K#']:
          print(keyinput)
          os.system("aplay -D plughw:1,0 /home/pi/dambee_lh/music/click.wav &")
          if keyinput == ['3','2','1','4','8','6','9','8']:
              display.DispTemp()
          elif keyinput == [] and display.dispnum == 6:
              display.DispWait()
              
          keyinput = []
        else:
            try:
                keyinput.append(ckey[touch])      
                os.system("aplay -D plughw:1,0 /home/pi/dambee_lh/music/click.wav &")
            except:
              pass
    elif touch == 0:
      keybusy = False   
      keycnt = 0
                
  # NFC Check 
  if task.task == TASK_IDLE and open_time == 0 and netdiscon == False:
    try:
      uid = nfc.getUID()
      #uid = None    
      if uid is not None:
        if nfcbusy == False:
          k = "".join(["%02X" % i for i in uid])
          l = k.replace("0x","")
          os.system("aplay -D plughw:1,0 /home/pi/dambee_lh/music/beep.wav")
          DEBUGPrint("UID :", l)
          nfcbusy = True
          if len(l) > 12:
            mysocket.webcmd[7]["cardNumber"] = l[0:12]
          elif len(l) < 12:
            mysocket.webcmd[7]["cardNumber"] = "00AABBCCDDEE"#"0000"+l
          else:
            mysocket.webcmd[7]["cardNumber"] = l
          mysocket.SendMessage(7)
          
      else:
        nfcbusy = False
    except Exception as e:
      DEBUGPrint("NFC except:", e)
        
  # Socket Command
  NetCommand(pcmd, task, TFT)
     
  # Send Auth. Key Command
  if pcmd.fAuth == True:
      pcmd.fAuth = False
      DEBUGPrint("서버인증요청")
      mysocket.SendMessage(1)
      
  if pcmd.ferror != 0:
    sendErrorMessage(ser, pcmd.ferror-1)
    pcmd.ferror = 0
  
  if task.task == TASK_IDLE and (cur_time - net_time) >= 1:      
    now = datetime.now()
    t = now.strftime('%H%M%S')
    if t == '000000':
      schedule.readSchedule()
      DEBUGPrint("Read Schedule")
          
    schedule.checkSchedule()
    if schedule.fStart == True:
      inout.relay(1)
      inout.fled(1)
      schedule.fStart = False
    if schedule.fEnd == True:
      inout.relay(0)
      inout.fled(0)
      schedule.fEnd = False

    # Cover Check
    if pcmd.option['brkyn'] == 0:
        if Input.InputState['Cover'] == 1 and bCover == False:          
          bCover = True
          alarm_timer = 0
          error_disp |= 1          
          errorcode = 3 
          mysocket.SendMessage(3, "3")
        elif Input.InputState['Cover'] == 0 and bCover == True:
          bCover = False
          if netdiscon == False and mysocket.bRetryCon == False:
              display.DispWait()
          errorcode = 0
          mysocket.SendMessage(3, "0")

    if pcmd.option['brkyn'] == 1 and bCover == True:
        bCover = False
                     
    # Check Network Disconnection       
    network = mysocket.CheckNetwork()    
    if network == '127.0.0.1' and netdiscon == False:
        netdiscon = True
        error_disp |= 2        
    elif network != '127.0.0.1' and netdiscon == True:
        netdiscon = False
        if bCover == False and mysocket.bRetryCon == False:
          display.DispWait()
        
    # Check Socket Disconnection
    if  mysocket.connected == False:
        DEBUGPrint("접속이 끊겼습니다.")    
        mysocket.exitThread = False
        if mysocket.bRetryCon == False:
            error_disp |= 4            
            mysocket.bRetryCon = True          
        mysocket.Init(pcmd.Server['IP'], pcmd.Server['PORT'], task) 
        #mysocket.Init('192.168.0.2', 9000, task)
    else:
        if mysocket.bRetryCon == True:
            mysocket.bRetryCon = False
            if netdiscon == False and bCover == False and aliveerror == False:
              display.DispWait()
              
    if pcmd.ferrorcnt >= 2:
          aliveerror = True
    elif pcmd.ferrorcnt == 0:
          if aliveerror == True:
              aliveerror = False 
              if netdiscon == False and bCover == False and mysocket.bRetryCon == False:
                  display.DispWait()
            
    if bCover == True:
        if display.dispnum != 3:
            display.DispCoverOpen()
            error_disp = 2
    elif netdiscon == True: 
        if display.dispnum != 4:
            display.DispNetworkDiscon()
            error_disp = 3
    elif mysocket.bRetryCon == True or aliveerror == True:
        if display.dispnum != 5:
            display.DispSocketDiscon()
            error_disp = 4 
                                                   
    net_time = cur_time
  
  if bCover == True:
    if (cur_time - alarm_time) >= 4:
        os.system("aplay -D plughw:2,0 /home/pi/dambee_lh/music/emergency.wav")
        alarm_time = cur_time
        
  if (cur_time - pcmd.alive_time) >= 60:
    pcmd.alive_time = cur_time
    pcmd.ferrorcnt = pcmd.ferrorcnt + 1
    errmsg = "%d" %(errorcode)
    mysocket.SendMessage(3, errmsg)
    DEBUGPrint("+++++++++++ Alive Message")
      
  # Command Line Instruction
  key = cli(key, inout, task, TFT)
  if key == "quit": break
  

  
  cur_time = int(time())    
  
  sleep(0.001)

DEBUGPrint("접속 종료")

mysocket.Close()
inout.close()
#webrtc.stop()
