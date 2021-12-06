import os

import command as pcmd
import mysocket 
import Input 
import display
from util import *
from task import *
import log
#import webrtc
  
def DEBUGPrint(msg, param1="", param2=""):
  string = "[UTIL]" + str(msg) + str(param1) + str(param2)
  print(string)
  log.Log(string)

def GetKey(dic, key):
  index = 0
  name = []
  try:
    for k in dic.keys():
      name.append(k)
  
    for v in dic.values():
      if v == key:
        break
      index += 1
    
    return ("[Key] " + name[index])
  except:  
    return 0
  
def parsing_str(cmd):
  if cmd.find('setsyn') == 8:
    pcmd.setsync(cmd)
  elif cmd.find('setope') == 8:
    pcmd.dooropen(cmd)
  elif cmd.find('setopt') == 8:
    pcmd.setopt(cmd)
  elif cmd.find('setnet') == 8:
    pcmd.setnet(cmd)
  elif cmd.find('setpng') == 8:
    DEBUGPrint("Ping")
  elif cmd.find('setrst') == 8:
    DEBUGPrint("System Reset")
  elif cmd.find('setini') == 8:
    DEBUGPrint("Factory Reset")
    pcmd.fExit = True
  elif cmd.find('getble') == 8:
    pcmd.getble(cmd)
    iv = pcmd.ble["enckey"]
    blemac = pcmd.ble["mac"].replace(":","")
    mysocket.key = "20"+blemac+iv+"21"
    mysocket.iv = iv
    DEBUGPrint("key : ", mysocket.key)
    DEBUGPrint("iv : ", mysocket.iv)
  
def cli(key, inout, task, TFT):
  if key != "":  
    if key == 'quit':
      return "quit"
    elif key == "open":
      pcmd.fOpen = True
      open_time = 0
    elif key == "invalid":
      pcmd.fFail = True
      disp_time = 0
    elif key == "option":
      DEBUGPrint(pcmd.option)
    elif key == "network":
      DEBUGPrint(pcmd.network)
    elif key == "system":
      DEBUGPrint(pcmd.system)
    elif key == "setopentime":
      t = int(input("opentime ? "))
      pcmd.option["opetime"] = t
      DEBUGPrint(pcmd.option)
    elif key == "setvol":
      v = int(input("volume ? "))
      pcmd.option["spk"] = v
      #sound.SetVol(v/3)
      DEBUGPrint(pcmd.option)
    elif key == 'bright':
      b = int(input("Bright ? "))
      if b > 5: b = 5
      if b < 0: b = 0
      pcmd.option["lcd"] = b
      
      TFT.backlight(b)
      DEBUGPrint(pcmd.option)
    elif key == "save":
      pcmd.save()
      DEBUGPrint("Saved Complete!")
    #elif key == "start":
      #webrtc.start(mysocket.roomnumber)
    elif key == "stop":
      #webrtc.stop()
      print("WebRTC Restart")
      os.system("sudo service uv4l_raspicam restart")
    elif key == "gled":
      inout.fled(1)
    elif key == "rled":
      inout.fled(2)
    elif key == "ledoff":
      inout.fled(0)     
    elif key == 'pwm':
      v = int(input("pwm "))
      inout.pwmgled(v)
    #elif key == "soundoff":
      #sound.Quit()      
    #elif key == "soundinit":
    #  sound.Init(pcmd.option['spk'] / 3)
    elif len(key) == 2 and key[0] == 's':
      mysocket.SendMessage(int(key[1]))
    elif key == "temp":
      os.system("vcgencmd measure_temp")
    elif key == "call":
      if task.task == TASK_IDLE:
        task.task = TASK_REQUEST_CALL
        DEBUGPrint("Request Calling....")
    elif key == "idle":
      task.task = TASK_IDLE
      mysocket.cmd = ""
      DEBUGPrint("Forced Idle Mode")
      display.DispWait()
    elif key == 'shutdown':
      os.system("sudo shutdown -h now")  
    elif key == 'reboot':
      os.system("sudo reboot")
    else :
      display.DispTest(key)
    key = ""
    
  return key