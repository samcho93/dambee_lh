from datetime import datetime
import json
import log
import os
import subprocess 
import re

sysinfo = {}

system = {'fcltsn' : '1023', 'frmver' : '1.0.0.1', 'plfmaddr' : 'lhhome.com/lhhows', 'mac' : 'AA:BB:CC:DD:EE:DD', 'AuthKey' : '1', 'orgnztSn' : '1'}
network = {'ipaddr' : '', 'gateway' : '', 'netmask' : '', 'dns' : ''}
option = {'interval' : 15, 'opetime' : 5, 'lcd' : 3, 'lcdtxt' : 1, 'spk' : 1, 'mic' : 1, 'bluepwr' : 1, 'brkyn' : 0}

ble = {'enckey' : "", 'mac' : 'AA:BB:CC:DD:EE:DD'}
Server = {'IP' : "", 'PORT' : 9000}

fOpen = False
fFail = False
fExit = False
fAuth = False
fRetry = False
fCallFail = False

def DEBUGPrint(msg, param1="", param2=""):
  string = "[COMMAND]" + str(msg) + str(param1) + str(param2)
  print(string)
  log.Log(string)
  
def save():
  global system, network, option, ble, Server
  global sysinfo
  
  sysinfo["System"] = system
  sysinfo["Network"] = network
  sysinfo["Option"] = option
  sysinfo["Server"] = Server
  sysinfo["BLE"] = ble
  
  with open("/home/pi/dambee_lh/sysinfo.json","w", encoding='utf-8') as sfile:
    json.dump(sysinfo, sfile, indent="\t")
    
  sfile.close()
    
  DEBUGPrint("Write System Configuration File")
  
def read():
  global sysinfo, tft
  global system, network, option, ble, Server
  
  with open("/home/pi/dambee_lh/sysinfo.json", "r") as info:
    sysinfo = json.load(info)
    
  info.close()
  
  system = sysinfo["System"] 
  network = sysinfo["Network"]
  option = sysinfo["Option"]
  Server = sysinfo["Server"]
  
  a = subprocess.getoutput('amixer scontrols -c 2')
  b = a.split('\n')

  c = b[0].split()
  d = c[3].split(',')
  d = d[0].replace("'", '')
  
  per = ['50%', '70%', '100%']
  
  vol = "%s" %(per[option['spk']-1])
  subprocess.call(['amixer', '-c', '2', 'sset', d, vol])
  
  c = b[1].split()
  d = c[3].split(',')
  d = d[0].replace("'", '')

  vol = "%s" %(per[option['mic']-1])
  subprocess.call(['amixer', '-c', '2', 'sset', 'Mic', vol])
    
  DEBUGPrint("Read System Configuration File")
    
def setsync(cmd):
  DEBUGPrint(cmd)
  
#+067cmd:setope,authkey:0123456789abcdef,gender:0,reqtime:20210505184020
def dooropen(cmd):
  global fOpen, fFail, fAuth

  DEBUGPrint(cmd)
  DEBUGPrint("#"*50)
  DEBUGPrint("> CMD : door open")
  tmp = cmd.replace(',', ' ')
  lst = tmp.split()
  
  tmp = lst[1]
  loc = tmp.find('authkey') + 8
  system["AuthKey"] = tmp[loc:]
  DEBUGPrint('> Authkey : ', system["AuthKey"])
  
  tmp = lst[2]
  loc = tmp.find('gender') + 7
  if tmp[loc] == '1':
    DEBUGPrint('> Gender : Women')
  else:
    DEBUGPrint('> Gender : Man')
    
  tmp = lst[3]
  loc = tmp.find('reqtime') + 8
  cur = tmp[loc:]
  DEBUGPrint('> Request Time : ', tmp[loc:])
  
  tmp = lst[5]
  loc = tmp.find('pw1') + 4
  DEBUGPrint("> pw1 : ", tmp[loc:])
  
  tmp = lst[6]
  loc = tmp.find('pw2') + 4
  DEBUGPrint("> pw2 : ", tmp[loc:])
  
  tmp = lst[7]
  loc = tmp.find('pw3') + 4
  DEBUGPrint("> pw3 : ", tmp[loc:]) 
  
  now = datetime.now()
  DEBUGPrint('> Current Time : ', now.strftime('%Y%m%d%H%M%S'))

  dc = datetime.strptime(cur, '%Y%m%d%H%M%S')
  diff = now - dc
  DEBUGPrint("Respons Time(sec) : ", diff.seconds,"(s)")
  DEBUGPrint("#"*50)

  if diff.seconds < 20:
    fAuth = True
    DEBUGPrint("BLE인증완료")
#    fOpen = True
#  else:
#    fFail = True

#+089cmd:setnet,ipaddr:200.24.21.11,gateway:200.24.21.1,netmask:255.255.255.0,dns:168.126.63.1      
def setnet(cmd):
  global network

  DEBUGPrint("#"*50)
  DEBUGPrint("> CMD : Set Network")
  tmp = cmd.replace(',', ' ')
  lst = tmp.split()

  tmp = lst[1]
  loc = tmp.find('ipaddr') + 7
  network['ipaddr'] = tmp[loc:]
  
  tmp = lst[2]
  loc = tmp.find('gateway') + 8
  network['gateway'] = tmp[loc:]

  tmp = lst[3]
  loc = tmp.find('netmask') + 8
  network['netmask'] = tmp[loc:]

  tmp = lst[4]
  loc = tmp.find('dns') + 4
  network['dns'] = tmp[loc:]
  DEBUGPrint(network)
  DEBUGPrint("#"*50)

  save()

#+078cmd:setopt,interval:15,opetime:55,lcd:3,lcdtxt:1,spk:1,mic:1,bluepwr:1,brkyn:0  
def setopt(cmd):
  global option
  
  DEBUGPrint("#"*50)
  DEBUGPrint("> CMD : Set Option")
  tmp = cmd.replace(',', ' ')
  lst = tmp.split()

  tmp = lst[1]
  loc = tmp.find('interval') + 9
  option['interval'] = int(tmp[loc:])

  tmp = lst[2]
  loc = tmp.find('opetime') + 8
  option['opetime'] = int(tmp[loc:])

  tmp = lst[3]
  loc = tmp.find('lcd') + 4
  option['lcd'] = int(tmp[loc:])

  tmp = lst[4]
  loc = tmp.find('lcdtxt') + 7
  option['lcdtxt'] = int(tmp[loc:])

  tmp = lst[5]
  loc = tmp.find('spk') + 4
  option['spk'] = int(tmp[loc:])

  tmp = lst[6]
  loc = tmp.find('mic') + 4
  option['mic'] = int(tmp[loc:])

  tmp = lst[7]
  loc = tmp.find('bluepwr') + 8
  option['bluepwr'] = int(tmp[loc:])

  tmp = lst[8]
  loc = tmp.find('brkyn') + 6
  option['brkyn'] = int(tmp[loc:])

  DEBUGPrint(option)
  DEBUGPrint("#"*50)

  save()

def getinfo():
  pass

def getble(cmd):
  global ble

  DEBUGPrint("#"*50)
  DEBUGPrint("> CMD : Get BLE Infor.")
  tmp = cmd.replace(',', ' ')
  lst = tmp.split()

  tmp = lst[1]
  loc = tmp.find('enckey') + 7
  ble['enckey'] = tmp[loc:]

  tmp = lst[2]
  loc = tmp.find('mac') + 4
  ble['mac'] = tmp[loc:]
  
  DEBUGPrint(ble)
  DEBUGPrint("#"*50)

  save()

  
