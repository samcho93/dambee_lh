#-*- coding:utf-8 -*-

from socket import *
import threading
from time import time, sleep
import json
import requests
import re
import command as pcmd
from task import *
from pycrpyto import AESCryptoCBC
import log
import urllib.request
import util

exitThread = False
connected = False
bRetryCon = False

cmd = ""
result = ""
dicdata = {}
iv = ""
key = ""
port = 0
ip = ""
roomnumber = 0
callsn = 0
task = 0
webcmd = []
msgnum = 0
recvData = ""

response = {"responseId" : "", "errorCode" : 0}

url = "https://yeolligo.com/setAtypeFcltInstall.do"
#url = "http://192.168.50.56:7002/setAtypeFcltInstall.do"
headers= {'Content-Type': 'application/json; charset=utf-8'}
datas = {
  "macAddr":"",
  "fcltIp":"",
  "fcltMapY":"126.5423523",
  "fcltMapX":"36.241241"
  }
  
  
def DEBUGPrint(msg, param1="", param2=""):
  string = "[MYSOCKET]" + str(msg) + str(param1) + str(param2)
  print(string)
  log.Log(string)
  
def receive(sock):
    global exitThread
    global connected
    global cmd
    global dicdata
    global recvData
    global result
    global iv
    global key
    global task

    while exitThread == False:
        try:
          recvData = sock.recv(1024)
          error = False
        except:
          error = True
          DEBUGPrint("Socket Receved Error!!!")
        
        try:
          if error == False:
            aes2 = AESCryptoCBC(key, iv)
            dec = aes2.decrypt(recvData)
                   
            DEBUGPrint("dec : ", dec)
            DEBUGPrint("recvData : ", recvData)
            
            string = str(dec)
            #string = str(recvData)#.decode('utf-8')
            if recvData == b'':
                exitThread = True
                connected = False
                sock.close()
            elif string[0] != '{' and string[:-1] != '}':
                DEBUGPrint("Error Data : ", str(recvData), string)
                result = "error"
                #task.task = task.TASK_IDLE
                #cmd = ""
                #DEBUGPrint("Forced Idle Mode in Socket Thread!!")
            else:
               
                dicdata = json.loads(dec)
                #dicdata = json.loads(recvData)
                DEBUGPrint('dicdata : ', dicdata)
                
                if "method" in dicdata:
                    cmd = dicdata["method"]
                    DEBUGPrint("Method ", cmd)
                elif "resulttime" in dicdata:
                    result = recvData
                    DEBUGPrint("resulttime : ", dicdata["resulttime"])
        except Exception as e:
          print(e)
                          
        sleep(0.001)

    DEBUGPrint("?????????????????? ???????????????")

def encdata(data):
    global clientSock
    global key, iv
    
    string = json.dumps(data)
    DEBUGPrint(string)
    #clientSock.send(string.encode('utf-8'))
    aes = AESCryptoCBC(key, iv)        
    enc = aes.encrypt(string)
    clientSock.send(enc)

def webpost():
    global url, datas, headers

    r = requests.post(url, json=datas, headers=headers)
    DEBUGPrint(r.json())
 
def Init(server_ip, server_port, tsk):
    global ip, port
    global clientSock
    global connected
    global webcmd
    global task

    task = tsk
    
    req = requests.get("http://ipconfig.kr")   
    datas["fcltIp"] = re.search(r'IP Address : (\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})', req.text)[1]
    #datas["fcltIp"] = pcmd.network["ipaddr"]
    datas["macAddr"] = pcmd.ble["mac"]
    
    with open("/home/pi/dambee_lh/webcmd.json", "r") as wcmd:
      try:
        webcmd = json.load(wcmd)
        webcmd = webcmd["webcmd"]
      except:
        DEBUGPrint("<webcmd.json> file not found")
    
    for i in range(8):
      if "macAddr" in webcmd[i]: 
        webcmd[i]["macAddr"] = pcmd.ble["mac"]  
    #DEBUGPrint(webcmd)    
    
    port = server_port
    ip = server_ip
    clientSock = socket(AF_INET, SOCK_STREAM)
    clientSock.settimeout(5)
    
    try:
        clientSock.connect((ip, port)) # yeolligo-soc.com
        connected = True
        DEBUGPrint('?????? ??????')
        clientSock.settimeout(None)

        #sender = threading.Thread(target=send, args=(clientSock,))
        receiver = threading.Thread(target=receive, args=(clientSock,))
        #sender.start()
        receiver.daemon = True
        receiver.start()
        
        webpost()
        SendMessage(0)
    except:
        DEBUGPrint("????????? ?????? ???????????????")
               
def CheckNetwork():
    try:
        s = socket(AF_INET, SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        return ip
    except:
        return '127.0.0.1'
                
def SendMessage(sel, param=0, ser=None):
    try:
        global result, roomnumber, webcmd, task
        global iv, key
        global recvData
        
        msgnum = sel

        if sel == 3:
          webcmd[sel]['errorCode'] = param
          webcmd[sel]['orgnztSn'] = pcmd.system["orgnztSn"]
          
        if "videocallSn" in webcmd[sel]:
            webcmd[sel]["videocallSn"] = callsn
          
        if "fcltSn" in webcmd[sel]:
            webcmd[sel]["fcltSn"] = pcmd.system["fcltsn"]
          
        DEBUGPrint("## Send Message : ", sel)
        DEBUGPrint("orig : ", webcmd[sel])
        
        da = str(webcmd[sel])        
        da = json.dumps(da)
        da = da.replace("\'","\"")
        da = da[1:-1]
        #print("dump : ", da)
        
        result = ""
        aes = AESCryptoCBC(key, iv)
        enc = aes.encrypt(da)             
        clientSock.send(enc)
        
        aes2 = AESCryptoCBC(key, iv)
        dec = aes2.decrypt(enc)
        print("*** dec : ", dec)
        
        
        #recvData = clientSock.recv(1024)
        timeout = int(time())
        
        while result == "":
          cur_time = int(time())
          if (cur_time - timeout) > 3: 
            DEBUGPrint("Socket Timeout")
            timeout = -1
            task.task = 0
            break 
    
        if timeout != -1:
          recv = result
          #DEBUGPrint('from Server > ', recvData)
      
          #??? ??????
          # Decoding
          if recv is not "error":
            aes2 = AESCryptoCBC(key, iv)
            dec = aes2.decrypt(recv)
            dic = json.loads(dec)
            
            DEBUGPrint('Decoding : ', dec)
            pcmd.alive_time = int(time())
            pcmd.ferrorcnt = 0
            
            if "fcltSn" in dic:
              pcmd.system["fcltsn"] = dic["fcltSn"]
            elif "orgnztSn" in dic:
              pcmd.system["orgnztSn"] = dic["orgnztSn"]
            elif "authKey" in dic:
              #dic["authKey"] = "1"
              if dic["authKey"] == pcmd.system["AuthKey"]:
                if pcmd.fOpen == False:
                  pcmd.fOpen = True
                pcmd.system["AuthKey"] = '1'
                DEBUGPrint("Valid Authkey...")
                pcmd.ferror = 1
              else:
                DEBUGPrint("Invalid Authkey...")
                pcmd.ferror = 2 
            
            if webcmd[sel]['method'] == "reqFingerCardCheck": 
              if dic['errorCode'] == 0:
                if pcmd.fOpen == False :
                  pcmd.fOpen = True
    
            DEBUGPrint("Call taskProcess")
            task.taskProcess(dic)
          else:
            dicdata = json.loads(recvData)
            if dicdata['errorCode'] == 999:
                DEBUGPrint("Error : ", dicdata)
                if task.task == TASK_REQUEST_CALL_RESULT_WAIT:
                    task.task = TASK_REQUEST_CALL_RESULT
      
          result = ""
    except:
        DEBUGPrint("SendMessage Error")
        
            
def Close():
    global clientSock

    clientSock.close()

