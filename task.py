import threading
import glob
import os
import mysocket as ms
from time import time, sleep
import command as pcmd
import display
import log
#import webrtc
#import sound

TASK_IDLE                     = 0
TASK_REQUEST_CALL             = 1
TASK_REQUEST_CALL_RESP        = 2
TASK_REQUEST_CALL_RESULT      = 3
TASK_REQUEST_CALL_RESULT_WAIT = 4
TASK_REQUEST_CALL_TIMEOUT     = 5
TASK_REQUEST_CALL_TERMINATE   = 6
TASK_VIDEOCALL_COMPLETE       = 7
TASK_CALLING                  = 8
TASK_CALL_TERMINATE_WAIT      = 9
TASK_CALL_ENTRY_WAIT          = 10
TASK_CALL_OPEN                = 11

def DEBUGPrint(msg, param1="", param2=""):
  string = "[TASK]" + str(msg) + str(param1) + str(param2)
  print(string)
  log.Log(string)
  
class TASK():
  def __init__(self):
    self.task = TASK_IDLE
    self.taskthread = threading.Thread(target=self.taskThread,daemon=True)
    self.taskthread.start()    
    self.timeout = 0
    self.timewait = 0
    self.belltime = 0
      
  def taskThread(self):
    while True:
      if self.task == TASK_REQUEST_CALL:
        try:
          DEBUGPrint("TASK_REQUEST_CALL")       
          self.timewait = int(time()*1000)
          self.task = TASK_REQUEST_CALL_RESP
          ms.SendMessage(4)      
        except:
          DEBUGPrint("task error : TASK_REQUEST_CALL")
          task = TASK_IDLE           
        
      elif self.task == TASK_CALL_ENTRY_WAIT:
        try:
          cur_time = int(time()*1000)
          if((cur_time - self.timewait) >= 6000):
            DEBUGPrint("Request Calling....")
            
            display.DispCallWait()
            self.task = TASK_REQUEST_CALL_RESULT
        except:
          DEBUGPrint("task error : TASK_CALL_ENTRY_WAIT")
          task = TASK_IDLE 
      elif self.task == TASK_REQUEST_CALL_RESULT:
        try:
          DEBUGPrint("TASK_REQUEST_CALL_RESULT")
          
          #os.system("aplay -D plughw:2,0 /home/pi/dambee_lh/music/doorbell.wav &")
          self.task = TASK_REQUEST_CALL_RESULT_WAIT   
          ms.SendMessage(5)
          self.timeout = int(time()*1000)
          self.belltime = self.timeout - 3800    
        except:
          DEBUGPrint("task error : TASK_REQUEST_CALL_RESULT")
          task = TASK_IDLE                     
      elif self.task == TASK_REQUEST_CALL_RESULT_WAIT:
        try:
          cur_time = int(time()*1000)
          if((cur_time - self.timeout) >= 20000):
            DEBUGPrint("videoCallTimeout!!!")
            pcmd.fRetry = True
            #display.DispWait()
            self.task = TASK_REQUEST_CALL_TIMEOUT
          if ((cur_time - self.belltime) >= 4000):
            self.belltime = cur_time
            os.system("aplay -D plughw:1,0 /home/pi/dambee_lh/music/doorbell.wav &")
        except:
          DEBUGPrint("task error : TASK_REQUEST_CALL_RESULT_WAIT")
          task = TASK_IDLE 
      elif self.task == TASK_VIDEOCALL_COMPLETE:
        DEBUGPrint("TASK_VIDEOCALL_COMPLETE")
        
        self.task = TASK_CALLING
        self.timeout = 0
                  
      elif self.task == TASK_REQUEST_CALL_TIMEOUT:
        try:
          DEBUGPrint("TASK_REQUEST_CALL_TIMEOUT")              
          ms.SendMessage(6)   
          self.task = TASK_IDLE
        except:
          DEBUGPrint("task error : TASK_REQUEST_CALL_TIMEOUT")
          task = TASK_IDLE 
      elif self.task == TASK_CALL_TERMINATE_WAIT:  
        self.task = TASK_IDLE
      
      sleep(0.01)
              
  def taskProcess(self, dic):
    DEBUGPrint("taskProcess dic : ", dic)
    if "errorCode" in dic:
      errorCode = dic["errorCode"] 
      if errorCode == 0:
          if self.task == TASK_REQUEST_CALL_RESP:
            try:
              DEBUGPrint("TASK_REQUEST_CALL_RESP")
              if "videocallSn" in dic:
                #ms.roomnumber = dic["roomNumber"]
                ms.callsn = dic["videocallSn"]
                
                #DEBUGPrint("RoomNumber : ", ms.roomnumber) 
                DEBUGPrint("VideoCallSn : ", ms.callsn)
                #webrtc.start(ms.roomnumber)
                display.DispCall()                         
                self.task = TASK_REQUEST_CALL_RESULT            
            except:
              DEBUGPrint("task process error : TASK_REQUEST_CALL_RESP")                            
      else:
        DEBUGPrint("ErrorCode : ", errorCode)        
        
        if self.task == TASK_REQUEST_CALL_RESULT_WAIT:
          display.DispCallFail()          
          sleep(1)
          display.DispWait()
          self.timeout = 0
        elif self.task == TASK_REQUEST_CALL_RESULT:
          display.DispCallFail()          
          sleep(1)
          display.DispWait()
          
          self.timeout = 0
        self.task = TASK_IDLE        
