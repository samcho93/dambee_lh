import command as cmd
import mysocket 
#import webrtc
#import sound
from task import *
import log
import json
import schedule

getDeciceOption = {
    "responseId" : "",
    "opetime" : 3,
    "lcd" : 3,
    "lcdtxt" : 3,
    "spk" : 3,
    "mic" : 3,
    "bluepwr" : 3,
    "brkyn" : 3,
    "errorCode" : 0
}

response = {"responseId" : "", "errorCode" : 0}
def DEBUGPrint(msg, param1="", param2=""):
  string = "[SOCKETCMD]" + str(msg) + str(param1) + str(param2)
  print(string)
  log.Log(string)
  
def NetCommand(pcmd, task, TFT):
    global getDeciceOption, response

    if mysocket.cmd == "getDeviceOption":
        mysocket.cmd = ""
        getDeciceOption["responseId"] = mysocket.dicdata["responseId"]
        getDeciceOption["opetime"] = pcmd.option["opetime"]
        getDeciceOption["lcd"] = pcmd.option["lcd"]
        getDeciceOption["lcdtxt"] = pcmd.option["lcdtxt"]
        getDeciceOption["spk"] = pcmd.option["spk"]
        getDeciceOption["mic"] = pcmd.option["mic"]
        getDeciceOption["bluepwr"] = pcmd.option["bluepwr"]
        getDeciceOption["brkyn"] = pcmd.option["brkyn"]     

        try:
          mysocket.encdata(getDeciceOption)
          DEBUGPrint("> ", getDeciceOption)
        except Exception as e:
          DEBUGPrint("NetCommand error : getDeciceOption\n", e)
        
    elif mysocket.cmd == "setDeviceOption":
        mysocket.cmd = ""
        pcmd.option["opetime"] = mysocket.dicdata["opetime"] 
        pcmd.option["lcd"] = mysocket.dicdata["lcd"]
        pcmd.option["lcdtxt"] = mysocket.dicdata["lcdtxt"]
        pcmd.option["spk"] = mysocket.dicdata["spk"]
        pcmd.option["mic"] = mysocket.dicdata["mic"]
        pcmd.option["bluepwr"] = mysocket.dicdata["bluepwr"]
        pcmd.option["brkyn"] = mysocket.dicdata["brkyn"]
        response["responseId"] = mysocket.dicdata["responseId"]

        try:
          mysocket.encdata(response)
          pcmd.save()
          pcmd.read()
          TFT.backlight(pcmd.option['lcd'])
          DEBUGPrint("> ", response)
        except Exception as e:
          DEBUGPrint("NetCommand error : setDeviceOption\n", e)
    elif mysocket.cmd == "setInit":
        try:
          mysocket.cmd = ""
          response["responseId"] = mysocket.dicdata["responseId"]          

          mysocket.encdata(response)
          DEBUGPrint("> ", response)
        except Exception as e:
          DEBUGPrint("NetCommand error : setInit\n", e)
    elif mysocket.cmd == "setReboot":
        try:
          mysocket.cmd = ""
          response["responseId"] = mysocket.dicdata["responseId"]
  
          mysocket.encdata(response)
          DEBUGPrint("> ", response)
          os.system("sudo reboot")
        except Exception as e:
          DEBUGPrint("NetCommand error : setReboot\n", e)
    elif mysocket.cmd == "setSchedule":
        try:
          mysocket.cmd = ""
          response["responseId"] = mysocket.dicdata["responseId"]
          print(mysocket.dicdata["schedule"])
          
          with open("/home/pi/dambee_lh/schedule.json","w", encoding='utf-8') as schd:
            json.dump(mysocket.dicdata, schd, indent="\t")
            
          schd.close()
          
          schedule.readSchedule()
          
          mysocket.encdata(response)
          DEBUGPrint("> ", response)
        except Exception as e:
          DEBUGPrint("NetCommand error : setSchedule\n", e)
          
    elif mysocket.cmd == "deleteSchedule":
        try:
          mysocket.cmd = ""
          response["responseId"] = mysocket.dicdata["responseId"]
          
          schd = open("/home/pi/dambee_lh/schedule.json","w", encoding='utf-8') 
          schd.close()
  
          mysocket.encdata(response)
          DEBUGPrint("> ", response)
        except Exception as e:
          DEBUGPrint("NetCommand error : deleteSchedule\n", e)
    elif mysocket.cmd == "videoCallComplate":
        try:
          os.system("killall -9 aplay")
          
          mysocket.cmd = ""
          response["responseId"] = mysocket.dicdata["responseId"]       
          
          mysocket.encdata(response)    
          DEBUGPrint("> ", response)
          
          display.DispCalling()
            
          task.task = TASK_VIDEOCALL_COMPLETE
        except Exception as e:
          DEBUGPrint("NetCommand error : videoCallComplate\n", e)                                 
    elif mysocket.cmd == "openNow":
        try:
          if cmd.fOpen == False:
              cmd.fOpen = True
          mysocket.cmd = ""
          response["responseId"] = mysocket.dicdata["responseId"]
          
          mysocket.encdata(response)
          DEBUGPrint("> ", response)
          
          task.task = TASK_CALL_OPEN
        except Exception as e:
          DEBUGPrint("NetCommand error : openNow\n", e)          
    elif mysocket.cmd == "callTerminate":
        try:
          mysocket.cmd = ""
          response["responseId"] = mysocket.dicdata["responseId"]
                  
          mysocket.encdata(response)
          DEBUGPrint("> ", response)    
          
          if task.task != TASK_CALL_OPEN:
            #display.DispWait()
            cmd.fCallFail = True       
          #webrtc.stop()
          task.task = TASK_CALL_TERMINATE_WAIT
        except Exception as e:
          DEBUGPrint("NetCommand error : callTerminate\n", e)     
                 
    