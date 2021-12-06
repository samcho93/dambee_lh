import serial
import signal
import threading
from time import sleep

#btser = serial.Serial('/dev/ttyS0', 115200)
#debugser = serial.Serial('/dev/ttyAMA1', 115200)
ser = serial.Serial('/dev/ttyS0', 115200)

line = [] 
exitThread = False  

def parsing(data):
  length = len(data)
  num = data[1]
  cmd = data[2]
  if length == num + 4:
    if data[0] == 0x04 and data[num + 3] == 0x03:
      if cmd == 0x3:
        print("Door Open")
        del line[:]
    else:
      del line[:]
      
def parsing_str(cmd):
  if cmd.find('setope') == 8:
    print("door open")
  elif cmd.find('setopt') == 8:
    print("Set Option")
  elif cmd.find('setnet') == 8:
    print("Set Network")
  elif cmd.find('setpng') == 8:
    print("Ping")
  elif cmd.find('setrst') == 8:
    print("System Reset")
  elif cmd.find('setini') == 8:
    print("Factory Reset")
    
def handler(signum, frame):
     exitThread = True

def readThread(ser):
    global line
    global exitThread

    while not exitThread:
        for c in ser.read():
          line.append(chr(c))
          
          #if len(line) > 3:
          #  parsing(line)
          #str = "%02x" %(c)
          #print(str)             
          if c == 10:
            tmp = ''.join(line)
            print(tmp)
            parsing_str(tmp)
            del line[:]
            

  
def bleopen():
  global ser

  signal.signal(signal.SIGINT, handler)
  thread = threading.Thread(target=readThread, args=(ser,))

  thread.start()

if __name__ == "__main__":

  sendInitMessage()
    